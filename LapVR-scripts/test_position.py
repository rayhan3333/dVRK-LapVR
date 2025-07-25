import Amp1394Python as Amp
import rospy, time, math
import numpy as np
from ambf_client import Client
from scipy.spatial.transform import Rotation as R

from ambf_msgs.msg import ObjectState

class BoardController:
    def __init__(self, board_id):
        self.port = Amp.PortFactory("")
        self.board = Amp.AmpIO(board_id)
        self.port.AddBoard(self.board)

        self.board.WriteWatchdogPeriod(0)
        self.board.WritePowerEnable(False)
        self.port.WriteAllBoards()
        self.calibrated = False

    def enable_motors(self):
        self.board.WritePowerEnable(True)
        for i in range(4):
            self.board.WriteAmpEnableAxis(i, True)
        time.sleep(1)
        self.port.WriteAllBoards()

    # get raw encoder data
    def get_encoders(self):
        self.port.ReadAllBoards()
        encoder = [0, 0, 0, 0]
        for i in range(4):
            encoder[i] = self.board.GetEncoderPosition(i)
        return encoder

    # get raw potentiometer data
    def get_pot(self):
        self.port.ReadAllBoards()
        pot = [0, 0, 0, 0]
        for i in range(4):
            pot[i] = self.board.GetAnalogInput(i)
        return pot

    def write_current(self, axis, magnitude):
        if self.calibrated:
            self.board.SetMotorCurrent(axis, magnitude)
            self.port.WriteAllBoards()

    def get_current(self, axis):
        return self.board.GetMotorCurrent(axis)

    def cleanup(self):
        for i in range(4):
            self.board.SetMotorCurrent(i, 32768)
            self.board.WriteAmpEnableAxis(i, False)
        self.board.WritePowerEnable(False)
        self.port.WriteAllBoards()
        self.port.RemoveBoard(self.board.GetBoardId())

# linear mapping for joint positions
def map_value(x, min_input, max_input, min_output, max_output):
    return (x - min_input) / (max_input - min_input) * (max_output - min_output) + min_output

def currentAdjust(error, idx):
    if idx == 2:
        max_error = 0.04
        max_current_change = 4096
        deadband = 0.005
    else:
        max_error = 0.4
        max_current_change = 4096
        deadband = 0.05

    if abs(error) < deadband:
        percent = 0.0
    else:
        norm_error = max(-1.0, min(error / max_error, 1.0))
        # Smooth sigmoid-like scaling
        # percent = math.tanh(2.5 * norm_error)
        percent = norm_error ** 3

    desired_current = int(32768 + max_current_change * percent) & 0xFFFFFFFF
    return desired_current

def calibrate_handle(board, handle_name):
    print(f"Press Enter to stop calibration for {handle_name} and save limits.")
    while True:
        try:
            data = board.get_encoders()
            x_center = data[0]
            y_center = data[1]
            z_center = data[2]
            yaw_center = data[3]
            time.sleep(2)
            if input() == '':
                break
        except KeyboardInterrupt:
            break
    print(f"Calibration stopped. Centers for {handle_name} are saved.")
    print(f"{handle_name} Centers: {x_center} {y_center} {z_center} {yaw_center}\n")
    return x_center, y_center, z_center, yaw_center

def get_contact_force(sensor, tool):
    # Get insertion axis in world frame (z-axis of tool)
    quat = tool._state.pose.orientation
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    R_tool = R.from_quat(quat_list).as_dcm()
    insertion_axis_world = R_tool[:, 2]  # z-axis of tool in world frame

    k_stiffness = 10000.0
    mu = 0.4
    contact_force = np.zeros(3)
    total_insertion_force = 0.0

    for i in range(sensor.get_num_contact_events()):
        obj_name = sensor.get_contact_object_name(i)
        if obj_name is None:
            continue    
        if not any(prefix in obj_name for prefix in contact_objs_prefix):
            continue

        event = sensor.get_contact_event(i)
        data = event.contact_data[0]
        penetration = -data.distance.data
        if penetration <= 0:
            continue

        normal_vec = np.array([data.contact_normal.x,
                               data.contact_normal.y,
                               data.contact_normal.z])
        norm = np.linalg.norm(normal_vec)
        normal_vec /= norm
        contact_force_world = normal_vec * penetration * k_stiffness
        insertion_force = np.dot(contact_force_world, insertion_axis_world)
        total_insertion_force += insertion_force

    for i in range(2):
        if np.isclose(base2.get_joint_vel(i), 0.0, atol=0.1):
            contact_force[i] = 0
        else:
            contact_force[i] = -mu * total_insertion_force * np.sign(base2.get_joint_vel(i))
    contact_force[2] = total_insertion_force

    return contact_force

rospy.init_node('lapvr_node')
board9 = BoardController(9)
_client = Client("lapvr")
_client.connect()

base2 = _client.get_obj_handle('/ambf/env/lapvr2/baselink')
left_finger_ghost = _client.get_obj_handle('/ambf/env/ghosts/lapvr2/left_finger_ghost')
right_finger_ghost = _client.get_obj_handle('/ambf/env/ghosts/lapvr2/right_finger_ghost')
actuator = _client.get_obj_handle('/ambf/env/ghosts/lapvr2/Actuator0')
tool_sensor = _client.get_obj_handle('/ambf/env/lapvr2/tip_sensor')
tool_handle = _client.get_obj_handle('/ambf/env/lapvr2/toolrolllink')

x_center_9, y_center_9, z_center_9, roll_center_9 = calibrate_handle(board9, "right handle")
x_range, y_range, z_range, roll_range = 9780, 9583, 21848, 6182
x_limit_9 = x_center_9 + x_range / 2
y_limit_9 = y_center_9 - y_range / 2
z_limit_9 = z_center_9
roll_limit_9 = roll_center_9 + roll_range / 3

board9.calibrated = True
board9.enable_motors()

grasped = False
position_frozen = False
graspable_objs_prefix = ["Needle", "Thread", "Puzzle", "Cover"]
contact_objs_prefix = ["Plane", "phantom"]
latest_error = {'x': 0.0, 'y': 0.0, 'z': 0.0}
frozen_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

while not rospy.is_shutdown():
    x9, y9, z9, roll9 = board9.get_encoders()
    x_curr_9 = map_value(x9, x_limit_9 - x_range, x_limit_9, 1, -1)
    y_curr_9 = map_value(y9, y_limit_9, y_limit_9 + y_range, 1, -1)
    z_curr_9 = map_value(z9, z_limit_9, z_limit_9 + z_range, 0, 0.17)
    roll_curr_9 = map_value(roll9, roll_limit_9 - roll_range, roll_limit_9, 1, -2.14)
    gripper_9 = map_value(board9.get_pot()[0], 22100, 28000, 1, 0)

    if gripper_9 < 0.05:
        if not grasped:
            sensed = left_finger_ghost.get_all_sensed_obj_names() + right_finger_ghost.get_all_sensed_obj_names()
            for prefix in graspable_objs_prefix:
                matches = [obj for obj in sensed if prefix in obj]
                if matches:
                    actuator.actuate(matches[0])
                    grasped = True
    else:
        actuator.deactuate()
        grasped = False

    # use velocity to predit the position
    predit_pos_0 = base2.get_joint_pos(0) + base2.get_joint_vel(0) * 0.05
    predit_pos_1 = base2.get_joint_pos(1) + base2.get_joint_vel(1) * 0.05
    predit_pos_2 = base2.get_joint_pos(2) + base2.get_joint_vel(2) * 0.05

    # calculate the error
    latest_error['x'] = predit_pos_0 - x_curr_9 
    latest_error['y'] = predit_pos_1 - y_curr_9
    latest_error['z'] = predit_pos_2 - z_curr_9

    # detect contact and sliding
    sliding_detected = False
    contact_force = get_contact_force(tool_sensor, tool_handle)
    if not np.allclose(contact_force, 0.0, atol=1):
        sliding_detected = True

    if sliding_detected:
        if not position_frozen:
            # Store current position ONCE when contact starts
            frozen_position['x'] = base2.get_joint_pos(0)
            frozen_position['y'] = base2.get_joint_pos(1)
            frozen_position['z'] = base2.get_joint_pos(2)
            position_frozen = True

        # Freeze position at contact
        print(frozen_position['x'])
        print(frozen_position['y'])
        print(frozen_position['z'])
        print()
        base2.set_joint_pos(0, frozen_position['x'])
        base2.set_joint_pos(1, frozen_position['y'])
        base2.set_joint_pos(2, frozen_position['z'])
    else:
        position_frozen = False
        # Free to move
        base2.set_joint_pos(0, x_curr_9)
        base2.set_joint_pos(1, y_curr_9)
        base2.set_joint_pos(2, z_curr_9)

    # Always set yaw and gripper
    base2.set_joint_pos(3, roll_curr_9)
    base2.set_joint_pos(5, gripper_9)
    base2.set_joint_pos(6, gripper_9)

    board9.write_current(0, currentAdjust(latest_error['x'], 0))
    board9.write_current(1, currentAdjust(-latest_error['y'], 1))
    board9.write_current(2, currentAdjust(latest_error['z'], 2))


    time.sleep(0.001)
    
board9.cleanup()