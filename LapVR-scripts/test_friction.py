import Amp1394Python as Amp
from simulation_manager import SimulationManager
import rospy
from ambf_client import Client
from ambf_msgs.msg import ObjectState, WorldState
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

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

    def get_encoders(self):
        self.port.ReadAllBoards()
        encoder = [self.board.GetEncoderPosition(i) for i in range(4)]
        return encoder

    def get_pot(self):
        self.port.ReadAllBoards()
        pot = [self.board.GetAnalogInput(i) for i in range(4)]
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
        percent = norm_error ** 3

    desired_current = int(32768 + max_current_change * percent) & 0xFFFFFFFF
    return desired_current

def calibrate_handle(board, handle_name):
    print(f"Press Enter to stop calibration for {handle_name} and save limits.")
    while True:
        try:
            data = board.get_encoders()
            x_center, y_center, z_center, roll_center = data
            time.sleep(2)
            if input() == '':
                break
        except KeyboardInterrupt:
            break
    print(f"Calibration stopped. Centers for {handle_name} are saved.")
    print(f"{handle_name} Centers: {x_center} {y_center} {z_center} {roll_center}\n")
    return x_center, y_center, z_center, roll_center

def get_contact_force_in_tool_frame(sensor, tool):
    if sensor.get_num_contact_events() == 0:
        return np.zeros(3)

    event = sensor.get_contact_event(0)
    data = event.contact_data[0]
    penetration = -data.distance.data
    if penetration < 0:
        return np.zeros(3)

    k_stiffness = 100.0
    normal_vec = np.array([data.contact_normal.x, data.contact_normal.y, data.contact_normal.z])
    normal_vec /= np.linalg.norm(normal_vec)
    force_world = normal_vec * penetration * k_stiffness

    quat = tool._state.pose.orientation
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    R_w2t = R.from_quat(quat_list).as_dcm().T
    force_tool = R_w2t @ force_world
    return force_tool

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

latest_error = {'x': 0.0, 'y': 0.0, 'z': 0.0}
grasped = False
graspable_objs_prefix = ["Needle", "Thread", "Puzzle", "Cover"]
roll_range = 6182
x_range = 9780
y_range = 9583
z_range = 21848

x_center_9, y_center_9, z_center_9, roll_center_9 = calibrate_handle(board9, "right handle")
x_limit_9 = x_center_9 + x_range / 2
y_limit_9 = y_center_9 - y_range / 2
z_limit_9 = z_center_9
roll_limit_9 = roll_center_9 + roll_range / 3

board9.calibrated = True
board9.enable_motors()

while not rospy.is_shutdown():
    data9 = board9.get_encoders()
    x9, y9, z9, roll9 = data9
    x_curr_9 = map_value(x9, x_limit_9 - x_range, x_limit_9, 1, -1)
    y_curr_9 = map_value(y9, y_limit_9, y_limit_9 + y_range, 1, -1)
    z_curr_9 = map_value(z9, z_limit_9, z_limit_9 + z_range, 0, 0.17)
    roll_curr_9 = map_value(roll9, roll_limit_9 - roll_range, roll_limit_9, 1, -2.14)
    gripper9 = map_value(board9.get_pot()[0], 22100, 28000, 1, 0)

    if gripper9 < 0.05:
        if not grasped:
            sensed_object_names = left_finger_ghost.get_all_sensed_obj_names() + right_finger_ghost.get_all_sensed_obj_names()
            for prefix in graspable_objs_prefix:
                matches = [obj for obj in sensed_object_names if prefix in obj]
                if matches:
                    grasped_obj_name = matches[0]
                    actuator.actuate(grasped_obj_name)
                    grasped = True
                    print('Grasping', grasped_obj_name)
    else:
        actuator.deactuate()
        if grasped:
            print('Releasing Grasped Object')
        grasped = False

    predit_pos_0 = base2.get_joint_pos(0) + base2.get_joint_vel(0) * 0.05
    predit_pos_1 = base2.get_joint_pos(1) + base2.get_joint_vel(1) * 0.05
    predit_pos_2 = base2.get_joint_pos(2) + base2.get_joint_vel(2) * 0.05

    latest_error['x'] = predit_pos_0 - x_curr_9
    latest_error['y'] = predit_pos_1 - y_curr_9
    latest_error['z'] = predit_pos_2 - z_curr_9

    # Apply friction-based resistance based on contact force in tool frame
    contact_force_tool = get_contact_force_in_tool_frame(tool_sensor, tool_handle)
    f_x, f_y, f_z = contact_force_tool
    latest_error['x'] += 0.005 * f_x
    latest_error['y'] += 0.005 * f_y

    base2.set_joint_pos(0, x_curr_9)
    base2.set_joint_pos(1, y_curr_9)
    base2.set_joint_pos(2, z_curr_9)
    base2.set_joint_pos(3, roll_curr_9)
    base2.set_joint_pos(5, gripper9)
    base2.set_joint_pos(6, gripper9)

    board9.write_current(0, currentAdjust(latest_error['x'], 0))
    board9.write_current(1, currentAdjust(-latest_error['y'], 1))
    board9.write_current(2, currentAdjust(latest_error['z'], 2))

    time.sleep(0.001)

board9.cleanup()
