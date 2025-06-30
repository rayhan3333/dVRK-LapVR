import Amp1394Python as Amp
from simulation_manager import SimulationManager
import rospy
from ambf_client import Client
from ambf_msgs.msg import ObjectState, WorldState
import time
import math

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

def force_callback(msg):
    global latest_force
    latest_force['x'] = msg.wrench.force.x
    latest_force['y'] = msg.wrench.force.y
    latest_force['z'] = msg.wrench.force.z

rospy.init_node('lapvr_node')
# Initialize the clients and board controllers
board9 = BoardController(9) # right haptic device (board6 is the left haptic device)
_client = Client("lapvr")
_client.connect()
base2 = _client.get_obj_handle('/ambf/env/lapvr2/baselink')
left_finger_ghost = _client.get_obj_handle('/ambf/env/ghosts/lapvr2/left_finger_ghost')
right_finger_ghost = _client.get_obj_handle('/ambf/env/ghosts/lapvr2/right_finger_ghost')
actuator = _client.get_obj_handle('/ambf/env/ghosts/lapvr2/Actuator0')
latest_force = {'x': 0.0, 'y': 0.0, 'z': 0.0}
latest_error = {'x': 0.0, 'y': 0.0, 'z': 0.0}
frozen_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
grasped = False
position_frozen = False
graspable_objs_prefix = ["Needle", "Thread", "Puzzle", "Cover"]
yaw_range = 6182 # hardcoded
x_range = 9780
y_range = 9583
z_range = 21848
rospy.Subscriber('/ambf/env/lapvr2/toolrolllink/State', ObjectState, force_callback)

# Calibrate right handle (board 9)
x_center_9, y_center_9, z_center_9, yaw_center_9 = calibrate_handle(board9, "right handle")

x_limit_9 = x_center_9 + x_range / 2
y_limit_9 = y_center_9 - y_range / 2
z_limit_9 = z_center_9
yaw_limit_9 = yaw_center_9 + yaw_range / 3

# Enable motors after calibration completes
board9.calibrated = True
board9.enable_motors()

while not rospy.is_shutdown():
    # Process data for board 9 (right handle)
    data9 = board9.get_encoders()
    x9 = data9[0]
    y9 = data9[1]
    z9 = data9[2]
    yaw9 = data9[3]
    fx, fy, fz = latest_force['x'], latest_force['y'], latest_force['z']
    
    x_curr_9 = map_value(x9, x_limit_9 - x_range, x_limit_9, 1, -1) 
    y_curr_9 = map_value(y9, y_limit_9, y_limit_9 + y_range, 1, -1)
    z_curr_9 = map_value(z9, z_limit_9, z_limit_9 + z_range, 0, 0.17)
    yaw_curr_9 = map_value(yaw9, yaw_limit_9 - yaw_range, yaw_limit_9, 1, -2.14)
    data9 = board9.get_pot()
    gripper9 = map_value(data9[0], 22100, 28000, 1, 0)

    if  gripper9 < 0.05:
        if not grasped:
            sensed_object_names = left_finger_ghost.get_all_sensed_obj_names()
            sensed_object_names = sensed_object_names + right_finger_ghost.get_all_sensed_obj_names()
            for gon in graspable_objs_prefix:
                matches = [son for son in sensed_object_names if gon in son]
                if matches:
                    grasped_obj_name = matches[0]
                    actuator.actuate(grasped_obj_name)
                    grasped = True
                    print(abs(gripper9))
                    print('Grasping Sensed Object Names', grasped_obj_name)
                    print(f"Actuator Status: {actuator.is_active()}")
    else:
        actuator.deactuate()
        grasped_obj_name = False
        if grasped is True:
            print('Releasing Grasped Object')
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
    if abs(fz) > 300 and (abs(fx) > 50 or abs(fy) > 50):
        sliding_detected = True

    if sliding_detected:
        if not position_frozen:
            # Save frozen position when contact begins
            frozen_position['x'] = base2.get_joint_pos(0)
            frozen_position['y'] = base2.get_joint_pos(1)
            frozen_position['z'] = base2.get_joint_pos(2)
            position_frozen = True

        # Apply resisting effort to keep tool near frozen position
        stiffness = 10.0  # Adjust as needed
        damping = 20.0

        for i, axis in enumerate(['x', 'y', 'z']):
            joint_idx = i  # joint indices: 0, 1, 2
            curr_pos = base2.get_joint_pos(joint_idx)
            curr_vel = base2.get_joint_vel(joint_idx)
            error = frozen_position[axis] - curr_pos
            effort = stiffness * error - damping * curr_vel
            base2.set_joint_effort(joint_idx, effort)
    else:
        position_frozen = False
        # Normal control when not in contact
        base2.set_joint_effort(0, 0)
        base2.set_joint_effort(1, 0)
        base2.set_joint_effort(2, 0)
        base2.set_joint_pos(0, x_curr_9)
        base2.set_joint_pos(1, y_curr_9)
        base2.set_joint_pos(2, z_curr_9)

    # Always set yaw and gripper
    base2.set_joint_pos(3, yaw_curr_9)
    base2.set_joint_pos(5, gripper9)
    base2.set_joint_pos(6, gripper9)

    board9.write_current(0, currentAdjust(latest_error['x'], 0))
    board9.write_current(1, currentAdjust(-latest_error['y'], 1))
    board9.write_current(2, currentAdjust(latest_error['z'], 2))

    time.sleep(0.001)
    
board9.cleanup()