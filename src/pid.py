#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList

# PID constants
KP = 0.3
KI = 0.05
KD = 0.2

# max velocity scaling factor
VEL_KP = 0.1

CLOSE_POSITION = 790
OPEN_POSITION = 1470
GOAL_TOLERANCE = 2 # gripper goal position tolerance
VELOCITY = 20
DEADBAND = 1

class Regulator():
    def __init__(self):
        rospy.init_node('dynamixel_ui_controller', anonymous=True)
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.get_time()

        rospy.Subscriber("/glove_data_ome", Float32, self.glove_sensor_callback, queue_size=1)
        rospy.Subscriber("/gripper_data_ome", Float32, self.gripper_sensor_callback, queue_size=1)
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.dynamixel_state_callback, queue_size=1)

        self.dynamixel_command_service = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.dynamixel_command_service.wait_for_service(timeout=2.0)
        self.dynamixel_command_vel(VELOCITY)
        
        self.dynamixel_current_position = None
        self.goal_position = None
        self.glove_reading = None
        self.gripper_reading = None
        # self.moving_constant = (855-829)/(1330-790)
        self.moving_constant = (870-830)/(OPEN_POSITION-CLOSE_POSITION)

        timer = rospy.Timer(rospy.Duration(0.1), self.refresh_gripper_state)


    def refresh_gripper_state(self, event):
        if self.glove_reading is None or self.gripper_reading is None or self.dynamixel_current_position is None:
            return

        error = self.gripper_reading-self.glove_reading
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        # PID output
        pid_output = KP * error + KI * self.integral + KD * derivative

        # convert sensor error to motor position correction
        pos_correction = pid_output / self.moving_constant

        new_position = self.dynamixel_current_position + pos_correction
        print("New position is greater or smaller that boundaries!", new_position)
        new_position = max(min(new_position, OPEN_POSITION), CLOSE_POSITION)
        # velocity scaled to error
        velocity = int(min(max(VELOCITY * abs(error) * VEL_KP, 5), 100))

        #print(f"PID pos correction: {pos_correction:.2f}, vel: {velocity}")

        self.dynamixel_command_vel(velocity)
        self.dynamixel_command_pos(int(new_position))

        #print("NEW velocity: ", velocity)
        #print("NEW position: ", int(new_position))

    def glove_sensor_callback(self, msg):

        self.glove_reading = msg.data

    def gripper_sensor_callback(self, msg):
        self.gripper_reading = msg.data

    def dynamixel_state_callback(self, msg):
        self.dynamixel_current_position = msg.dynamixel_state[0].present_position

    def dynamixel_command_vel(self, new_speed):
        try:
            response = self.dynamixel_command_service('', 1, 'Profile_Velocity', new_speed)
            if response.comm_result:
                #rospy.loginfo("Command velocity executed successfully.")
                pass
            else:
                rospy.logwarn("Failed to set velocity.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def dynamixel_command_pos(self, new_position):
        print("command sent to dynamixel, old position: ", self.dynamixel_current_position, "new:", new_position)
        try:
            response = self.dynamixel_command_service('', 1, 'Goal_Position', new_position)
            if response.comm_result:
                ...
            else:
                rospy.logwarn("Failed to set position.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        controller = Regulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass