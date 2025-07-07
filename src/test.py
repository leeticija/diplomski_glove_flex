#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList

# sensor: gripper open - 855, closed - 829
# glove open (up) - 855, close (down) - 829
CLOSE_POSITION = 790
OPEN_POSITION = 1226
GOAL_TOLERANCE = 2 # gripper given goal position tolerance
VELOCITY = 15
DEADBAND = 1

class GloveGripperController:

    def __init__(self):
        rospy.init_node('dynamixel_ui_controller', anonymous=True)
        rospy.Subscriber("/glove_data_ome", Float32, self.glove_sensor_callback, queue_size=1)
        rospy.Subscriber("/gripper_data_ome", Float32, self.gripper_sensor_callback, queue_size=1)
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.dynamixel_state_callback, queue_size=1)

        self.dynamixel_command_service = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.dynamixel_command_service.wait_for_service(timeout=2.0)
        self.dynamixel_command_vel(VELOCITY)
        self.dynamixel_current_position = None

        print("potrebno pomaknuti motor za: ", (1226-950)/(855-829), "za promjenu od 1 stupnja savijenosti.") # tolko se mora pomaknuti za promjenu od 1 stupnja savijanja.
        self.moving_constant = (855-829)/(1226-950) # za koliko se senzor promijeni ako motor pomaknem za 1.
        self.goal_position = None
        self.glove_reading = None
        self.gripper_reading = None

        timer = rospy.Timer(rospy.Duration(0.1), self.refresh_gripper_state)
    
    def refresh_gripper_state(self, event):
        # print("GlOVE: ", self.glove_reading)
        # print("GRIPPER: ", self.gripper_reading)
        diff = abs(self.glove_reading - self.gripper_reading)
        print("diff: ", diff)
        print("current gripper position: ", self.dynamixel_current_position)

        if (self.glove_reading < self.gripper_reading) and self.dynamixel_current_position - 0.5*(diff/self.moving_constant) > CLOSE_POSITION:
            print("closing! velocitiy sped up for ", 10*diff, "%")
            self.dynamixel_command_vel(int(VELOCITY*0.1*diff))
            self.dynamixel_command_pos(int(self.dynamixel_current_position - 0.5*(diff/self.moving_constant)))
        elif self.glove_reading < self.gripper_reading:
            print("cannot!, wanted: ", int(self.dynamixel_current_position - (diff/self.moving_constant)), "max closed: ", CLOSE_POSITION)
        if (self.glove_reading > self.gripper_reading) and self.dynamixel_current_position + 0.5*(diff/self.moving_constant) < OPEN_POSITION:
            print("opening! velocitiy sped up for ", 10*diff, "%")
            self.dynamixel_command_vel(int(VELOCITY*0.1*diff))
            self.dynamixel_command_pos(int(self.dynamixel_current_position + 0.5*(diff/self.moving_constant)))
        elif self.glove_reading > self.gripper_reading:
            print("cannot!, wanted: ", int(self.dynamixel_current_position + (diff/self.moving_constant)), "max opened: ", OPEN_POSITION)

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
        controller = GloveGripperController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
