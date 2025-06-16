#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList

CLOSE_POSITION = 950
OPEN_POSITION = 1175
VELOCITY = 15

class GloveGripperController:

    def __init__(self):
        rospy.init_node('dynamixel_ui_controller', anonymous=True)
        rospy.Subscriber("/glove_data", Float32, self.glove_sensor_callback)
        rospy.Subscriber("/gripper_data", Float32, self.gripper_sensor_callback)
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.dynamixel_state_callback, queue_size=1)
        
        self.dynamixel_command_service = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.dynamixel_command_vel(VELOCITY)
        self.current_position = None
        rospy.loginfo("Glove Gripper Controller started!")

    def glove_sensor_callback(self, msg):
        #rospy.loginfo(f"Glove data: {msg.data}")
        if msg.data < 0:
            self.dynamixel_command_pos(CLOSE_POSITION)
        else:
            self.dynamixel_command_pos(OPEN_POSITION)

    def gripper_sensor_callback(self, msg):
        #rospy.loginfo(f"Gripper data: {msg.data}")
        ...

    def dynamixel_state_callback(self, msg):
        self.current_position = msg.dynamixel_state[0].present_position

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
        try:
            response = self.dynamixel_command_service('', 1, 'Goal_Position', new_position)
            if response.comm_result:
                #rospy.loginfo("Command position executed successfully.")
                pass
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
