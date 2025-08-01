#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList

CLOSE_POSITION = 790
OPEN_POSITION = 1470
VELOCITY = 5

class Test():
    def __init__(self):
        rospy.init_node('non_linearity_bend_test', anonymous=True)

        self.dynamixel_command_service = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.dynamixel_command_service.wait_for_service(timeout=2.0)
        
        self.dynamixel_command_vel(VELOCITY)
        #self.dynamixel_command_pos(CLOSE_POSITION)
        # rospy.sleep(10)
        self.dynamixel_command_pos(CLOSE_POSITION)


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
        # print("command sent to dynamixel, old position: ", self.dynamixel_current_position, "new:", new_position)
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
        controller = Test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass