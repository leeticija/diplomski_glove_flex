#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from glove_gripper_control.msg import StampedFloat32
import time

class SimulatedMotor:
    def __init__(self):
        rospy.init_node("simulated_motor_node")
        self.position_pub = rospy.Publisher("/sim_motor/state", Float32, queue_size=10)
        rospy.Subscriber("/sim_motor/set_position", StampedFloat32, self.set_position_callback)
        rospy.Subscriber("/sim_motor/set_velocity", StampedFloat32, self.set_velocity_callback)

        # Motor state
        self.current_position = 850.0  # starting position
        # self.target_position = 850.0
        self.velocity = 0.0
        self.last_update_time = rospy.get_time()

        # Delay handling
        self.command_delay = 0.003  # 3 ms delay
        self.position_cmd = None
        self.velocity_cmd = None
        #self.command_queue_time = None

        rospy.Timer(rospy.Duration(0.05), self.update) # sluzi za publishanje motor state-a. Mora biti brzi nego sto zahtjevi dolaze (svakih 0.01 s).

    # za requestanje promjene pozicije motora!
    def set_position_callback(self, msg):
        self.position_cmd = msg.data
        print("Position command: ",  msg.data, str(msg.header.stamp))
        #self.command_queue_time = rospy.get_time()
    
    # za requestanje promjene brzine motora!
    def set_velocity_callback(self, msg):
        self.velocity_cmd = msg.data
        print("Velocity command: ", self.velocity_cmd)
        #self.command_queue_time = rospy.get_time()

    def update(self, event):

        self.position_pub.publish(Float32(self.current_position))

        print("Updating motor position!")

        if not (self.position_cmd and self.velocity_cmd):
            #print("Position and velocity not commanded!")
            rospy.logwarn_throttle(1, "Waiting for position and velocity commands...")
            return

        now = rospy.get_time() # zamijeniti sa simuliranim vremenom (slati vrijeme kao varijablu!)
        dt = now - self.last_update_time
        self.last_update_time = now

        # Each unit of Profile_Velocity = 0.229 RPM
        # potrebno izracunati gdje je motor sad:
        rps = (self.velocity_cmd*0.229)/60
        deg_per_sec = rps*360
        degrees_passed = dt*deg_per_sec
        # convert to ticks:
        ticks_passed = (degrees_passed/360)*4096
        direction = 0
        direction = 1 if self.position_cmd > self.current_position else -1
        self.current_position = self.current_position + (direction*ticks_passed)

        self.position_pub.publish(Float32(self.current_position))


if __name__ == "__main__":
    try:
        motor = SimulatedMotor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass