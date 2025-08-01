#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from glove_gripper_control.msg import StampedFloat32
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList
import numpy as np
import matplotlib.pyplot as plt

# PID constants
KP = 0.3
KI = 0.5
KD = 0.2
DT = 0.01

CLOSE_POSITION = 790
OPEN_POSITION = 1470
GOAL_TOLERANCE = 2 # gripper goal position tolerance
VELOCITY = 10
DEADBAND = 1

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 0
        self.last_time = rospy.get_time()
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, error):
        current_time = rospy.get_time()
        self.dt = current_time - self.last_time
        self.last_time = current_time
        # PID terms
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        self.prev_error = error
        # PID output
        pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return pid_output


class SimulatedCascadedControllerNode():

    def __init__(self):
        rospy.init_node('cascaded_pid_controller')
        #rospy.Subscriber("/sim_motor/state", Float32, self.dynamixel_state_callback, queue_size=1)
        self.simulated_glove_pub = rospy.Publisher('/simulated_glove_flex', StampedFloat32, queue_size=10)
        self.simulated_gripper_flex_pub = rospy.Publisher('/simulated_gripper_flex', StampedFloat32, queue_size=10)
        self.position_publisher = rospy.Publisher('/sim_motor/set_position', StampedFloat32, queue_size=1)
        self.velocity_publisher = rospy.Publisher('/sim_motor/set_velocity', StampedFloat32, queue_size=1)
        rospy.Subscriber('/sim_motor/state', Float32, self.dynamixel_state_callback)
        self.pid = PID(KP, KI, KD)
        self.dynamixel_current_position = None
        self.dt = 0.1

        self.glove_sensor_data_generator = self.simulate_glove_time_series()
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)

    def control_loop(self, event):
        print("Testing Control Loop.")
        if not self.dynamixel_current_position:
            print("dynamixel did not set position.")
            rospy.logwarn_throttle(5, "Waiting for position data...")
            return
        try:
            # PUBLISH SIMULATED GLOVE FLEX VALUES:
            t, current_glove_sensor_value = next(self.glove_sensor_data_generator)
            msg = StampedFloat32()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.data = float(current_glove_sensor_value)
            self.simulated_glove_pub.publish(msg)
            print(f"{t:.2f}s -> {current_glove_sensor_value:.1f}")

            # znaci: glove je u rasponu [830, 870], a gripper isto.
            # gripperov motor je u rasponu [790, 1470]
            # buduci da je linearan odziv, pridruzimo svakoj vrijednosti senzora neku vrijednost motora:
            # 1470-790=680 pomaka motora
            # 870-830=40 flex pomaka
            
            flex_per_one_step = 40/680
            print("Motor current position: ", self.dynamixel_current_position)
            current_gripper_sensor_value = 830+((self.dynamixel_current_position-790)*flex_per_one_step)
            current_gripper_sensor_value = round(current_gripper_sensor_value, 2)
            gripper_sensor_msg = StampedFloat32()
            gripper_sensor_msg.header = Header()
            gripper_sensor_msg.header.stamp = rospy.Time.now()
            gripper_sensor_msg.data = float(current_gripper_sensor_value)
            self.simulated_gripper_flex_pub.publish(gripper_sensor_msg)
            print("Gripper sensor:", current_gripper_sensor_value)

            # preracunati iz trenutne glove sensor u target dynamixel position value:
            steps_per_flex = 680/40
            target_motor_position = 790+(current_glove_sensor_value-830)*steps_per_flex
            print("Target motor position:", target_motor_position)
            
            # IZRACUN ERRORA:
            sensor_error = current_glove_sensor_value - current_gripper_sensor_value
            position_error = target_motor_position - self.dynamixel_current_position
            pid_output = self.pid.compute(position_error)
            
            # PUBLISH NEW POSITION COMMAND:
            new_command = pid_output + self.dynamixel_current_position
            motor_position_command_msg = StampedFloat32()
            motor_position_command_msg.header = Header()
            motor_position_command_msg.header.stamp = rospy.Time.now()
            motor_position_command_msg.data = new_command
            print("Publishing new position command:", new_command)
            self.position_publisher.publish(motor_position_command_msg)
            
            # PUBLISH NEW VELOCITY COMMAND:
            new_speed = VELOCITY + (abs(sensor_error) * VELOCITY * 0.1)
            velocity_command_msg = StampedFloat32()
            velocity_command_msg.header = Header()
            velocity_command_msg.header.stamp = rospy.Time().now()
            velocity_command_msg.data = new_speed
            self.velocity_publisher.publish(velocity_command_msg)

            # fitness error cu racunati s trenutnim razlikama u senzorima:
            fitness_error = current_glove_sensor_value - current_gripper_sensor_value
            position_error = target_motor_position - self.dynamixel_current_position
            #output = self.pid.compute(position_error)
            # error directly affects speed:
            #new_speed = error * VELOCITY
        except StopIteration:
            print("Yielded all sensor values.")

        #position_error = self.target_position - self.dynamixel_current_position
        #output = self.pid.compute(position_error)
        # error directly affects speed:
        #new_speed = error * VELOCITY

    def simulate_glove_time_series(self):
        duration = 3.0  # seconds
        dt = 0.01       # sampling interval (10 ms)
        t = np.arange(0, duration, dt)  # time vector

        # Sensor value range
        min_val = 830
        max_val = 870
        amplitude = (max_val - min_val) / 2
        offset = (max_val + min_val) / 2

        # Frequency: 1 full wave over 3 seconds
        frequency = 1 / duration  # ≈ 0.333 Hz

        # Sinusoidal signal
        sensor_values = offset + amplitude * np.sin(2 * np.pi * frequency * t)

        # Plot it
        # plt.plot(t, sensor_values)
        # plt.title("Simulated Flex Sensor Sinusoidal Data")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Sensor Reading")
        # plt.grid(True)
        # plt.show()

        for i in range(len(t)):
            yield t[i], sensor_values[i]

    def error_sum_fitness(self, glove_data_series):
        ...

    def glove_sensor_callback(self, msg):
        self.glove_reading = msg.data

    def gripper_sensor_callback(self, msg):
        self.gripper_reading = msg.data

    def dynamixel_state_callback(self, msg):
        self.dynamixel_current_position = msg.data


if __name__ == "__main__":
    try:
        controller = SimulatedCascadedControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass