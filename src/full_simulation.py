#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Float32
# from std_msgs.msg import Header
# from glove_gripper_control.msg import StampedFloat32
# from dynamixel_workbench_msgs.srv import DynamixelCommand
# from dynamixel_workbench_msgs.msg import DynamixelStateList
import numpy as np
import time
import matplotlib.pyplot as plt

KP = 10
KI = 0.2
KD = 0.1

DT = 0.01

CLOSE_POSITION = 790
OPEN_POSITION = 1470
GOAL_TOLERANCE = 2 # gripper goal position tolerance
VELOCITY = 50
DEADBAND = 1

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, error, dt):
        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        # PID output
        pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return pid_output

class Chromosome():

    def __init__(self, kp, ki, kd, vel):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.base_velocity = vel
        self.pid = PID(KP, KI, KD)
        self.glove_sensor_data_generator = self.simulate_glove_time_series()
        self.dynamixel_current_position = 830+(1470-790)/2
        self.last_update_time = 0.0
        self.fitness = 0
        self.mse = 0 # mean squared error

        self.gripper_time_data = []
        self.gripper_values_data = []

        self.glove_time_data = []
        self.glove_values_data = []
    
    def __str__(self):
        return f"Chromosome(kp={self.kp}, ki={self.ki}, kd={self.kd}, base_velocity={self.base_velocity})"

    # def simulate_glove_time_series(self):
        # duration = 3.0  # seconds
        # dt = 0.1       # sampling interval (10 ms)
        # t = np.arange(0, duration, dt)  # time vector

        # # Sensor value range
        # min_val = 830
        # max_val = 870
        # amplitude = (max_val - min_val) / 2
        # offset = (max_val + min_val) / 2

        # # Frequency: 1 full wave over 3 seconds
        # frequency = 1 / duration  # ≈ 0.333 Hz

        # # Sinusoidal signal
        # sensor_values = offset + amplitude * np.sin(2 * np.pi * frequency * t)

        # # Plot it
        # # plt.plot(t, sensor_values)
        # # plt.title("Simulated Flex Sensor Sinusoidal Data")
        # # plt.xlabel("Time (s)")
        # # plt.ylabel("Sensor Reading")
        # # plt.grid(True)
        # # plt.show()

        # for i in range(len(t)):
        #     yield t[i], sensor_values[i]

    def simulate_glove_time_series(self):

        dt = 0.01  # time step in seconds

        # Phase durations
        t1 = 1.0  # hold at 850
        t2 = 2.0  # rise to 860
        t3 = 1.0  # hold at 860
        t4 = 1.0  # fall to 835 (parabolically)
        t5 = 1.0  # hold at 835

        # Phase 1: Hold at 850
        t_phase1 = np.arange(0, t1, dt)
        v_phase1 = np.full_like(t_phase1, 850.0)

        # Phase 2: Linear rise to 860
        t_phase2 = np.arange(t1, t1 + t2, dt)
        v_phase2 = np.linspace(850, 860, len(t_phase2))

        # Phase 3: Hold at 860
        t_phase3 = np.arange(t1 + t2, t1 + t2 + t3, dt)
        v_phase3 = np.full_like(t_phase3, 860.0)

        # Phase 4: Parabolic fall to 835
        t_phase4 = np.arange(t1 + t2 + t3, t1 + t2 + t3 + t4, dt)
        # Use a downward parabola from 860 to 835
        normalized_time = np.linspace(0, 1, len(t_phase4))
        v_phase4 = 860 - 25 * (normalized_time ** 2)  # 860 → 835

        # Phase 5: Hold at 835
        t_phase5 = np.arange(t1 + t2 + t3 + t4, t1 + t2 + t3 + t4 + t5, dt)
        v_phase5 = np.full_like(t_phase5, 835.0)

        # Combine all phases
        t_all = np.concatenate([t_phase1, t_phase2, t_phase3, t_phase4, t_phase5])
        v_all = np.concatenate([v_phase1, v_phase2, v_phase3, v_phase4, v_phase5])

        # Yield time and value as a generator
        for ti, vi in zip(t_all, v_all):
            yield ti, vi

    def calculate_fitness(self):
        print("Calculating fitness for:", self)
        while True:
            try:
                t, current_glove_sensor_value = next(self.glove_sensor_data_generator)

                flex_per_one_step = 40/680
                current_gripper_sensor_value = 830+((self.dynamixel_current_position-790)*flex_per_one_step)
                current_gripper_sensor_value = round(current_gripper_sensor_value, 2)

                # spremiti podatke u neki time series, da kasnije mogu graf nacrtati
                self.gripper_time_data.append(t)
                self.gripper_values_data.append(current_gripper_sensor_value)
                self.glove_time_data.append(t)
                self.glove_values_data.append(current_glove_sensor_value)

                # preracunati iz trenutne glove sensor u target dynamixel position value:
                steps_per_flex = 680/40
                target_motor_position = 790+(current_glove_sensor_value-830)*steps_per_flex

                # IZRACUN ERRORA:
                dt = t - self.last_update_time
                self.last_update_time = t
                sensor_error = current_glove_sensor_value - current_gripper_sensor_value
                position_error = target_motor_position - self.dynamixel_current_position
                pid_output = self.pid.compute(position_error, dt)

                new_position_command = pid_output + self.dynamixel_current_position
                new_speed_command = VELOCITY + (abs(sensor_error) * VELOCITY * 0.1)

                self.mse = self.mse + (current_glove_sensor_value - current_gripper_sensor_value)**2
                position_error = target_motor_position - self.dynamixel_current_position

                rps = (new_speed_command*0.229)/60
                deg_per_sec = rps*360
                degrees_passed = dt*deg_per_sec
                ticks_passed = (degrees_passed/360)*4096
                direction = 0
                direction = 1 if new_position_command > self.dynamixel_current_position else -1
                self.dynamixel_current_position = self.dynamixel_current_position + (direction*ticks_passed)

            except StopIteration:
                break

        self.fitness = (1/(self.mse + 1e-6)) * 100000 # 100000 is here for scaling.
        return self.fitness

    def plot_result(self):
        plt.figure(figsize=(10, 5))
        plt.plot(self.glove_time_data, self.glove_values_data, 'o', label="Glove Sensor", color='red')
        plt.plot(self.gripper_time_data, self.gripper_values_data, 'x', label="Gripper Sensor", color='blue')
        plt.xlabel("Time (s)")
        plt.ylabel("Sensor Reading")
        plt.title(f"{self}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    try:
        # kromosom je jedan par cetvorke kp, ki, kd, vel

        t1 = time.time()
        c = Chromosome(KP, KI, KD, VELOCITY)
        c.calculate_fitness()
        print("MSE:", c.mse)
        print("FITNESS:", c.fitness)
        print("Test duration:", time.time()-t1, "s")
        c.plot_result()

    except Exception as e:
        print(e)