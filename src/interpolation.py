#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

#sensor_values = [825, 850, 870, 875, 890]  # From sensor
#real_degrees  = [0,   15,   45,   75,   90]   # Measured manually
sensor_values = [825, 844, 864, 894, 903]  # From sensor
real_degrees  = [0, 22.5, 45, 90, 112.5]   # Measured manually

# 🔹 2. Create linear interpolation function
interp_func = interp1d(sensor_values, real_degrees, kind='cubic')

# 🔹 3. Interpolate at arbitrary sensor values
sensor_range = np.linspace(min(sensor_values), max(sensor_values), 500)
interpolated_angles = interp_func(sensor_range)

# 🔹 4. Plot everything
plt.figure(figsize=(8, 5))
plt.plot(sensor_range, interpolated_angles, label='Interpolirano (kubno)', color='blue')
plt.plot(sensor_values, real_degrees, 'o', label='Tocke', color='red')

plt.title('Sensor Calibration: Linear Interpolation')
plt.xlabel('Sensor value')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()