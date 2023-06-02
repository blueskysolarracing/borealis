"""
Basic simulation of PI cruise control algorithm to find a starting point for gains k_p and k_i.

We assume that the car can maintain a certain max acceleration (positve and negative) and scale the 
current acceleration linearly with the output of the PI control loop (relative to maxima of -255 and 255).
At each timestep, the PI control loop is ran and the car's speed is recomputed until it reaches the target.

"""

from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import time

#--- CONSTANTS ---#
k_p = 70
k_i = 0.1
k_d = 0
initial_velocity_m_per_sec = 15
target_velocity_m_per_sec = 20
timestep_ms = 20
#To avoid cycling between regen and accel frequently when the car is a near target, we
#define a deadband where the PI control loop is paused and reset when target_speed - 2 < current_speed < target_speed + 2
velocity_tolerance_hysteresis = 0
integratorMin = -200
integratorMax = 200
derivativeMin = -200
derivativeMax = 200
outputMin = -255
outputMax = 255
noise_std_dev = 0.05 #Add noise to the speed measurement

#--- CAR PROPERTIES ---#
coefficient_rolling_resistance = 0.0025
car_mass_kg = 280
torque_slope_Nm_per_velocity = 6.12 #Torque increases by 6.12 Nm per m/s
air_density_kg_per_m_squared = 1.2
CdA = 0.067

#--- RUNTIME VARIABLES ---#
integrator = 0
prevError = 0
prevSpeed = 0
output = 0
current_velocity_m_per_sec = initial_velocity_m_per_sec
current_velocity_avg = initial_velocity_m_per_sec

velocity_average_queue_length = 25
velocity_queue = deque()
velocity_sum = 0.0

time_list = [0]
velocity_list = [current_velocity_m_per_sec]
integrator_list = [integrator]
output_list = [0]
times_between_motor_mode_change = [] #List containing the times between a change from regen to accel or vice-versa
time_last_motor_mode_change = time.time()

def velocity_moving_average(current_velocity):
    global velocity_sum
    if len(velocity_queue) < velocity_average_queue_length:
        velocity_queue.append(current_velocity)
        velocity_sum += current_velocity
    else:
        velocity_sum -= velocity_queue.popleft()
        velocity_queue.append(current_velocity)
        velocity_sum += current_velocity
    return velocity_sum / len(velocity_queue)

def compute_max_accel(current_velocity):
    return (torque_slope_Nm_per_velocity*current_velocity - 0.5*air_density_kg_per_m_squared*CdA*current_velocity**2 - 2744*coefficient_rolling_resistance) / car_mass_kg

def compute_max_decel(current_velocity):
    #Assuming torque provided by regen is half of that provided by accel
    return (-0.5*torque_slope_Nm_per_velocity*current_velocity - 0.5*air_density_kg_per_m_squared*CdA*current_velocity**2 - 2744*coefficient_rolling_resistance) / car_mass_kg

def plot_max_accel():
    velocity_list = [i/3.6 for i in range(100)]
    accel_list = []
    for velocity in velocity_list:
        accel_list.append(compute_max_accel(velocity))
    
    plt.plot(velocity_list, accel_list)
    plt.title("Max acceleration vs speed")
    plt.xlabel("Car velocity (m/s)")
    plt.ylabel("Max acceleration (m/s^2)")
    plt.show()

def plot_max_decel():
    velocity_list = [i/3.6 for i in range(100)]
    accel_list = []
    for velocity in velocity_list:
        accel_list.append(compute_max_decel(velocity))
    
    plt.plot(velocity_list, accel_list)
    plt.title("Max acceleration vs speed")
    plt.xlabel("Car velocity (m/s)")
    plt.ylabel("Max deceleration (m/s^2)")
    plt.show()

def compute_new_speed(current_velocity, PI_control):
    new_velocity = current_velocity
    if PI_control < 0: #Regen
        current_max_decel = compute_max_decel(current_velocity)
        new_velocity = current_velocity + PI_control/outputMin * current_max_decel * timestep_ms / 1000
    elif PI_control > 0: #Accel
        current_max_accel = compute_max_accel(current_velocity)
        new_velocity = current_velocity + PI_control/outputMax * current_max_accel * timestep_ms / 1000
        
    return new_velocity

for i in range(10000):
    #PI CONTROL LOOP
    previous_pi_output = output

    error = target_velocity_m_per_sec - current_velocity_avg
    proportionalError = k_p * error
    integrator += k_i * timestep_ms * error
    derivative = k_d * (error - prevError) / timestep_ms

    #Bound integrator
    if integrator > integratorMax:
        integrator = integratorMax
    elif integrator < integratorMin:
        integrator = integratorMin

    #Bound derivative
    if derivative > derivativeMax:
        derivative = derivativeMax
    elif derivative < derivativeMin:
        derivative = derivativeMin

    output = proportionalError + integrator + derivative
    #Bound output
    if output > outputMax:
        output = outputMax
    elif output < outputMin:
        output = outputMin
    output_list.append(output)

    prevError = error
    prevSpeed = current_velocity_avg
    integrator_list.append(integrator)

    #Motor mode change
    if ((previous_pi_output < 0) and (output > 0)) or ((previous_pi_output > 0) and (output < 0)):
        times_between_motor_mode_change.append(20*i - time_last_motor_mode_change)
        time_last_motor_mode_change = 20*i

    #Hysteresis
    if abs(current_velocity_m_per_sec - target_velocity_m_per_sec) < velocity_tolerance_hysteresis:
        output = previous_pi_output

    #CAR KINEMATICS
    current_velocity_m_per_sec = compute_new_speed(current_velocity_m_per_sec, PI_control=output) + np.random.normal(0, noise_std_dev, size=None)
    current_velocity_avg = velocity_moving_average(current_velocity_m_per_sec)
    velocity_list.append(current_velocity_avg)

    time_list.append( time_list[-1] + timestep_ms / 1000 )

#Plots and prints
ax1 = plt.subplot()
plt.plot(time_list, velocity_list)
plt.axhline(y=target_velocity_m_per_sec, color='green', linestyle='--')
ax2 = ax1.twinx()
ax2.plot(time_list, output_list, 'r')

print(f"Times between motor changes (ms): {times_between_motor_mode_change[1:-1]}")

plt.show()