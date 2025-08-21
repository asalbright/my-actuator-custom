import matplotlib.pyplot as plt
import math
import time
import csv

import myactuator_rmd_py as rmd

# Set up driver
driver = rmd.CanDriver("can0")

# Set up actuator
actuator = rmd.ActuatorInterface(driver, 1)

# Frequency of the sine wave (in Hz)
frequency = .3  # Adjust this value to increase or decrease the frequency
actuator.releaseBrake()
# Lists to store motor data
currents = []
shaft_speeds = []
shaft_angles = []
target_currents = []  # List to store target current data for plotting
x_time = []



update_interval = 5  # Update the plot every 10 iterations
iteration = 0

try:
    start_time = time.time()
    last_position = 0  # Initialize last_position to a default value (e.g., 0)
    
    # Track the last sent position
    while True:
        # Calculate elapsed time
        elapsed_time = time.time() - start_time

        # Get motor status
        motorInfo = actuator.getMotorStatus2()

        # Append motor data and time in sync
        currents.append(motorInfo.current)
        shaft_speeds.append(motorInfo.shaft_speed)
        shaft_angles.append(actuator.getSingleTurnAngle())

        # Update the position at the specified interval
        iteration += 1
        if iteration % update_interval == 0:
            # Generate step function for target current
            target_current_value = 0.5  # Start at 0.4 and increase by 0.1 every second

            # Send current setpoint
            motorPOS = actuator.sendCurrentSetpoint(target_current_value)
            target_currents.append(target_current_value)  # Append the current value to the list
            last_position = target_current_value  # Update the last sent position
        else:
            # Append the last sent position to keep the lists synchronized
            target_currents.append(last_position)
        
        # Append the time for this iteration (only when data is appended)
        x_time.append(elapsed_time)

        # Small delay to avoid overwhelming the actuator
        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nStopped by User.")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Ensure the motor is shut down and data is saved

    # Save data to a CSV file
    with open("motor_data.csv", "w", newline="") as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write the header
        csvwriter.writerow(["Time (s)", "Current (A)", "Shaft Speed (rpm)", "Shaft Angle (deg)", "Target Current (A)"])
        # Find the minimum length of all lists to avoid IndexError
        min_length = min(len(x_time), len(currents), len(shaft_speeds), len(shaft_angles), len(target_currents))
        # Write the data
        for i in range(min_length):
            csvwriter.writerow([x_time[i], currents[i], shaft_speeds[i], shaft_angles[i], target_currents[i]])
    print("Data saved to motor_data.csv")

    actuator.shutdownMotor()  # Shutdown the motor when interrupted
    print("Motor shutdown successfully.")