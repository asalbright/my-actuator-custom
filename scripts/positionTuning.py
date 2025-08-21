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
frequency = 0.475  # Adjust this value to increase or decrease the frequency

# Lists to store motor data
currents = []
shaft_speeds = []
shaft_angles = []
positions = []  # List to store position data for plotting
x_time = []

update_interval = 5  # Update the plot every 10 iterations
iteration = 0

try:
    if iteration == 0:
        start_position = actuator.getMultiTurnAngle()  # Get the starting position
        print(f"Starting position: {start_position}")

    start_time = time.time()
    last_position = (
        start_position  # Initialize the last position with the starting position
    )
    while True:
        # Calculate elapsed time
        elapsed_time = time.time() - start_time

        # Get motor status
        motorInfo = actuator.getMotorStatus2()

        # Append motor data and time in sync
        currents.append(motorInfo.current)
        shaft_speeds.append(motorInfo.shaft_speed)
        shaft_angles.append(actuator.getMultiTurnAngle())

        # Update the position at the specified interval
        iteration += 1
        if iteration % update_interval == 0:
            # Generate step position starting from the initial position
            position = start_position + 70 * math.sin(
                2 * math.pi * frequency * elapsed_time
            )

            # Send position setpoint
            motorPOS = actuator.sendPositionAbsoluteSetpoint(position, 1000)
            positions.append(position)
            last_position = position  # Update the last sent position
        else:
            # Append the last sent position to keep the lists synchronized
            positions.append(last_position)

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
        csvwriter.writerow(
            [
                "Time (s)",
                "Current (A)",
                "Shaft Speed (rpm)",
                "Shaft Angle (deg)",
                "Position (deg)",
            ]
        )
        # Find the minimum length of all lists to avoid IndexError
        min_length = min(
            len(x_time),
            len(currents),
            len(shaft_speeds),
            len(shaft_angles),
            len(positions),
        )
        # Write the data
        for i in range(min_length):
            csvwriter.writerow(
                [x_time[i], currents[i], shaft_speeds[i], shaft_angles[i], positions[i]]
            )
    print("Data saved to motor_data.csv")

    actuator.shutdownMotor()  # Shutdown the motor when interrupted
    print("Motor shutdown successfully.")
