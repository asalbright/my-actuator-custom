import numpy as np
import time
import csv

from pynput import keyboard
import myactuator_rmd_py as rmd
from myactuator_custom.manual_can import ManualCAN


driver = rmd.CanDriver("can0")

a2 = rmd.ActuatorInterface(driver, 2)
a3 = rmd.ActuatorInterface(driver, 3)


def on_press(key):
    try:
        if key.char == "s":
            print("Key 's' pressed. Stopping main loop and shutting down motors...")
            shutdown_all_motors()
    except AttributeError:
        pass


listener = keyboard.Listener(on_press=on_press)
listener.start()

MAX_SAMPLES = 100_000  # Adjust as needed

# Pre-allocate arrays
currents_a2 = np.zeros(MAX_SAMPLES)
shaft_speeds_a2 = np.zeros(MAX_SAMPLES)
shaft_angles_a2 = np.zeros(MAX_SAMPLES)
currents_a3 = np.zeros(MAX_SAMPLES)
shaft_speeds_a3 = np.zeros(MAX_SAMPLES)
shaft_angles_a3 = np.zeros(MAX_SAMPLES)
x_time = np.zeros(MAX_SAMPLES)
target_positions = np.zeros(MAX_SAMPLES)  # To store target positions

sample_idx = 0  # Track how many samples have been filled


def Main():
    try:
        with ManualCAN() as manual_can:
            change_gains(manual_can)
            # step_function()
            # a3.sendVelocitySetpoint(-2)
            # track_motor_performance()
    except KeyboardInterrupt:
        pass
    finally:
        # Save only the filled part of the arrays
        with open("robot_data.csv", "w", newline="") as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(
                [
                    "Time (s)",
                    "A2 Current (A)",
                    "A2 Shaft Speed (rpm)",
                    "A2 Shaft Angle (deg)",
                    "A3 Current (A)",
                    "A3 Shaft Speed (rpm)",
                    "A3 Shaft Angle (deg)",
                    "Target Position (deg)",
                ]
            )
            for i in range(sample_idx):
                csvwriter.writerow(
                    [
                        x_time[i],
                        currents_a2[i],
                        shaft_speeds_a2[i],
                        shaft_angles_a2[i],
                        currents_a3[i],
                        shaft_speeds_a3[i],
                        shaft_angles_a3[i],
                        target_positions[i],
                    ]
                )
        print("Data saved to robot_data.csv")
        listener.stop()


# def step_function():
#     step_size = 5  # degrees per step (change as needed)
#     interval = 0.5  # seconds between steps (change as needed)
#     num_steps = 3  # total number of steps (change as needed)
#     speed = 10  # speed for setpoint (change as needed)
#     start_angle = 0  # starting angle (degrees)

#     current_angle = start_angle
#     for i in range(num_steps):
#         print(f"Step {i+1}: Moving to {current_angle} degrees at speed {speed}")
#         a3.sendPositionAbsoluteSetpoint(current_angle, speed)
#         current_target_position = current_angle  # Update the global target
#         time.sleep(interval)
#         current_angle += step_size


def track_motor_performance():
    global sample_idx
    step_size = 5
    interval = 0.5
    num_steps = 3
    speed = 10
    start_angle = 0

    current_angle = start_angle
    steps_sent = 0
    next_step_time = time.time()

    start_time = time.time()
    while sample_idx < MAX_SAMPLES and steps_sent < num_steps:
        elapsed_time = time.time() - start_time

        # Send step if it's time
        if time.time() >= next_step_time and steps_sent < num_steps:
            print(
                f"Step {steps_sent+1}: Moving to {current_angle} degrees at speed {speed}"
            )
            a3.sendPositionAbsoluteSetpoint(current_angle, speed)
            current_target_position = current_angle
            current_angle += step_size
            steps_sent += 1
            next_step_time += interval

        try:
            motorInfo_a2 = a2.getMotorStatus2()
            motorInfo_a3 = a3.getMotorStatus2()
        except rmd.ProtocolException as e:
            print(f"ProtocolException at sample {sample_idx}: {e}")
            time.sleep(0.01)
            continue

        # Store data in arrays
        currents_a2[sample_idx] = motorInfo_a2.current
        shaft_speeds_a2[sample_idx] = motorInfo_a2.shaft_speed
        shaft_angles_a2[sample_idx] = a2.getMultiTurnAngle()
        currents_a3[sample_idx] = motorInfo_a3.current
        shaft_speeds_a3[sample_idx] = motorInfo_a3.shaft_speed
        shaft_angles_a3[sample_idx] = a3.getMultiTurnAngle()
        x_time[sample_idx] = elapsed_time
        target_positions[sample_idx] = current_target_position

        if motorInfo_a2.current > 20:
            shutdown_all_motors()
            print("High current detected on A2. Shutting down motors for safety.")
        if motorInfo_a3.current > 10:
            shutdown_all_motors()
            print("High current detected on A3. Shutting down motors for safety.")

        print(
            f"Sample {sample_idx}: Time={elapsed_time:.2f}s | "
            f"A2 Current={motorInfo_a2.current:.2f}A, | "
            f"A3 Current={motorInfo_a3.current:.2f}A | "
            f"Target={current_target_position}"
        )

        sample_idx += 1
        time.sleep(0.01)


def change_gains(manual_can):
    message_id = 0x142
    gain_values = manual_can.get_gain(message_id)
    print(gain_values)
    # time.sleep(1)
    # # manual_can.set_gains(message_id, gain_type='current', kp=0.1, ki=0.2)
    manual_can.set_gains(message_id, gain_type="position", kp=0, ki=0, kd=0)
    # manual_can.set_gains(message_id, gain_type="speed", kp=0.02, ki=0.0005)
    time.sleep(1)
    gain_values = manual_can.get_gain(message_id)
    print(gain_values)


def shutdown_all_motors():
    print("Shutting down all motors...")
    actuators = [a2, a3]  # Use the actuators defined in the main script
    motors_stopped = [False] * len(actuators)  # Track which motors have stopped

    while not all(motors_stopped):  # Continue until all motors are stopped
        for i, actuator in enumerate(actuators, start=1):
            if not motors_stopped[i - 1]:  # Only process motors that haven't stopped
                velocity = actuator.getMotorStatus2().shaft_speed
                # print(f"Actuator {i} - Current velocity: {velocity}")

                if abs(velocity) > 0.01:  # If velocity is not zero, send setpoint
                    print(f"Actuator {i} - Setting velocity target to 0...")
                    actuator.sendVelocitySetpoint(0)
                else:  # If velocity is zero, mark the motor as stopped
                    print(
                        f"Actuator {i} - Motor velocity is 0. Proceeding to shutdown."
                    )
                    time.sleep(0.01)
                    actuator.sendVelocitySetpoint(0)
                    actuator.shutdownMotor()
                    motors_stopped[i - 1] = True

        # Add a small delay to avoid spamming the motor controllers
        time.sleep(0.1)

    print("All motors shut down successfully.")


if __name__ == "__main__":
    Main()
