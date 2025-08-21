import time
import math
import csv

from pynput import keyboard
import myactuator_rmd_py as rmd
import myactuator_custom.get_error_codes as ec

driver = rmd.CanDriver("can0")

a2 = rmd.ActuatorInterface(driver, 2)
a3 = rmd.ActuatorInterface(driver, 3)

MIN_a2 = 15
MAX_a2 = 85
MIN_a3 = 30
MAX_a3 = 150

loopTime = 0.05

startTime = time.time()


# Check if actuators are present
missing_motors = []

try:
    angle_a2 = a2.getMultiTurnAngle()  # Get multi-turn angle for actuator a2
    print(f"a2 multi-turn angle: {angle_a2:.2f} degrees")
except Exception:
    missing_motors.append("a2")

try:
    angle_a3 = a3.getMultiTurnAngle()  # Get multi-turn angle for actuator a3
    print(f"a3 multi-turn angle: {angle_a3:.2f} degrees")
except Exception:
    missing_motors.append("a3")

if missing_motors:
    print(f"Not reading motor(s): {', '.join(missing_motors)}")
else:
    print("Robot arm ready")

# Initialize actuator parameters

run_main_loop = True
a1_angle_bounds = True

# Jog velocity (in units per second)
jog_velocity = 20

# Key mappings for each actuator (matching KeyboardJointMovement.py)
key_mappings = {
    2: {"increase": "t", "decrease": "g"},
    3: {"increase": "y", "decrease": "h"},
}

# Track jog state for each actuator
jog_state = {2: 0, 3: 0}

# Store pressed keys to handle key releases
pressed_keys = set()


def on_press(key):
    """Handle key presses to jog velocities."""
    global run_main_loop
    try:
        if key.char == "s":
            print("Key 's' pressed. Stopping main loop and shutting down motors...")
            run_main_loop = False  # Stop the main loop first
            shutdown_all_motors()  # Then shut down the motors
        if key.char == "r":
            print("Key 'r' pressed. Resetting motors...")
            reset_motors()
        if key.char == "x":
            print("Key 'x' pressed. Exiting main loop...")
            run_main_loop = False
    except AttributeError:
        pass


def reset_motors():
    a2.reset()
    a3.reset()
    print("Motors reset successfully.")


def shutdown_all_motors():
    print("Shutting down all motors...")
    actuators = [a2, a3]  # Use the actuators defined in the main script
    motors_stopped = [False] * len(actuators)  # Track which motors have stopped

    while not all(motors_stopped):  # Continue until all motors are stopped
        for i, actuator in enumerate(actuators, start=1):
            if not motors_stopped[i - 1]:  # Only process motors that haven't stopped
                velocity = actuator.getMotorStatus2().shaft_speed
                print(f"Actuator {i} - Current velocity: {velocity}")

                if abs(velocity) > 0.01:  # If velocity is not zero, send setpoint
                    print(f"Actuator {i} - Setting velocity target to 0...")
                    actuator.sendVelocitySetpoint(0)
                else:  # If velocity is zero, mark the motor as stopped
                    print(
                        f"Actuator {i} - Motor velocity is 0. Proceeding to shutdown."
                    )
                    actuator.shutdownMotor()
                    motors_stopped[i - 1] = True

        # Add a small delay to avoid spamming the motor controllers
        time.sleep(0.1)

    print("All motors shut down successfully.")


def jog_on_press(key):
    """Handle key presses for jogging actuators."""
    try:
        if hasattr(key, "char") and key.char:
            pressed_keys.add(key.char)
            for actuator_id, keys in key_mappings.items():
                if key.char == keys["increase"]:
                    jog_state[actuator_id] = jog_velocity
                elif key.char == keys["decrease"]:
                    jog_state[actuator_id] = -jog_velocity
    except AttributeError:
        pass


def jog_on_release(key):
    """Handle key releases to stop jogging actuators."""
    try:
        if hasattr(key, "char") and key.char:
            if key.char in pressed_keys:
                pressed_keys.remove(key.char)
            for actuator_id, keys in key_mappings.items():
                if key.char in (keys["increase"], keys["decrease"]):
                    jog_state[actuator_id] = 0
    except AttributeError:
        pass


def set_velocity(actuator_id, velocity):
    if actuator_id == 2:
        a2.sendVelocitySetpoint(velocity)
        print(f"Actuator {actuator_id} - Velocity set to: {velocity}")
    elif actuator_id == 3:
        a3.sendVelocitySetpoint(velocity)
        print(f"Actuator {actuator_id} - Velocity set to: {velocity}")


# Start the main keyboard listener (for s/r/x)
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Start jog listener (non-blocking, events handled in main loop)
jog_listener = keyboard.Listener(on_press=jog_on_press, on_release=jog_on_release)
jog_listener.start()

# Prompt user to begin program
input("Press Enter to begin the main loop...")

# Start loop
try:
    while run_main_loop:

        currentTime = time.time()

        ec.get_error_codes(a2)
        ec.get_error_codes(a3)

        angle_a2 = a2.getMultiTurnAngle()
        angle_a3 = a3.getMultiTurnAngle()

        a2PositiveBoundReached = angle_a2 >= MAX_a2
        a2NegativeBoundReached = angle_a2 <= MIN_a2
        a3PositiveBoundReached = angle_a3 >= MAX_a3
        a3NegativeBoundReached = angle_a3 <= MIN_a3

        for actuator_id, velocity in jog_state.items():
            if (velocity > 0 and actuator_id == 2 and a2PositiveBoundReached) or (
                velocity < 0 and actuator_id == 2 and a2NegativeBoundReached
            ):
                set_velocity(actuator_id, 0)
            elif (velocity > 0 and actuator_id == 3 and a3PositiveBoundReached) or (
                velocity < 0 and actuator_id == 3 and a3NegativeBoundReached
            ):
                set_velocity(actuator_id, 0)
            else:
                set_velocity(actuator_id, velocity)

        # print(
        #     f"Temperature (C): {a1.getMotorStatus2().temperature}, {a2.getMotorStatus2().temperature}"
        # )
        # print(
        #     f"Current (A): {a1.getMotorStatus2().current}, {a2.getMotorStatus2().current}"
        # )
        # print(
        #     f"Total Current (A): {a1.getMotorStatus2().current + a2.getMotorStatus2().current}"
        # )
        print(
            f"a2 Angle: {a2.getMultiTurnAngle():.2f}, a3 Angle: {a3.getMultiTurnAngle():.2f}"
        )

        while time.time() - currentTime < loopTime:
            time.sleep(0.001)

except KeyboardInterrupt:
    pass

listener.stop()
jog_listener.stop()
