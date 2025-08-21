from pynput import keyboard
import time

from myactuator_custom.shutdown_motors import actuators

# Jog velocity (in units per second)
jog_velocity = 10

# Key mappings for each actuator
key_mappings = {
    1: {"increase": "r", "decrease": "f"},
    2: {"increase": "t", "decrease": "g"},
    3: {"increase": "y", "decrease": "h"},
    4: {"increase": "u", "decrease": "j"},
    5: {"increase": "i", "decrease": "k"},
    6: {"increase": "o", "decrease": "l"},
}

# Control mode: "jog"
control_mode = "jog"  # Default to jog mode


def toggle_mode():
    """Toggle between position and jog modes."""
    global control_mode
    control_mode = "jog" if control_mode == "position" else "position"
    print(f"Control mode switched to: {control_mode}")


def shutdown_all_motors():
    print("Shutting down all motors...")
    motors_stopped = [False] * len(actuators)  # Track which motors have stopped

    while not all(motors_stopped):  # Continue until all motors are stopped
        for i, actuator in enumerate(actuators, start=1):
            if not motors_stopped[i - 1]:  # Only process motors that h
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


def set_velocity(actuator_id, velocity):
    """Set the velocity of the specified actuator."""
    if actuator_id > len(actuators):
        print(f"No actuator detected for ID {actuator_id}.")
        return

    actuators[actuator_id - 1].sendVelocitySetpoint(velocity)
    print(f"Actuator {actuator_id} - Velocity set to: {velocity}")


def on_press(key):
    """Handle key presses to jog velocities."""
    try:
        # Check which key was pressed and act based on jog mode
        for actuator_id, keys in key_mappings.items():
            if key.char == keys["increase"]:
                set_velocity(actuator_id, jog_velocity)  # Use jog_velocity variable
                return
            elif key.char == keys["decrease"]:
                set_velocity(actuator_id, -jog_velocity)  # Use jog_velocity variable
                return

        # Handle the 's' key to shut down all motors
        if key.char == "s":
            print("Key 's' pressed. Shutting down motors...")
            shutdown_all_motors()
            return False  # Stop the listener after shutting down motors
    except AttributeError:
        pass


def on_release(key):
    """Handle key releases to stop jog mode velocities."""
    try:
        # Check which key was released and stop the corresponding actuator
        for actuator_id, keys in key_mappings.items():
            if key.char in (keys["increase"], keys["decrease"]):
                set_velocity(actuator_id, 0)  # Stop the actuator
                return
    except AttributeError:
        pass


print("Press the following keys to jog actuators:")
for actuator_id, keys in key_mappings.items():
    print(
        f"  Actuator {actuator_id}: Increase ('{keys['increase']}'), Decrease ('{keys['decrease']}')"
    )
print("Press 's' to shut down all motors.")

# Start listening for key presses and releases
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
