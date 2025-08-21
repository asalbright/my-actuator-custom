import time

import myactuator_rmd_py as rmd

# Set up driver
driver = rmd.CanDriver("can0")

# Set up actuators
actuators = [
    rmd.ActuatorInterface(driver, 1),
    rmd.ActuatorInterface(driver, 2),
    rmd.ActuatorInterface(driver, 3),
    rmd.ActuatorInterface(driver, 4),
    rmd.ActuatorInterface(driver, 5),
    rmd.ActuatorInterface(driver, 6),
    # Add more actuators here as needed, e.g., rmd.ActuatorInterface(driver, 3)
]


# Function to ensure motor velocity is zero before shutdown
def ensure_motor_stopped(actuator, actuator_id):
    # Get the current velocity
    velocity = actuator.getMotorStatus2().shaft_speed
    print(f"Actuator {actuator_id} - Current velocity: {velocity}")

    # If velocity is not zero, set velocity target to 0
    if abs(velocity) > 0.01:  # Allow a small tolerance
        print(f"Actuator {actuator_id} - Setting velocity target to 0...")
        actuator.sendVelocitySetpoint(0)

        # Wait until the velocity is approximately 0
        while abs(actuator.getMotorStatus2().shaft_speed) > 0.01:
            velocity = actuator.getMotorStatus2().shaft_speed
            print(
                f"Actuator {actuator_id} - Waiting for motor to stop. Current velocity: {velocity}"
            )
            time.sleep(0.1)

    print(f"Actuator {actuator_id} - Motor velocity is 0. Proceeding to shutdown.")


# Ensure all motors are stopped and then shut them down
if __name__ == "__main__":
    for i, actuator in enumerate(actuators, start=1):
        ensure_motor_stopped(actuator, i)
        actuator.shutdownMotor()
        print(f"Actuator {i} shut down successfully.")
