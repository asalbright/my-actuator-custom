# !/usr/bin/env python3

import time

import myactuator_rmd_py as rmd


def main():
    # Set up driver
    driver = rmd.CanDriver("can0")

    # Set up actuator
    a1 = rmd.ActuatorInterface(driver, 1)

    # a1.releaseBrake()
    # time.sleep(1)
    a1.sendVelocitySetpoint(2000)
    time.sleep(3)
    # a1.lockBrake()
    # a1.sendPositionAbsoluteSetpoint(100000,10000)
    a1.shutdownMotor()

    try:
        while True:
            read_value = a1.getMotorStatus1().error_code
            if (
                read_value != rmd.actuator_state.ErrorCode.NO_ERROR
            ):  # Compare to NO_ERROR
                print(f"Error code: {read_value}")
            # time.sleep()  # Sleep to avoid flooding the output
    except KeyboardInterrupt:
        print("\nInterrupted! Checking motor status...")
        # Read and print the final motor status
        final_status = a1.getMotorStatus1()
        print(
            f"Final motor status: Error code = {final_status.error_code}, Temperature = {final_status.temperature}, Voltage = {final_status.voltage}, Brake released = {final_status.is_brake_released}"
        )
        print("Exiting program.")


if __name__ == "__main__":
    main()
