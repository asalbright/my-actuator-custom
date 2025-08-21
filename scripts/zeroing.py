## this script tests that the zeroing function works correctly
import myactuator_rmd_py as rmd
import time

driver = rmd.CanDriver("can0")

# Create a dictionary of actuators for easy access and iteration
actuators = {i: rmd.ActuatorInterface(driver, i) for i in range(1, 4)}

# Detect actuators
for i, a in actuators.items():
    try:
        a.getMotorStatus2()
        print(f"Actuator {i} detected.")
    except Exception as e:
        print(f"Actuator {i} not detected: {e}")

# # Zero actuators
# for i, a in actuators.items():
#     a.setCurrentPositionAsEncoderZero()
#     print(f"Actuator {i} zeroed.")

time.sleep(2)

# Reset actuators
for i, a in actuators.items():
    a.reset()
    print(f"Actuator {i} reset.")
