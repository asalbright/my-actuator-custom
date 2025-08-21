import time

import myactuator_rmd_py as rmd

# Set up driver
driver = rmd.CanDriver("can0")

# Set up actuator
a1 = rmd.ActuatorInterface(driver, 6)

# Convert the returned values to strings for printing
print("Initial Position A1: " + str(a1.getSingleTurnAngle()))

a1.sendPositionAbsoluteSetpoint(0, 100)

# Wait for the motor position to read 0
while abs(a1.getSingleTurnAngle()) > 0.01:
    time.sleep(0.1)
    print("Current Position A1: " + str(a1.getSingleTurnAngle()))

if abs(a1.getSingleTurnAngle()) <= 0.01:
    print("Motor is at position 0")
    a1.shutdownMotor()
