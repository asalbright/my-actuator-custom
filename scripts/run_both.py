import time

import myactuator_rmd_py as rmd

# Set up driver
driver = rmd.CanDriver("can0")

    # Set up actuator
a1 = rmd.ActuatorInterface(driver, 1)
a2 = rmd.ActuatorInterface(driver, 2)

a1.sendVelocitySetpoint(100)
a2.sendVelocitySetpoint(500)

# time.sleep(10)

# a1.sendVelocitySetpoint(0)
# a2.sendVelocitySetpoint(0)

# time.sleep(5)

# a1.shutdownMotor()
# a2.shutdownMotor()
