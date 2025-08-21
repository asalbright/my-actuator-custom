import myactuator_rmd_py as rmd

    # Set up driver
driver = rmd.CanDriver("can0")

    # Set up actuator
actuator = rmd.ActuatorInterface(driver, 1)


print(actuator.getControllerGains())

print(actuator.getControlMode())

print(actuator.getMotorStatus2())