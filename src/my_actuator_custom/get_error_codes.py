import myactuator_rmd_py as rmd


def get_error_codes(a):
    read_value = a.getMotorStatus1().error_code
    if read_value != rmd.actuator_state.ErrorCode.NO_ERROR:  # Compare to NO_ERROR
        print(f"Error code: {read_value}")


if __name__ == "__main__":
    get_error_codes()
