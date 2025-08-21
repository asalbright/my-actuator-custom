import time
import math
import csv
import numpy as np
import os

from pynput import keyboard
import myactuator_rmd_py as rmd
import myactuator_custom.get_error_codes as ec

from myactuator_custom.Receiver_code import CommandReceiver


driver = rmd.CanDriver("can0")

a2 = rmd.ActuatorInterface(driver, 2)
a3 = rmd.ActuatorInterface(driver, 3)


currents_a2_events = []
shaft_speeds_a2_events = []
shaft_angles_a2_events = []
currents_a3_events = []
shaft_speeds_a3_events = []
shaft_angles_a3_events = []
tip_x_target_events = []
a2_target_angles = []
a3_target_angles = []
connor_targets = []
x_target_differents = []
tip_x_position_events = []
position_error_events = []

MIN_a2 = 20  ## can be 15
MAX_a2 = 80  ## can be 85
MIN_a3 = -154  ## can be -150
MAX_a3 = -40  ## can be -30

control_mode = "tip"

a3polarity = -1  # Set to -1 if a3 needs to be inverted

loopTime = 0.05

startTime = None

link_length = 0.4318  # Check if actuators are present

missing_motors = []


try:
    StartingAngle_a2 = a2.getMultiTurnAngle()  # Get multi-turn angle for actuator a2
    print(f"a2 multi-turn angle: {StartingAngle_a2:.2f} degrees")
except Exception:
    missing_motors.append("a2")

try:
    StartingAngle_a3 = (
        a3.getMultiTurnAngle() * a3polarity
    )  # Get multi-turn angle for actuator a3
    print(f"a3 multi-turn angle: {StartingAngle_a3:.2f} degrees")
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
jog_velocity = 10  # This sets the magnitude for both x and y

tip_jog_velocity = 0.1

# Tip velocity command (updated by arrow keys)
tip_velocity_command = {"x": 0.0, "y": 0.0}


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
    global run_main_loop, control_mode, first_loop, startTime
    try:
        if key.char == "s":
            print("Key 's' pressed. Stopping main loop and shutting down motors...")
            run_main_loop = False
        if key.char == "r":
            print("Key 'r' pressed. Resetting motors...")
            reset_motors()
        if key.char == "x":
            print("Key 'x' pressed. Exiting main loop...")
            run_main_loop = False
        if key.char == "m":
            # Cycle through the three control modes
            modes = [
                "tip",
                "tip position",
                "Tip Position without control loop",
                "joint",
            ]
            current_index = modes.index(control_mode)
            control_mode = modes[(current_index + 1) % len(modes)]
            print(f"Control mode switched to: {control_mode}")
            startTime = time.time()
            first_loop = True
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
        if velocity != 0:
            print(f"Actuator {actuator_id} - Velocity set to: {velocity}")
    elif actuator_id == 3:
        a3.sendVelocitySetpoint(velocity)
        if velocity != 0:
            print(f"Actuator {actuator_id} - Velocity set to: {velocity}")


def tip_jog_on_press(key):
    """Handle arrow key presses for tip jogging."""
    try:
        if key == keyboard.Key.right:
            tip_velocity_command["x"] = -tip_jog_velocity
        elif key == keyboard.Key.left:
            tip_velocity_command["x"] = tip_jog_velocity
        elif key == keyboard.Key.up:
            tip_velocity_command["y"] = tip_jog_velocity
        elif key == keyboard.Key.down:
            tip_velocity_command["y"] = -tip_jog_velocity
    except AttributeError:
        pass


def tip_jog_on_release(key):
    """Handle arrow key releases for tip jogging."""
    try:
        if key == keyboard.Key.right or key == keyboard.Key.left:
            tip_velocity_command["x"] = 0.0
        elif key == keyboard.Key.up or key == keyboard.Key.down:
            tip_velocity_command["y"] = 0.0
    except AttributeError:
        pass


def resolved_rate_control(g2, g3, vx, vy):

    # Use the current tip velocity command
    commandedvelocities = InJacobian(vx, vy, g2, g3)

    a2commandedvelocity = commandedvelocities[0]
    a3commandedvelocity = commandedvelocities[1]

    # Jacobian = link_length * np.array(
    #     [
    #         [-(math.sin(g2 + g3) + math.sin(g2)), -(math.sin(g2 + g3))],
    #         [math.cos(g2 + g3) + math.cos(g2), math.cos(g2 + g3)],
    #     ]
    # )
    # vHat = Jacobian @ commandedvelocities
    # print(f"Too check vHat: [{vHat}] should equal [{vx}, {vy}]")

    return a2commandedvelocity, a3commandedvelocity


def InJacobian(vx, vy, g2, g3):
    inJ = np.array(
        [
            [math.cos(g2 + g3), math.sin(g2 + g3)],
            [-(math.cos(g2) + math.cos(g2 + g3)), -(math.sin(g2) + math.sin(g2 + g3))],
        ]
    )
    v = np.array([vx, vy])
    det = 1 / (link_length * math.sin(g3))
    commandedvelocities = (inJ @ v) * det
    return commandedvelocities


def rate_limits(a2AVel_c, a3AVel_c):
    aVMax = math.radians(180)
    scale = 1
    scale2 = 1
    scale3 = 1
    if abs(a2AVel_c) > aVMax:
        scale2 = aVMax / abs(a2AVel_c)
    if abs(a3AVel_c) > aVMax:
        scale3 = aVMax / abs(a3AVel_c)
    scale = min(scale2, scale3)
    a2AVel_c = a2AVel_c * scale
    a3AVel_c = a3AVel_c * scale
    return a2AVel_c, a3AVel_c


def joint_limits(angle_a2_deg, angle_a3_deg, a2_vel, a3_vel):
    """Clamp velocities to zero if joint limits are reached."""
    if (a2_vel > 0 and angle_a2_deg >= MAX_a2) or (
        a2_vel < 0 and angle_a2_deg <= MIN_a2
    ):
        a3_vel = 0
        a2_vel = 0
    if (a3_vel > 0 and angle_a3_deg >= MAX_a3) or (
        a3_vel < 0 and angle_a3_deg <= MIN_a3
    ):
        a2_vel = 0
        a3_vel = 0

    # print(angle_a2_deg, MIN_a2, MAX_a2)
    # print(angle_a3_deg, MIN_a3, MAX_a3)
    return a2_vel, a3_vel


def main():
    global run_main_loop, startTime, first_loop

    # Start the main keyboard listener (for s/r/x)
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    receiver = CommandReceiver(listen_port=12345)
    receiver.start()

    # Start jog listener (non-blocking, events handled in main loop)
    jog_listener = keyboard.Listener(on_press=jog_on_press, on_release=jog_on_release)
    jog_listener.start()

    # Start tip jog listener for arrow keys
    tip_jog_listener = keyboard.Listener(
        on_press=tip_jog_on_press, on_release=tip_jog_on_release
    )
    tip_jog_listener.start()

    # Prompt user to begin program
    input("Press Enter to begin the main loop...")

    startTime = time.time()
    startedMoving = False

    # Start loop
    try:
        while run_main_loop:
            currentTime = time.time()
            Pref_sin_frequency = 0.15  # Hz

            ec.get_error_codes(a2)
            ec.get_error_codes(a3)

            angle_a2_deg = a2.getMultiTurnAngle()
            angle_a3_deg = a3.getMultiTurnAngle() * a3polarity

            motorInfo_a2 = a2.getMotorStatus2()
            motorInfo_a3 = a3.getMotorStatus2()
            plot_time = time.time() - startTime
            currents_a2_events.append((plot_time, motorInfo_a2.current))
            shaft_speeds_a2_events.append((plot_time, motorInfo_a2.shaft_speed))
            shaft_angles_a2_events.append((plot_time, angle_a2_deg))
            currents_a3_events.append((plot_time, motorInfo_a3.current))
            shaft_speeds_a3_events.append((plot_time, motorInfo_a3.shaft_speed))
            shaft_angles_a3_events.append((plot_time, angle_a3_deg * a3polarity))

            if motorInfo_a2.current > 20:
                shutdown_all_motors()
                time.sleep(0.02)
                shutdown_all_motors()
                print("High current detected on A2. Shutting down motors for safety.")
                run_main_loop = False  # Exit main loop after shutdown
                break
            if motorInfo_a3.current > 10:
                shutdown_all_motors()
                time.sleep(0.02)
                shutdown_all_motors()
                print("High current detected on A3. Shutting down motors for safety.")
                run_main_loop = False  # Exit main loop after shutdown
                break

            angle_a2_s = math.radians(angle_a2_deg)
            angle_a3_s = math.radians(angle_a3_deg)

            a2AVel_s_deg = a2.getMotorStatus2().shaft_speed  # i think this is rad/s
            a3AVel_s_deg = a3.getMotorStatus2().shaft_speed * a3polarity

            a2AVel_s = math.radians(a2AVel_s_deg)
            a3AVel_s = math.radians(a3AVel_s_deg)

            a2PositiveBoundReached = angle_a2_deg >= MAX_a2
            a2NegativeBoundReached = angle_a2_deg <= MIN_a2
            a3PositiveBoundReached = angle_a3_deg >= MAX_a3
            a3NegativeBoundReached = angle_a3_deg <= MIN_a3

            if control_mode == "joint":
                # Joint jog (old style, t/g/y/h keys)
                for actuator_id, velocity in jog_state.items():
                    if actuator_id == 2:
                        vel = velocity
                        vel, _ = joint_limits(angle_a2_deg, angle_a3_deg, vel, 0)
                        set_velocity(actuator_id, vel)
                    elif actuator_id == 3:
                        vel = velocity
                        _, vel = joint_limits(angle_a2_deg, angle_a3_deg, 0, vel)
                        set_velocity(actuator_id, vel)

            elif control_mode == "tip":
                # Tip jog (arrow keys, resolved rate control)
                if tip_velocity_command["x"] != 0.0 or tip_velocity_command["y"] != 0.0:

                    a2AVel_c, a3AVel_c = resolved_rate_control(
                        angle_a2_s,
                        angle_a3_s,
                        tip_velocity_command["x"],
                        tip_velocity_command["y"],
                    )
                    a2AVel_c, a3AVel_c = rate_limits(a2AVel_c, a3AVel_c)

                    # print(math.degrees(a3AVel_c))
                    # print(math.degrees(a2AVel_c))
                    a2AVel_c, a3AVel_c = joint_limits(
                        angle_a2_deg, angle_a3_deg, a2AVel_c, a3AVel_c
                    )

                    a2.sendVelocitySetpoint(math.degrees(a2AVel_c))
                    a3.sendVelocitySetpoint(math.degrees(a3AVel_c * a3polarity))

                    P_s = find_P_s(angle_a2_s, angle_a3_s)
                    print(f"Tip position: {P_s}")

                    Pdx_s = link_length * (
                        -((a2AVel_s + a3AVel_s) * math.sin(angle_a2_s + angle_a3_s))
                        - (a2AVel_s * math.sin(angle_a2_s))
                    )
                    Pdy_s = link_length * (
                        ((a2AVel_s + a3AVel_s) * math.cos(angle_a2_s + angle_a3_s))
                        + (a2AVel_s * math.cos(angle_a2_s))
                    )

                else:
                    # If no tip jog, you may want to stop the actuators (optional)
                    a2.sendVelocitySetpoint(0)
                    a3.sendVelocitySetpoint(0)

            #####################################################################################################
            #################---------------------------------------------------------_##########################
            #####################################################################################################

            elif control_mode == "tip position":
                if first_loop:
                    first_loop = False
                    StartingAngle_a2 = angle_a2_s
                    StartingAngle_a3 = angle_a3_s
                    P_start = find_P_s(StartingAngle_a2, StartingAngle_a3)
                    Px_offset = P_start[0, 0] - receiver.get_latest_command()

                P_s = find_P_s(angle_a2_s, angle_a3_s)
                tip_x_position_events.append((time.time() - startTime, P_s[0, 0]))
                camera_cg_c = receiver.get_latest_command()
                connor_targets.append(
                    (time.time() - startTime, camera_cg_c + Px_offset)
                )
                P_ref = generated__sin_Pref(Pref_sin_frequency, P_start, currentTime)
                # P_ref = generated_noisy_sin_Pref(
                #     Pref_sin_frequency, P_start, currentTime
                # )
                tip_x_target_events.append((time.time() - startTime, P_ref[0, 0]))
                x_diff = P_ref[0, 0] - camera_cg_c
                x_target_differents.append((time.time() - startTime, x_diff))
                P_ref[0, 0] = camera_cg_c + Px_offset

                e = P_ref - P_s
                x_error = e[0, 0]
                # if abs(x_error) > 0.005 and not startedMoving:
                #     time.sleep(0.01)
                #     print(x_error)
                #     continue
                # startedMoving = True
                position_error_events.append((time.time() - startTime, x_error))
                Kpx = 7  # 11
                Kpy = 0.7
                dP_c = e * np.array([[Kpx], [Kpy]])
                a2AVel_c, a3AVel_c = resolved_rate_control(
                    angle_a2_s,
                    angle_a3_s,
                    dP_c[0, 0],
                    dP_c[1, 0],
                )

                a2AVel_c, a3AVel_c = rate_limits(a2AVel_c, a3AVel_c)

                a2AVel_c, a3AVel_c = joint_limits(
                    angle_a2_deg, angle_a3_deg, a2AVel_c, a3AVel_c
                )
                print(math.degrees(a3AVel_c))
                print(math.degrees(a2AVel_c))

                a2.sendVelocitySetpoint(math.degrees(a2AVel_c))
                a3.sendVelocitySetpoint(math.degrees(a3AVel_c) * a3polarity)

            elif control_mode == "Tip Position without control loop":
                if first_loop:
                    first_loop = False
                    StartingAngle_a2 = angle_a2_s
                    StartingAngle_a3 = angle_a3_s
                    P_start = find_P_s(StartingAngle_a2, StartingAngle_a3)

                # y = 0
                # x, _, _ = receiver.get_latest_command()
                # tip_x_target_events.append((time.time() - startTime, x))
                # Px_ref = x
                # Py_ref = y

                P_ref = generated__sin_Pref(Pref_sin_frequency, P_start, currentTime)
                tip_x_target_events.append((time.time() - startTime, P_ref[0, 0]))
                Px_ref = P_ref[0, 0]
                Py_ref = P_ref[1, 0]

                c_2 = (Px_ref**2 + Py_ref**2 - 2 * (link_length**2)) / (
                    2 * (link_length**2)
                )
                s_2 = -math.sqrt(max(0, 1 - c_2**2))
                angle_a3_c = math.atan2(s_2, c_2)
                angle_a2_c = math.atan2(Py_ref, Px_ref) - math.atan2(
                    link_length * s_2, link_length * (1 + c_2)
                )
                angle_a2_c_deg = math.degrees(angle_a2_c)
                angle_a3_c_deg = math.degrees(angle_a3_c)

                a2_target_angles.append((time.time() - startTime, angle_a2_c_deg))
                a3_target_angles.append(
                    (time.time() - startTime, angle_a3_c_deg * a3polarity)
                )

                error = angle_a3_c_deg - angle_a3_deg
                if abs(error) > 2 and not startedMoving:
                    time.sleep(0.01)
                    # print(abs(error))
                    continue
                startedMoving = True

                print(f"angle_a2_c_deg: {angle_a2_c_deg}")
                print(f"angle_a3_c_deg: {angle_a3_c_deg}")
                print(f"Target position: {Px_ref}")

                a2.sendPositionAbsoluteSetpoint(angle_a2_c_deg, 100)
                a3.sendPositionAbsoluteSetpoint(angle_a3_c_deg * a3polarity, 100)

            # print(time.time() - currentTime)

            if time.time() - currentTime > loopTime:
                print("Loop time exceeded!")
            while time.time() - currentTime < loopTime:
                time.sleep(0.005)

        print("sending shutdown command")
        shutdown_all_motors()
        time.sleep(1)
        shutdown_all_motors()
        return

    except KeyboardInterrupt:
        pass

    finally:
        receiver.stop()
        listener.stop()
        jog_listener.stop()
        tip_jog_listener.stop()
        with open(
            os.path.join(data_dir, "current_a2_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Current (A)"])
            for t, val in currents_a2_events:
                writer.writerow([t, val])
        print("Data saved to current_a2_events.csv")
        with open(
            os.path.join(data_dir, "current_a3_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Current (A)"])
            for t, val in currents_a3_events:
                writer.writerow([t, val])
        print("Data saved to current_a3_events.csv")
        with open(
            os.path.join(data_dir, "shaft_angles_a2_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Shaft Angle (degrees)"])
            for t, val in shaft_angles_a2_events:
                writer.writerow([t, val])
        print("Data saved to shaft_angles_a2_events.csv")
        with open(
            os.path.join(data_dir, "shaft_angles_a3_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Shaft Angle (degrees)"])
            for t, val in shaft_angles_a3_events:
                writer.writerow([t, val])
        print("Data saved to shaft_angles_a3_events.csv")
        with open(
            os.path.join(data_dir, "shaft_speeds_a2_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Shaft Speed (degrees/s)"])
            for t, val in shaft_speeds_a2_events:
                writer.writerow([t, val])
        print("Data saved to shaft_speeds_a2_events.csv")
        with open(
            os.path.join(data_dir, "shaft_speeds_a3_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Shaft Speed (degrees/s)"])
            for t, val in shaft_speeds_a3_events:
                writer.writerow([t, val])
        print("Data saved to shaft_speeds_a3_events.csv")
        with open(
            os.path.join(data_dir, "tip_x_target_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Tip X Target (m)"])
            for t, val in tip_x_target_events:
                writer.writerow([t, val])
        print("Data saved to tip_x_target_events.csv")
        with open(os.path.join(data_dir, "a2_target_angles.csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "A2 Target Angle (degrees)"])
            for t, val in a2_target_angles:
                writer.writerow([t, val])
        print("Data saved to a2_target_angles.csv")
        with open(os.path.join(data_dir, "a3_target_angles.csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "A3 Target Angle (degrees)"])
            for t, val in a3_target_angles:
                writer.writerow([t, val])
        print("Data saved to a3_target_angles.csv")
        with open(os.path.join(data_dir, "connor_targets.csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Connor Target (m)"])
            for t, val in connor_targets:
                writer.writerow([t, val])
        print("Data saved to connor_targets.csv")
        with open(
            os.path.join(data_dir, "x_target_differents.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "X Target Difference"])
            for t, val in x_target_differents:
                writer.writerow([t, val])
        print("Data saved to x_target_differents.csv")
        with open(
            os.path.join(data_dir, "tip_x_position_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Tip X Position (m)"])
            for t, val in tip_x_position_events:
                writer.writerow([t, val])
        print("Data saved to tip_x_position_events.csv")
        with open(
            os.path.join(data_dir, "position_error_events.csv"), "w", newline=""
        ) as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Position Error (m)"])
            for t, val in position_error_events:
                writer.writerow([t, val])
        print("Data saved to position_error_events.csv")
        with open(os.path.join(data_dir, "raw_cg.csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s) ", "raw_cg (m)"])
            for t, val in receiver.raw_cg_data():
                writer.writerow([t - startTime, val])
        print("Data saved to raw_cg.csv")


def generated__sin_Pref(Pref_sin_frequency, P_Start, currentTime):
    elapsed_time = time.time() - startTime
    Px_start = P_Start[0, 0]
    Py_start = P_Start[1, 0]
    Py_ref = Py_start + 0 * (
        1
        + math.sin(
            (2 * math.pi * Pref_sin_frequency * elapsed_time) + ((3 * math.pi) / 2)
        )
    )
    Px_ref = Px_start + 0.25 * (
        1
        + math.sin(
            (2 * math.pi * Pref_sin_frequency * elapsed_time) + ((3 * math.pi) / 2)
        )
    )
    P_ref = np.array([[Px_ref], [Py_ref]])
    # print(f"Generated P_ref: {P_ref}")
    # tip_x_target_events.append((time.time() - startTime, Px_start))
    return P_ref


def generated_noisy_sin_Pref(
    Pref_sin_frequency, P_Start, currentTime, spike_prob=0.15, spike_magnitude=0.01
):
    elapsed_time = time.time() - startTime
    Px_start = P_Start[0, 0]
    Py_start = P_Start[1, 0]
    sine_val = math.sin(
        (2 * math.pi * Pref_sin_frequency * elapsed_time) + ((3 * math.pi) / 2)
    )

    # Random spike logic
    spike = 0.0
    if np.random.rand() < spike_prob:
        spike = np.random.uniform(-spike_magnitude, spike_magnitude)

    Px_ref = Px_start + 0.25 * (1 + sine_val) + spike
    Py_ref = Py_start  # No y movement

    P_ref = np.array([[Px_ref], [Py_ref]])
    return P_ref


def find_P_s(angle_a2_s, angle_a3_s):
    Px_s = link_length * (math.cos(angle_a2_s + angle_a3_s) + math.cos(angle_a2_s))
    Py_s = link_length * (math.sin(angle_a2_s + angle_a3_s) + math.sin(angle_a2_s))
    P_s = np.array([[Px_s], [Py_s]])
    return P_s


if __name__ == "__main__":
    # Create data directory if it doesn't exist
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    os.makedirs(data_dir, exist_ok=True)

    main()

#     # --- Test resolved_rate_control function ---
#     print("Testing resolved_rate_control with sample values...")

#     # Example test cases: (g2_deg, g3_deg, vx, vy)
#     test_cases = [
#         (30, -60, 100, 0),  # Move tip in +x direction
#         # (45, 45, 0, 100),    # Move tip in +y direction
#         # (60, 30, -100, 0),   # Move tip in -x direction
#         # (90, 45, 0, -100),   # Move tip in -y direction
#         # (45, 90, 50, 50),    # Move tip diagonally
#     ]

#     for g2_deg, g3_deg, vx, vy in test_cases:
#         tip_velocity_command["x"] = vx
#         tip_velocity_command["y"] = vy
#         print(f"\nInput: g2={g2_deg}°, g3={g3_deg}°, vx={vx}, vy={vy}")
#         resolved_rate_control(g2_deg, g3_deg)
