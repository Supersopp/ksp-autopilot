import krpc
import time
import pid_controller
import os
import math

if __name__ == "__main__":
    connection = krpc.connect(name="autopilot")
    vessel     = connection.space_center.active_vessel
    vessel_name = vessel.name
    if vessel_name == "Cricketjumper MKI":
        print("Connected to " + vessel_name)
        running = True
    else:
        running = False

    altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')
    surface_altitude = connection.add_stream(getattr, vessel.flight(), 'surface_altitude')
    ned_rf = vessel.surface_reference_frame
    orbit_rf = vessel.orbit.body.reference_frame
    velocity_surface = connection.add_stream(connection.space_center.transform_velocity,vessel.position(orbit_rf),vessel.velocity(orbit_rf),orbit_rf,ned_rf)
    stage_1_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)
    launcher_liquid_fuel = connection.add_stream(stage_1_resources.amount, 'LiquidFuel')

    altitude_pid       = pid_controller.PID(0.2,0.0,0.0,0.0,0.0)
    vertical_speed_pid = pid_controller.PID(0.2,1.0,0.0,1.0,0.0)
    horizontal_speed_pid = pid_controller.PID(2.0,0.0,0.0,30.0,-30.0)
    altitude_target = 500
    vertical_speed_target = 0

    state = "INIT"
    loop_time = 0.1
    launch_site_altitude_ASL = altitude()
    dt = 0.0

    while running:
        # Find time step from last iteration.
        # Do not calculate if in INIT state.
        time_now = connection.space_center.ut
        if state != "INIT":
            dt = time_now - time_last
        time_last = time_now

        altitude_above_launch_site = altitude() - launch_site_altitude_ASL
        velocity_ned   = (-velocity_surface()[1],-velocity_surface()[2],velocity_surface()[0])
        vertical_speed = -velocity_ned[2]
        speed_north    = velocity_ned[0]
        speed_east     = velocity_ned[1]

        # Flight modes state machine
        if state == "INIT":
            altitude_control = False
            vertical_speed_control = False
            spoiler_active = False
            position_control = False
            horizontal_speed_control = False
            vessel.control.throttle = 0.0
            state = "LANDED"

        elif state == "LANDED":
            vertical_speed_control =  False
            altitude_control = False
            position_control = False
            vessel.control.rcs = False
            vessel.control.throttle = 0.0
            vessel.auto_pilot.disengage()
            if vessel.available_thrust > 0.0:
                state = "PRE_LAUNCH"
                launch_time = time_now + 2.0

        elif state == "PRE_LAUNCH":
            vessel.auto_pilot.target_pitch_and_heading(90,90)
            vessel.auto_pilot.engage()
            vessel.control.rcs = True
            vessel.control.throttle = 0.0
            if time_now > launch_time:
                state = "LAUNCH"

        elif state == "LAUNCH":
            vertical_speed_target = vessel.flight().terminal_velocity
            vertical_speed_control = True
            if altitude_above_launch_site < 2.0:
                vessel.auto_pilot.target_pitch_and_heading(90,90)
            elif altitude_above_launch_site < 10.0:
                vessel.control.gear = False
                vessel.auto_pilot.target_pitch_and_heading(80,90)
            else:
                state = "ACENDING"

        elif state == "ACENDING":
            altitude_target = 500.0
            altitude_control = True
            if altitude() > altitude_target - 5.0:
                hover_time = time_now + 10.0
                state = "HOVER"
        
        elif state == "HOVER":
            altitude_target = 500.0
            altitude_control = True
            position_control = True
            if time_now > hover_time:
                state = "DECENDING"

        elif state == "DECENDING":
            vertical_speed_target = -0.2*(altitude_above_launch_site-5.0) - 1.0
            vertical_speed_control = True
            position_control = True
            spoiler_active = True
            if altitude_above_launch_site < 5.0:
                state = "LANDING"

        elif state == "LANDING":
            vertical_speed_target = -1.0
            vertical_speed_control = True
            horizntal_speed_control = True
            horizontal_speed_target = (0.0,0.0)
            spoiler_active = True
            vessel.control.gear = True
            if vertical_speed > -0.1 and altitude_above_launch_site < 1:
                landing_time = time_now
                state = "POST_LANDING"

        elif state == "POST_LANDING":
            vessel.auto_pilot.target_pitch_and_heading(90,90)
            vessel.auto_pilot.engage()
            vessel.control.throttle = 0.0
            if landing_time + 10.0 < time_now:
                state = "LANDED"
                running = False

        elif state == "ABORT":
            spoiler_active = True
            if vertical_speed < -2.0:
                vessel.control.abort(True)
            if min(surface_altitude() ,altitude()) < 20.0:
                vertical_speed_target = -1.0
                vertical_speed_control = True
            if vertical_speed > -0.1 and min(surface_altitude() ,altitude()) < 10.0:
                landing_time = time_now
                state = "POST_LANDING"


        # PID control loop for altitude control.
        if altitude_control and state != "INIT":
            altitude_error = altitude_target - altitude()
            vertical_speed_target = altitude_pid.calculate_command(altitude_error,dt)
            vertical_speed_control = True
            altitude_control = False
        else:
            altitude_pid.reset()

        # PID control loop for vertical speed control.
        throttle_command = 0.0
        if vertical_speed_control and state != "INIT":
            vertical_speed_error = vertical_speed_target - vertical_speed
            throttle_command     = min(max(vertical_speed_pid.calculate_command(vertical_speed_error,dt),0.0),1.0) 
            vessel.control.throttle = throttle_command
            vertical_speed_control == False
        else:
            vertical_speed_pid.reset()
        
        # Position control.
        if position_control:
            horizontal_speed_target = (0.0, 0.0)
            horizontal_speed_control = True    
            
        # Horizontal speed control.
        if horizontal_speed_control:
            horizontal_speed_north_target = horizontal_speed_target[0]
            horizontal_speed_east_target  = horizontal_speed_target[1] 
            speed_north_error = horizontal_speed_north_target - speed_north
            speed_east_error  = horizontal_speed_east_target - speed_east
            horizontal_speed_error_magnitude = math.sqrt(speed_north_error*speed_north_error+speed_east_error*speed_east_error)
            horizontal_speed_error_direction = 180.0*math.atan2(speed_east_error,speed_north_error)/math.pi
            pitch = 90 -  min(max(horizontal_speed_pid.calculate_command(horizontal_speed_error_magnitude,dt),-30),30)
            heading = horizontal_speed_error_direction
            while heading < 0.0:
                heading += 360.0
            while heading > 360.0:
                heading -= 360.0 
            vessel.auto_pilot.target_pitch_and_heading(pitch,heading)
            vessel.auto_pilot.engage()
        horizontal_speed_control = False


        # Spoiler activation.
        if spoiler_active and vertical_speed < -0.1: 
            vessel.control.toggle_action_group(1)
        else:
            vessel.control.toggle_action_group(2)
        spoiler_activate = False

        # Check for abort condition.
        #if launcher_liquid_fuel() < 40.0:
        #    state = "ABORT"

        # Print status.
        if state != "INIT":
            os.system('clear')
            print("State:                 " + state)
            print("Altitude ASL:          " + str(altitude()))
            print("Altitude SURF:         " + str(surface_altitude()))
            print("Altitude ALS:          " + str(altitude_above_launch_site))
            print("Altitude ASL target:   " + str(altitude_target)) 
            print("Vertical speed:        " + str(vertical_speed))
            print("Vertical speed target: " + str(vertical_speed_target))
            print("Launcher fuel:         " + str(launcher_liquid_fuel()))
            print("Throttle effort:       " + str(throttle_command*100))
            print("Delta time:            " + str(dt))
            print("Velocity:              " + str(velocity_ned))
            if horizontal_speed_control:
                print("Horizontal speed err:  " + str(horizontal_speed_error_magnitude) + "m/s " + str(horizontal_speed_error_direction) + "deg")

        # Sleep for remainder of looptime.
        sleep_time = loop_time - (connection.space_center.ut - time_now)
        if sleep_time > 0.0:
            time.sleep(sleep_time)

