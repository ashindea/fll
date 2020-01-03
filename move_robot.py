#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.parameters import Port

import common_methods

DEFAULT_SPEED=170
DEFAULT_COLOR_FIND_SPEED=100
DEFAULT_LINEFOLLOW_SPEED=100
DEFAULT_ANGULAR_SPEED=45
TANK_CHASSIS_LEN_MM=200

def turn_to_direction(robot, gyro, target_angle, speed_mm_s = DEFAULT_SPEED):
    angle_change = target_angle - gyro.angle()

    robot.drive_time(0, 0.9 * angle_change, 1000)
    robot.stop(stop_type=Stop.BRAKE)

    max_attempts=10 # limit oscialltions to 10, not forever
    while ( abs(target_angle - gyro.angle()) > 3 and max_attempts >0):
        error=target_angle - gyro.angle()
        adj_angular_speed = error * 1.5
        robot.drive(0, adj_angular_speed)
        wait(100)
        max_attempts -= 1

    robot.stop(stop_type=Stop.BRAKE)

    adjusted_angle = gyro.angle()
    common_methods.log_string('turn_to_direction -- Adjusted target: ' + str(target_angle) + ' now: ' + str(adjusted_angle))




def turn(robot, angle, speed_mm_s = DEFAULT_SPEED):

    if angle > 0:    # right turns are a bit under-done
        angle = int(1.1 * angle)
    else:
        angle = int(angle / 1)

    robot.drive_time(0, angle, 1000)
    robot.stop(stop_type=Stop.BRAKE)

def move_reverse(robot,
    max_distance, 
    speed_mm_s = DEFAULT_SPEED):
    move_straight(robot, -1 * max_distance, speed_mm_s)

def move_straight(robot,
    max_distance, 
    speed_mm_s = DEFAULT_SPEED,
    stop_on_color = None, 
    stop_on_obstacle_at = -1):

    print('Move stratight at speed '+ str(speed_mm_s) + ' dist ' + str(max_distance))
    if (max_distance < 0 ):
        # moving in reverse
        speed_mm_s = -1 * speed_mm_s

    duration = abs(int(1000 * max_distance / speed_mm_s))

    # Drive forward
    brick.display.text('Driving for left and right' + str(200))
    print('Driving for left and right' + str(200))
 
    #robot.drive(speed = 200, steering = 0)
    robot.drive_time(speed_mm_s, 0, duration)
    robot.stop(stop_type=Stop.BRAKE)
 

def turn_to_color(robot,
    color_sensor,
    stop_on_color,
    rotate_dir = 1,
    angular_speed_deg_s = DEFAULT_ANGULAR_SPEED):
 
    robot.drive(0, rotate_dir * angular_speed_deg_s)
    # Check if color reached.
    common_methods.log_string('preturncolor: ' + str(color_sensor.color()) + ' intens: ' + str(color_sensor.reflection()))
    while color_sensor.color() != stop_on_color:
        common_methods.log_string('turncolor: ' + str(color_sensor.color()) + ' intens: ' + str(color_sensor.reflection()))
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)

def turn_to_color_right(robot,
    color_sensor,
    stop_on_color,
    angular_speed_deg_s = DEFAULT_ANGULAR_SPEED):
 
    turn_to_color(robot, color_sensor, stop_on_color, 1, angular_speed_deg_s)

def turn_to_color_left(robot,
    color_sensor,
    stop_on_color,
    angular_speed_deg_s = DEFAULT_ANGULAR_SPEED):
 
    turn_to_color(robot, color_sensor, stop_on_color, -1, angular_speed_deg_s )


def move_to_color(robot,
    color_sensor,
    stop_on_color,
    speed_mm_s = DEFAULT_COLOR_FIND_SPEED):
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        common_methods.log_string('color: ' + str(color_sensor.color()) + ' intens: ' + str(color_sensor.reflection()))
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)



def move_to_color_reverse(robot,
    color_sensor,
    stop_on_color,
    speed_mm_s = DEFAULT_COLOR_FIND_SPEED):
    move_to_color(robot,
        color_sensor,
        stop_on_color,
        speed_mm_s = -1 * speed_mm_s)

def move_to_obstacle(robot,
    obstacle_sensor,
    stop_on_obstacle_at,
    speed_mm_s = DEFAULT_SPEED):

    brick.display.text('Driving to obstacle' + str(stop_on_obstacle_at))
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while obstacle_sensor.distance() > stop_on_obstacle_at:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)
    
    brick.display.text('Reached obstacle' + str(obstacle_sensor.distance()))



def follow_line(robot,
    color_sensor,
    max_distance = 0, 
    stop_on_color=None,
    speed_mm_s = DEFAULT_SPEED):

    # PID tuning
    Kp = 1  # proportional gain
    Ki = 0  # integral gain
    Kd = 0  # derivative gain
    target_value = color_sensor.reflection()
    interval = 100 # millisecpnds to sample
    
    max_duration = 1000 * int(max_distance / speed_mm_s)
    cum_duration = 0
    error_increasing_cnt = 0
    integral = 0
    previous_error = 0

    while True:
        # Calculate steering using PID algorithm
        error = target_value - color_sensor.reflection()
        integral += (error * interval)
        derivative = (error - previous_error) / interval

        # u zero:     on target,  drive forward
        # u positive: too bright, turn right
        # u negative: too dark,   turn left
        u = (Kp * error) + (Ki * integral) + (Kd * derivative)
        print(' prev_err ' + str(previous_error)
            + ' err ' + str(error)
            + ' integrl ' + str(integral)
            + ' tgt ' + str(target_value)
            + ' actual ' + str(color_sensor.reflection())
            + ' deriv ' + str(derivative)
            + ' u ' + str(u)
            + ' error_cnt ' + str(error_increasing_cnt)
            + ' cum_dist ' + str(int((cum_duration * speed_mm_s)/1000))
        )

        robot.drive(speed_mm_s, u)
        wait(interval)
        cum_duration += interval

        # Check any endng conditions being met
        if ((max_distance > 0 and cum_duration >= max_duration) or 
            (stop_on_color and color_sensor.color() == stop_on_color)):
            robot.stop(stop_type=Stop.BRAKE)
            print('Stopping as end met')
            return True
 
        # If error increasing too much quit
        if error * previous_error > 0 and abs(error) >= abs(previous_error):
            error_increasing_cnt += 1
            if error_increasing_cnt > 5:
                robot.stop(stop_type=Stop.BRAKE)
                print('Too much Error , quitting')
                return False
        else:
            error_increasing_cnt = 0
 

        previous_error = error

# sweep and steop forward till color is found
def search_for_color(robot,
    color_sensor,
    stop_on_color):

    if  color_sensor.color() == stop_on_color: # if already there
        return True

 
    forward_steps =0 
    while forward_steps < 3:
        sweep_width = 1
        sweep_attempts = 0
        sweep_speed = 45

        while sweep_attempts < 5:
            common_methods.log_string('Sweep sweep_width ' + str(sweep_width))
            robot.drive_time(0, sweep_speed, sweep_width * 100) #sweep right
            if  color_sensor.color() == stop_on_color:
                robot.stop(stop_type=Stop.BRAKE)
                return True
            robot.drive_time(0, -1 * sweep_speed, sweep_width * 100) #sweep left
            if  color_sensor.color() == stop_on_color:
                robot.stop(stop_type=Stop.BRAKE)
                return True
           
            sweep_width += 1
            sweep_attempts += 1
        
        # reset to point at mid point
        robot.drive_time(0, sweep_speed, int(sweep_width * 100 / 2))
        # step forward by 1 cm to sweep again
        robot.drive_time(100, 0, 100)
        forward_steps += 1

    common_methods.sound_alarm()
    return False



# Used by line follower to align with the general direction of the line
def align_with_line_to_left(robot,
    color_sensor,
    line_color = Color.BLACK,
    border_color = Color.WHITE):

    #Find left white border of line
    move_to_color(robot, color_sensor, border_color)
    move_to_color(robot, color_sensor, line_color)
    move_to_color(robot, color_sensor, border_color)
 
    #move forward half the length of tank and rotate
    move_straight(robot, int(TANK_CHASSIS_LEN_MM/2))    
    turn_to_color_right(robot, color_sensor, border_color) 

def align_with_line_to_right(robot,
    color_sensor,
    line_color = Color.BLACK,
    border_color = Color.WHITE):

    #Find left white border of line
    move_to_color(robot=robot,color_sensor=color_sensor,
        stop_on_color=border_color)

    #move forward half the length of tank and rotate
    move_straight(robot, int(TANK_CHASSIS_LEN_MM/2))    
    turn_to_color_left(robot, color_sensor, line_color) 
    turn_to_color_left(robot, color_sensor, border_color) 



# Used by line follower to align with the general direction of the line
def align_with_line(robot,  color_sensor, line_color = Color.BLACK):

    # If not already on the line then search for it - need to start on line
    if  color_sensor.color() != line_color:
        if ( True != search_for_color(robot, color_sensor, line_color)):
            common_methods.log_string('Could not find line to follow')
            return False

    steering = 0
    while steering <=30:
        #jump forward 5cm and check if on line color
        common_methods.log_string('align right '
            + ' steer ' + str(steering))

        robot.drive_time(100, steering, 500)
        if  color_sensor.color() == line_color:
            robot.stop(stop_type=Stop.BRAKE)
            return True
        else:
            common_methods.log_string('align  right back up '
                + ' steer ' + str(steering))
            robot.drive_time(-100, steering, 500)


        # Check the other side
        if steering > 0 :
            common_methods.log_string('align  left '
                + ' steer ' + str(steering))
            robot.drive_time(100, -1 * steering, 500)
            if  color_sensor.color() == line_color:
                robot.stop(stop_type=Stop.BRAKE)
                return True
            else:
                common_methods.log_string('align  left back up '
                    + ' steer ' + str(steering))
                robot.drive_time(-100,  -1 * steering, 500)

        steering += 7 # Check 7 degrees on either side next

    return False


 
def follow_line_dark(robot,
    color_sensor,
    max_distance = 0, 
    stop_on_color=None,
    line_color = Color.BLACK,
    border_color = Color.WHITE,
    speed_mm_s = DEFAULT_SPEED):
   
    follow_attempts = 0 

    while follow_attempts < 2:
        follow_attempts += 1
        if ( True != search_for_color(robot, color_sensor, line_color)):
            common_methods.log_string('Could not find line to follow')
            return False
    
        if ( True != align_with_line(robot, color_sensor, line_color)):
            common_methods.log_string('Could not align with line')
            return False
        
        if follow_line_after_alignment(robot,
            color_sensor,
            max_distance = 0, 
            stop_on_color=None,
            line_color = Color.BLACK,
            border_color = Color.WHITE,
            speed_mm_s = speed_mm_s):
                return True
    return False

#must be on the left hite border to start this
def follow_line_border(robot,
    color_sensor,
    max_distance = 0, 
    stop_on_color=None,
    line_color = Color.BLACK,
    border_color = Color.WHITE,
    speed_mm_s = DEFAULT_LINEFOLLOW_SPEED):

    if ( True != search_for_color(robot, color_sensor, border_color)):
        common_methods.log_string('follow_line_border :Could not find line to follow')
        return False
    
    target_intensity = color_sensor.reflection()

    follow_speed_mm_s = min(speed_mm_s, DEFAULT_LINEFOLLOW_SPEED) # line follow speed is slower
    sample_distance_mm = 10  # sample every 1cm to check on track
    interval = sample_distance_mm / (speed_mm_s/1000) # millisecpnds to sample
    max_duration = 1000 * int(max_distance / speed_mm_s)
    cum_duration = 0
    intensity = color_sensor.reflection()
    off_track_cnt=0

    while True:
        intensity = color_sensor.reflection()
        current_color = color_sensor.color()
        error = target_intensity - intensity
        if current_color != line_color and current_color != border_color:
            turn = 1 #right
        elif current_color == border_color:
            turn = 0
        elif current_color == line_color :
            turn = -1 #left

        robot.drive(follow_speed_mm_s, turn * abs(error))
        wait(interval)
        cum_duration += interval
        print(' intensity ' + str(intensity)
                + ' error ' + str(error)
                + ' color ' + str(current_color)
                + ' turned ' + str(turn)
                + ' cum_dist ' + str(int((cum_duration * speed_mm_s)/1000))
            )

        # Check any endng conditions being met
        if ((max_distance > 0 and cum_duration >= max_duration) or 
            (stop_on_color and color_sensor.color() == stop_on_color)):
            robot.stop(stop_type=Stop.BRAKE)
            print('Stopping as end met')
            common_methods.sound_happy()
            return True

        prev_intensity = intensity
        prev_turn = turn





def follow_line_after_alignment(robot,
    color_sensor,
    max_distance = 0, 
    stop_on_color=None,
    line_color = Color.BLACK,
    border_color = Color.WHITE,
    speed_mm_s = DEFAULT_SPEED):

    speed_mm_s = 100 # line follow speed is slower
    sample_distance_mm =10
    interval = sample_distance_mm / (speed_mm_s/1000) # millisecpnds to sample
    max_duration = 1000 * int(max_distance / speed_mm_s)
    cum_duration = 0
    intensity = color_sensor.reflection()
    prev_intensity=intensity
    prev_turn=0
    last_turn_toward_goal=0

    while True:
        intensity = color_sensor.reflection()
        delta_intensity= intensity - prev_intensity
        current_color = color_sensor.color()



        if current_color != line_color and current_color != border_color:
            if ( True != search_for_color(robot, color_sensor, line_color)):
                common_methods.log_string('follow_line_after_alignment :Could not find line to follow')
                return False
            else:
                continue
        elif delta_intensity == 0:
            turn=0
        elif delta_intensity < 0:
            last_turn_toward_goal = prev_turn #save the last turn which took it closer
            turn=0        
        else:
            #Do the opposite of the prev turn or the opposite of the last turn with gain
            if prev_turn == 0:
                if last_turn_toward_goal == 0:
                    if ( True != search_for_color(robot, color_sensor, line_color)):
                        common_methods.log_string('follow_line_after_alignment :no prev turn, search')
                        return False
                    else:
                        continue
                else:
                    turn = -1 * last_turn_toward_goal
            else:
                turn = -1 * prev_turn
 
        robot.drive(speed_mm_s, turn)
        wait(interval)
        cum_duration += interval
        print(' intensity ' + str(intensity)
                + ' prev_int ' + str(prev_intensity)
                + ' delta_intensity ' + str(delta_intensity)
                + ' color ' + str(color_sensor.color())
                + ' turned ' + str(turn)
                + ' prev_trn ' + str(prev_turn)
                + ' cum_dist ' + str(int((cum_duration * speed_mm_s)/1000))
            )

        # Check any endng conditions being met
        if ((max_distance > 0 and cum_duration >= max_duration) or 
            (stop_on_color and color_sensor.color() == stop_on_color)):
            robot.stop(stop_type=Stop.BRAKE)
            print('Stopping as end met')
            return True

        prev_intensity = intensity
        prev_turn = turn




def follow_line_old(robot,
    left_motor, 
    right_motor, 
    color_sensor,
    color_sensor_left,
    color_sensor_right,
    max_distance = 0, 
    stop_on_color=None,
    speed_mm_s = 100):

    if ( color_sensor_left.color() != color_sensor_right.color()):
        brick.display.text('Cannot start left ' + str(color_sensor_left.color())
            + ' right :' + str(color_sensor_right.color()))
        return false

    target_color = color_sensor_right.color()
    brick.display.text('Target line color ' + str(color_sensor_left.color()))

    robot.drive(speed_mm_s, 0)

    while true:
        if (color_sensor_left.color() != color_sensor_right.color()):
            # steer
            robot.drive_time(speed_mm_s, 10, 100)
        else:
            wait(10)

