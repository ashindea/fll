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


def turn(robot, angle, 
    speed_mm_s = 100):
    robot.drive_time(speed_mm_s, angle, 1000)

def move_straight(robot,
    max_distance, 
    speed_mm_s = 100,
    stop_on_color = None, 
    stop_on_obstacle_at = -1):

    if (max_distance < 0 ):
        # moving in reverse
        speed_mm_s = -1 * speed_mm_s

    duration = 1000 * abs(int(max_distance / speed_mm_s))

    # Drive forward
    brick.display.text('Driving for left and right' + str(200))
 
    #robot.drive(speed = 200, steering = 0)
    robot.drive_time(speed_mm_s, 0, duration)
    robot.stop(stop_type=Stop.BRAKE)
 

def move_to_color(robot,
    color_sensor,
    stop_on_color,
    speed_mm_s = 200):

    brick.display.text('Driving to color' + str(stop_on_color))
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)
    
    brick.display.text('Reached color' + str(color_sensor.color()))

def move_to_color_reverse(robot,
    color_sensor,
    stop_on_color,
    speed_mm_s = 200):
    move_to_color(robot,
        color_sensor,
        stop_on_color,
        speed_mm_s = -1 * speed_mm_s)

def move_to_obstacle(robot,
    obstacle_sensor,
    stop_on_obstacle_at,
    speed_mm_s = 200):

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
    speed_mm_s = 100):

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



def follow_line_dark(robot,
    color_sensor,
    max_distance = 0, 
    stop_on_color=None,
    speed_mm_s = 100):
   
    sample_distance_mm =10
    interval = sample_distance_mm / (speed_mm_s/1000) # millisecpnds to sample
    max_duration = 1000 * int(max_distance / speed_mm_s)
    cum_duration = 0
    prev_intensity=intensity
    prev_turn=0

    while True:
        intensity = color_sensor.reflection()
        delta_intensity= intensity - prev_intensity

        if ( delta_intensity >= 0):
            turn=0
        else:
            #Do the opposite of the prev turn or turn one way
            if prev_turn == 0:
                turn = delta_intensity
            else
                turn = -1 * prev_turn
 
        robot.drive(speed_mm_s, turn)
        wait(interval)
        cum_duration += interval
        print(' intensity ' + str(intensity)
                + ' prev_int ' + str(prev_intensity)
                + ' turn ' + str(turn)
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

