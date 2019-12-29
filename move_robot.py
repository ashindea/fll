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
    color_sensor_left,
    color_sensor_right,
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

