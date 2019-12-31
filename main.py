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

#from sensor1 import create_sound
import setup_robot
import move_robot
import worker
import sensor1
import common_methods

def do_move_return(robot):
    move_robot.move_straight(robot=robot, max_distance=200, speed_mm_s=200)
    move_robot.turn(robot=robot, angle=90)

    worker.move_crane_to_floor(robot, crane_motor)
    worker.move_crane_up(robot, crane_motor, degrees=45)
    #move_robot.turn(robot=robot, angle=180)
    move_robot.turn(robot=robot, angle=90)

    worker.move_crane_up(robot, crane_motor, degrees=45)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_down(robot, crane_motor, degrees=25)
    move_robot.turn(robot=robot, angle=90)

    worker.move_crane_up(robot, crane_motor, degrees=45)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_down(robot, crane_motor, degrees=25)
    move_robot.turn(robot=robot, angle=90)

    move_robot.move_straight(robot, max_distance = 200, speed_mm_s = 300)


def do_color_dance(robot, color_sensor_left):
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.RED)#, speed_mm_s = 5)

    move_robot.move_to_color_reverse(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.RED)

def do_crane_dance(robot, crane_motor):
    move_robot.move_straight(robot=robot, max_distance=900, speed_mm_s=300)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_to_floor(robot, crane_motor)
    move_robot.move_straight(robot=robot, max_distance=80)

    worker.move_crane_up(robot, crane_motor, degrees=45)
    worker.move_crane_to_floor(robot, crane_motor)
    
    move_robot.move_reverse(robot=robot, max_distance=100)
    worker.move_crane_up(robot, crane_motor, degrees=90)

    move_robot.turn(robot=robot, angle=-90)
    move_robot.move_straight(robot=robot, max_distance=300)

    move_robot.turn(robot=robot, angle=180)
    move_robot.move_straight(robot=robot, max_distance=700)
 
    move_robot.turn(robot=robot, angle=50)
   
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.BLACK, speed_mm_s=350)
    worker.move_crane_down(robot, crane_motor, degrees=60)
    worker.move_crane_up(robot, crane_motor, degrees=60)


def do_line_follow(robot, color_sensor):
    move_robot.follow_line_dark(robot = robot,
        color_sensor = color_sensor,
        max_distance = 300, 
        stop_on_color=None,
        speed_mm_s = 100)

def do_find_line_then_follow(robot, color_sensor):
    print ('starting do_find_line_then_follow')
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor,
        stop_on_color=Color.BLACK)
    wait(1000)
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor,
        stop_on_color=Color.WHITE)
    """
    move_robot.follow_line_border(robot = robot,
            color_sensor = color_sensor,
            max_distance = 300, 
            stop_on_color=None,
            line_color = Color.BLACK,
            border_color = Color.WHITE,
            speed_mm_s = 100)
"""
#
# Write your program here
brick.sound.beep(600, 200, 7)
brick.display.text('Driving forward')

robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right=setup_robot.get_robot()

#do_crane_dance(robot, crane_motor)
#do_color_dance(robot, color_sensor_left)
#do_line_follow(robot, color_sensor=color_sensor_left)
do_find_line_then_follow(robot, color_sensor=color_sensor_left)
#do_move_return(robot)

#move_robot.search_for_color(robot, color_sensor_left, Color.BLACK)
"""
move_robot.move_to_color(robot, color_sensor_left, Color.BLACK)
move_robot.move_to_color(robot, color_sensor_left, Color.WHITE)
move_robot.follow_line_border
"""
#move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,stop_on_color=Color.WHITE)

#move_robot.follow_line_after_alignment(robot,color_sensor_left, max_distance=400)
wait(2000)
common_methods.sound_happy()
