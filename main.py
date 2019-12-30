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


def do_color_dance(robot, color_sensor_left):
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.RED)

    move_robot.move_to_color_reverse(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.BLACK)

def do_crane_dance(robot, crane_motor):
    move_robot.move_straight(robot=robot, max_distance=300)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_to_floor(robot, crane_motor)
    move_robot.move_straight(robot=robot, max_distance=80)
    worker.move_crane_up(robot, crane_motor, degrees=45)
    worker.move_crane_to_floor(robot, crane_motor)
    move_robot.move_reverse(robot=robot, max_distance=100)
    worker.move_crane_up(robot, crane_motor, degrees=90)

    move_robot.turn_reverse(robot=robot, angle=-90)

    move_robot.turn(robot=robot, angle=180)
    move_robot.move_straight(robot=robot, max_distance=150)
    worker.move_crane_down(robot, crane_motor, degrees=60)
    worker.move_crane_up(robot, crane_motor, degrees=60)


def do_line_follow(robot, color_sensor):
    move_robot.follow_line_dark(robot = robot,
        color_sensor = color_sensor,
        max_distance = 300, 
        stop_on_color=None,
        speed_mm_s = 100)

def do_find_line_then_follow(robot, color_sensor):
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.BLACK)
    wait(2000)
    move_robot.follow_line_dark(robot = robot,
        color_sensor = color_sensor,
        max_distance = 300, 
        stop_on_color=None,
        speed_mm_s = 100)

#
# Write your program here
brick.sound.beep(600, 200, 7)
brick.display.text('Driving forward')

robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right=setup_robot.get_robot()

#do_crane_dance(robot, crane_motor)
#do_color_dance(robot, color_sensor_left)
#do_line_follow(robot, color_sensor=color_sensor_left)
#do_find_line_then_follow(robot, color_sensor=color_sensor_left)

move_robot.search_for_color(robot, color_sensor_left, Color.BLACK)

wait(20000)

