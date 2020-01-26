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
from shared_all import *

def traffic():
    turn_to_direction(gyro, 45)
    move_crane_to_floor( crane_motor)
    move_straight(100)
    crane_motor.run(20)
    move_straight(max_distance=60, speed_mm_s = 50)
    crane_motor.stop(Stop.BRAKE)
    move_reverse(150)
    turn_to_direction(gyro, 0)
    move_crane_to_floor( crane_motor)
    move_crane_up( crane_motor, 130)


def swing():
    move_crane_to_floor( crane_motor)
    move_crane_up( crane_motor, 20)
    turn_to_direction(gyro, 45)
    move_straight(100)
    move_reverse(150)
    move_crane_up( crane_motor, 130)

def elevator():
    turn_to_direction(gyro, 0)
    move_straight(300)
    move_crane_down( crane_motor, 90)
    move_reverse(90, speed_mm_s=70)
    move_crane_up( crane_motor, 90)
    move_reverse(210)
    turn_to_direction(gyro, -90)

def steel():
    turn_to_direction(gyro, 0)
    move_straight(100)
    turn( angle=-40)
    move_crane_to_floor(crane_motor)
    turn(angle=40)

    crane_motor.run(40)
    move_straight(max_distance=130, speed_mm_s = 50)
    crane_motor.stop(Stop.BRAKE)
    move_crane_down(crane_motor, degrees=30)
    move_straight(max_distance=60)
    move_reverse(100)


move_straight(400)

traffic()

move_straight(1100)

swing()

turn_to_direction(gyro, 0)
move_reverse(150)
turn_to_direction(gyro, -90)
move_straight(300)

elevator()

turn_to_direction(gyro, 0)
turn_to_direction(gyro, -145)
move_straight(300)

steel()

turn_to_direction(gyro, 90)
move_straight(700)
run_to_home()
