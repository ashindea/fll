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


def gyro_test(robot, gyro):
    move_straight(max_distance=600)
    log_string(' Face east (0) ' + str(gyro.angle()))
    move_reverse(150)
    turn_to_direction(gyro, 90)
    log_string(' Face south (90) ' + str(gyro.angle()))
    turn(40)
    log_string(' Face SW (130) ' + str(gyro.angle()))
    move_straight(max_distance=300)
    turn_to_direction(gyro, -90)
    log_string(' Face N (-90) ' + str(gyro.angle()))
    move_straight(max_distance=800)
    log_string(' Still Face N (-90) ' + str(gyro.angle()))
    turn_to_direction(gyro, 180)
    log_string(' Face W (-180) ' + str(gyro.angle()))
    move_straight(max_distance=600)
    turn(-45)
    log_string(' Face SW (135) ' + str(gyro.angle()))
    move_straight(max_distance=600)
    log_string(' Still Face SW (135) ' + str(gyro.angle()))
    turn_to_direction(gyro, -135)
    log_string(' Face NW (-135) ' + str(gyro.angle()))
    move_straight(max_distance=300)
    log_string(' Still Face NW (-135) ' + str(gyro.angle()))

gyro_test(robot, gyro)
