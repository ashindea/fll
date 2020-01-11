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
import shared_all
from shared_all import crane_motor
from shared_all import gyro
from shared_all import color_sensor_left



def do_find_line_then_follow():
    shared_all.align_with_line_to_right(color_sensor_left)
    shared_all.follow_line_border(color_sensor_left,max_distance = 200)


do_find_line_then_follow()
