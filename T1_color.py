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


def color_test(robot, color_sensor):
    log_string('color ' + str(color_sensor.color()) + ' intens:' + str(color_sensor.reflection()))
    robot.drive(50, 0)
    for x in range(20):
        log_string('color ' + str(color_sensor.color()) + ' intens:' + str(color_sensor.reflection()))
        wait(100)
    robot.stop(stop_type=Stop.BRAKE)

color_test(robot=robot, color_sensor=color_sensor_left)


wait(120000)
