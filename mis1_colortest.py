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


def color_test(robot, color_sensor):
    robot.drive(50, 0)
    for x in range(20):
        common_methods.log_string('color ' + str(color_sensor.color()) + ' intens:' + str(color_sensor.reflection()))
        wait(100)
    robot.stop(stop_type=Stop.BRAKE)


common_methods.sound_attention()
robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right=setup_robot.get_robot()
color_test(robot, color_sensor_left)
common_methods.sound_happy()
wait(10000)


