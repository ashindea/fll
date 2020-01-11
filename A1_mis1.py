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


def gyro_color_test_angled():
    shared_all.log_string('Pre-run      gyro speed ' + str(shared_all.gyro.speed()) + ' angle:' + str(shared_all.gyro.angle()))
    shared_all.move_crane_to_floor(crane_motor)
    shared_all.move_crane_up(crane_motor, degrees=135)

    shared_all.move_straight(max_distance=800)
    shared_all.log_string('post Run speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    shared_all.turn_to_direction(gyro, target_angle=45)
    shared_all.log_string('post +45 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    shared_all.move_to_color(color_sensor=color_sensor_left,
        stop_on_color=Color.BLACK, speed_mm_s = 70)


    shared_all.turn_to_direction(gyro, target_angle=90)
    shared_all.log_string('post +90 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    shared_all.move_straight(max_distance=100)
    shared_all.move_crane_to_floor(crane_motor)
    shared_all.move_crane_up(crane_motor, degrees=45)
    shared_all.move_crane_to_floor(crane_motor)
    shared_all.move_reverse(max_distance=100)
    shared_all.move_crane_up(crane_motor, degrees=135)


    shared_all.turn_to_direction(gyro, target_angle=210)
    shared_all.log_string('post +210 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    shared_all.move_straight(max_distance=1300)
    shared_all.log_string('post Run speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))


gyro_color_test_angled()

# do_find_line_then_follow()
