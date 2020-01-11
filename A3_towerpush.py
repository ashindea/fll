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

def do_crane_push():
    shared_all.move_crane_to_floor(crane_motor)
    shared_all.move_crane_up(crane_motor, degrees=135)

    shared_all.move_straight(max_distance=60)
    shared_all.turn( angle=-50)
    # shared_all.move_straight(max_distance=100)
    shared_all.move_crane_to_floor(crane_motor)
    shared_all.turn(angle=50)

    crane_motor.run(40)
    shared_all.move_straight(max_distance=130, speed_mm_s = 50)
    crane_motor.stop(Stop.BRAKE)
    shared_all.move_crane_down(crane_motor, degrees=30)
    shared_all.move_straight(max_distance=60)
    """for x in range(5):
        worker.move_crane_up(robot, crane_motor, degrees=20)
        move_robot.move_straight(robot=robot, max_distance=40)"""

    shared_all.move_reverse(max_distance=350)

    """worker.move_crane_up(robot, crane_motor, degrees=20)
    move_robot.move_straight(robot=robot, max_distance=50)"""

    #move_robot.turn(robot=robot, angle=30)

do_crane_push()