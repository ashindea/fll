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


#
# Write your program here
brick.sound.beep(1000, 600)
brick.display.text('Driving forward')
wait(1000)
brick.display.text('Starting')

robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right=setup_robot.get_robot()

#move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
#    stop_on_color=Color.RED)

#move_robot.move_to_color_reverse(robot=robot,color_sensor=color_sensor_left,
#    stop_on_color=Color.BLACK)

move_robot.move_straight(robot=robot, max_distance=100)
move_robot.turn(robot=robot, angle=90)
worker.move_crane_down(robot, crane_motor, degrees=45)
worker.move_crane_up(robot, crane_motor, degrees=45)
move_robot.move_straight(robot=robot, max_distance=-100)
move_robot.turn(robot=robot, angle=180)
worker.move_crane_down(robot, crane_motor, degrees=90)

wait(20000)

