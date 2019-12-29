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


def create_sound(freq, duration):
    brick.sound.beep(freq, duration)

def return_home():
    # Initialize two motors and a drive base
    left = Motor(Port.B)
    right = Motor(Port.C)
    robot = DriveBase(left, right, 56, 114)
    # Initialize a sensor
    sensor = UltrasonicSensor(Port.S4)
    # Drive forward until an object is detected
    robot.drive(100, 0)
    while sensor.distance() > 500:
        wait(10)
    robot.stop()