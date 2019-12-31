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


def get_robot():
    # Initialize two motors and a drive base
    wheel_diameter_mm=54
    axle_track_mm=121

    crane_motor=Motor(Port.A)
    left_motor=Motor(Port.D)
    right_motor=Motor(Port.B)
    robot = DriveBase(left_motor, right_motor, wheel_diameter_mm, axle_track_mm)
    #robot.drive(2000, 0)

    color_sensor_left = ColorSensor(Port.S3)
    color_sensor_right = ColorSensor(Port.S2)
    # Initialize the Ultrasonic Sensor. 
    # obstacle_sensor = UltrasonicSensor(Port.S4)
    return robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right
