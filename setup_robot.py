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

def calibrate_gyro(gyro, new_angle=0):
    current_speed=gyro.speed()
    current_angle=gyro.angle()
    message = 'calibrating gyro speed ' + str(current_speed) + ' angle:' + str(current_angle)
    print(message)
    brick.display.text(message)
    wait(200)
    message = 'Resetting gyro to ' + str(new_angle) 
    print(message)
    brick.display.text(message)
    gyro.reset_angle(new_angle)
    wait(150)
    message = 'Reset gyro complete to ' + str(new_angle) 
    print(message)
    brick.display.text(message)
    current_speed=gyro.speed()
    current_angle=gyro.angle()
    message = 'After reset gyro speed ' + str(current_speed) + ' angle:' + str(current_angle)
    print(message)
    brick.display.text(message)
    wait(150)


def get_robot():
    # Initialize two motors and a drive base
    wheel_diameter_mm=54
    axle_track_mm=121

    crane_motor=Motor(Port.A)
    left_motor=Motor(Port.C)
    right_motor=Motor(Port.B)
    robot = DriveBase(left_motor, right_motor, wheel_diameter_mm, axle_track_mm)
    #robot.drive(2000, 0)

    gyro=GyroSensor(Port.S1)
    calibrate_gyro(gyro)
    color_sensor_left = ColorSensor(Port.S3)
    color_sensor_right = None #ColorSensor(Port.S2)
    # Initialize the Ultrasonic Sensor. 
    # obstacle_sensor = UltrasonicSensor(Port.S4)
    return robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right, gyro
