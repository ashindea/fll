
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

def move_crane_to_floor(robot, crane_motor):
    crane_motor.run_until_stalled(-180, Stop.COAST, 50)
    move_crane_up(robot, crane_motor, degrees = 10)


def move_crane_up(robot, crane_motor, degrees):
    brick.display.text('Angle at start ' + str(crane_motor.angle()))
    wait(100)
    crane_motor.run_angle(90,  degrees, Stop.BRAKE)
    brick.display.text('Angle at end ' + str(crane_motor.angle()))

def move_crane_down(robot, crane_motor, degrees):
    brick.display.text('Down Angle at start ' + str(crane_motor.angle()))
    wait(100)
    crane_motor.run_angle(90,  -1 * degrees)
    brick.display.text('down Angle at end ' + str(crane_motor.angle()))

