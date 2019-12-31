
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

def sound_happy():
    brick.sound.beep(1100, 80, 7)
    brick.sound.beep(900, 80, 7)

def sound_alarm():
    brick.sound.beep(300, 150, 7)
    wait(200)
    brick.sound.beep(300, 150, 7)
    wait(200)
    brick.sound.beep(300, 150, 7)

def log_string(message):
    print(message)
    brick.display.text(message)
