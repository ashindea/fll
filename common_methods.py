
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

SOUND_VOLUME=7

def sound_happy():
    brick.sound.beep(1100, 80, SOUND_VOLUME)
    brick.sound.beep(900, 80, SOUND_VOLUME)

def sound_attention():
    brick.sound.beep(700, 80, SOUND_VOLUME)
    brick.sound.beep(1200, 80, SOUND_VOLUME)

def sound_alarm():
    brick.sound.beep(300, 150, SOUND_VOLUME)
    wait(200)
    brick.sound.beep(300, 150, SOUND_VOLUME)
    wait(200)
    brick.sound.beep(300, 150, SOUND_VOLUME)

def log_string(message):
    print(message)
    brick.display.text(message)
