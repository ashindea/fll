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

def zigzag():
    robot.drive(90, -15)
    wait(1000)
    robot.drive(90, 15)
    wait(1000)
    robot.drive(90, -15)
    wait(1000)
    robot.drive(90, 15)
 


def do_move_return(robot):
    move_robot.move_straight(robot=robot, max_distance=200, speed_mm_s=200)
    move_robot.turn(robot=robot, angle=90)

    worker.move_crane_to_floor(robot, crane_motor)
    worker.move_crane_up(robot, crane_motor, degrees=45)
    #move_robot.turn(robot=robot, angle=180)
    move_robot.turn(robot=robot, angle=90)

    worker.move_crane_up(robot, crane_motor, degrees=45)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_down(robot, crane_motor, degrees=25)
    move_robot.turn(robot=robot, angle=90)

    worker.move_crane_up(robot, crane_motor, degrees=45)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_down(robot, crane_motor, degrees=25)
    move_robot.turn(robot=robot, angle=90)

    move_robot.move_straight(robot, max_distance = 200, speed_mm_s = 300)

def do_align_to_left_then_right():
    move_robot.align_with_line_to_left(robot, color_sensor=color_sensor_left)
    wait(4000)
    move_robot.align_with_line_to_right(robot, color_sensor=color_sensor_left)

def color_test(robot, color_sensor):
    common_methods.log_string('color ' + str(color_sensor.color()) + ' intens:' + str(color_sensor.reflection()))
    robot.drive(50, 0)
    for x in range(20):
        common_methods.log_string('color ' + str(color_sensor.color()) + ' intens:' + str(color_sensor.reflection()))
        wait(100)
    robot.stop(stop_type=Stop.BRAKE)

def do_color_dance(robot, color_sensor_left):
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.RED)#, speed_mm_s = 5)

    move_robot.move_to_color_reverse(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.RED)

def do_crane_dance(robot, crane_motor):
    move_robot.move_straight(robot=robot, max_distance=500, speed_mm_s=300)
    move_robot.turn(robot=robot, angle=90)
    worker.move_crane_to_floor(robot, crane_motor)
    move_robot.move_straight(robot=robot, max_distance=80)

    worker.move_crane_up(robot, crane_motor, degrees=45)
    worker.move_crane_to_floor(robot, crane_motor)
    
    move_robot.move_reverse(robot=robot, max_distance=100)
    worker.move_crane_up(robot, crane_motor, degrees=90)

    move_robot.turn(robot=robot, angle=-90)
    move_robot.move_straight(robot=robot, max_distance=300)

    move_robot.turn(robot=robot, angle=180)
    move_robot.move_straight(robot=robot, max_distance=300)
 
    move_robot.turn(robot=robot, angle=50)
   
    move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,
        stop_on_color=Color.BLACK, speed_mm_s=350)
    worker.move_crane_down(robot, crane_motor, degrees=60)
    worker.move_crane_up(robot, crane_motor, degrees=60)




def do_find_line_then_follow(robot, color_sensor):
    move_robot.align_with_line_to_right(robot, color_sensor_left)
    move_robot.follow_line_border(robot,color_sensor_left,max_distance = 200)

def gyro_test(robot, gyro):
    common_methods.log_string('Pre-run      gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.move_straight(robot=robot, max_distance=300)
    common_methods.log_string('post Run speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))
 
    move_robot.turn_to_direction(robot, gyro, target_angle=90)
    common_methods.log_string('post +90 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.turn_to_direction(robot=robot, gyro=gyro, target_angle=180)
    common_methods.log_string('post U turn   gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.move_straight(robot=robot, max_distance=300)
    common_methods.log_string('post st. run  gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.turn_to_direction(robot=robot, gyro=gyro, target_angle=-90)
    common_methods.log_string('post -90 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))
"""
"""

def gyro_color_test_angled(robot, gyro, color_sensor, crane_motor):
    common_methods.log_string('Pre-run      gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))
    worker.move_crane_to_floor(robot, crane_motor)
    worker.move_crane_up(robot, crane_motor, degrees=135)

    move_robot.move_straight(robot=robot, max_distance=800)
    common_methods.log_string('post Run speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.turn_to_direction(robot, gyro, target_angle=45)
    common_methods.log_string('post +45 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.move_to_color(robot=robot,color_sensor=color_sensor,
        stop_on_color=Color.BLACK, speed_mm_s = 70)


    move_robot.turn_to_direction(robot, gyro, target_angle=90)
    common_methods.log_string('post +90 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.move_straight(robot=robot, max_distance=100)
    worker.move_crane_to_floor(robot, crane_motor)
    worker.move_crane_up(robot, crane_motor, degrees=45)
    worker.move_crane_to_floor(robot, crane_motor)
    move_robot.move_reverse(robot=robot, max_distance=100)
    worker.move_crane_up(robot, crane_motor, degrees=135)


    move_robot.turn_to_direction(robot, gyro, target_angle=210)
    common_methods.log_string('post +210 turn gyro speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))

    move_robot.move_straight(robot=robot, max_distance=700)
    common_methods.log_string('post Run speed ' + str(gyro.speed()) + ' angle:' + str(gyro.angle()))
"""
"""

def do_crane_push(robot, gyro, color_sensor, crane_motor):
    worker.move_crane_to_floor(robot, crane_motor)
    worker.move_crane_up(robot, crane_motor, degrees=10)

    move_robot.move_straight(robot=robot, max_distance=50)
    move_robot.turn(robot=robot, angle=30)
    move_robot.move_straight(robot=robot, max_distance=100)
    move_robot.turn(robot=robot, angle=-40)

    crane_motor.run(40)
    move_robot.move_straight(robot=robot, max_distance=130, speed_mm_s = 50)
    crane_motor.stop(Stop.BRAKE)
    worker.move_crane_down(robot, crane_motor, degrees=30)
    move_robot.move_straight(robot=robot, max_distance=60)
    """for x in range(5):
        worker.move_crane_up(robot, crane_motor, degrees=20)
        move_robot.move_straight(robot=robot, max_distance=40)"""

    move_robot.move_reverse(robot=robot, max_distance=350)

    """worker.move_crane_up(robot, crane_motor, degrees=20)
    move_robot.move_straight(robot=robot, max_distance=50)"""

    #move_robot.turn(robot=robot, angle=30)

#
# Write your program here
common_methods.sound_attention()

robot, crane_motor, left_motor, right_motor,color_sensor_left, color_sensor_right, gyro =setup_robot.get_robot()

#do_crane_dance(robot, crane_motor)
#do_color_dance(robot, color_sensor_left)
#do_find_line_then_follow(robot, color_sensor=color_sensor_left)
#do_move_return(robot)
#zigzag()
#do_align_to_left_then_right()
#color_test(robot, color_sensor_left)
#gyro_test(robot, gyro)
#gyro_color_test_angled(robot, gyro, color_sensor_left, crane_motor)
do_crane_push(robot, gyro, color_sensor_left, crane_motor)
"""
#move_robot.move_to_color(robot, color_sensor_left, Color.WHITE)

#move_robot.move_to_color(robot, color_sensor_left, Color.BLACK)
#move_robot.move_to_color(robot, color_sensor_left, Color.WHITE)

move_robot.move_straight(robot, int(200/2))    

move_robot.turn_to_color_right(robot, color_sensor_left, Color.WHITE) 
"""
#move_robot.align_with_line_to_right(robot, color_sensor=color_sensor_left)
#move_robot.follow_line_border(robot,color_sensor_left,max_distance = 200)

#move_robot.turn_to_color_left(robot, color_sensor_left, Color.WHITE) 

#move_robot.search_for_color(robot, color_sensor_left, Color.BLACK)
"""
move_robot.move_to_color(robot, color_sensor_left, Color.BLACK)
move_robot.move_to_color(robot, color_sensor_left, Color.WHITE)
"""
#move_robot.move_to_color(robot=robot,color_sensor=color_sensor_left,stop_on_color=Color.WHITE)

#move_robot.follow_line_after_alignment(robot,color_sensor_left, max_distance=400)
wait(2000)
common_methods.sound_happy()
