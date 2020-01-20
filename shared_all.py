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


DEFAULT_SPEED=170
DEFAULT_COLOR_FIND_SPEED=100
DEFAULT_LINEFOLLOW_SPEED=100
DEFAULT_ANGULAR_SPEED=45
TANK_CHASSIS_LEN_MM=200
SENSOR_TO_AXLE=60
# WHEEL_DIAMETER_MM=54
# AXLE_TRACK_MM=121
WHEEL_DIAMETER_MM=89
AXLE_TRACK_MM=160

SOUND_VOLUME=7

#output
crane_motor=Motor(Port.D)
# side_crane=Motor(Port.C)

#drive motors
left_motor=Motor(Port.C, Direction.COUNTERCLOCKWISE)
# left_motor.duty(75)
right_motor=Motor(Port.B, Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)

#Sensors
gyro=GyroSensor(Port.S1)
color_sensor_left = ColorSensor(Port.S2)
color_sensor_right = ColorSensor(Port.S3)
# Initialize the Ultrasonic Sensor. 
# obstacle_sensor = UltrasonicSensor(Port.S4)


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

def calibrate_gyro(new_angle=0):
    current_speed=gyro.speed()
    current_angle=gyro.angle()
    log_string('calibrating gyro speed ' + str(current_speed) + ' angle:' + str(current_angle))
    wait(100)
    log_string('Resetting gyro to ' + str(new_angle)) 
    gyro.reset_angle(new_angle)
    wait(150)
    log_string('Reset gyro complete to ' + str(new_angle)) 
    current_speed=gyro.speed()
    current_angle=gyro.angle()
    log_string('After reset gyro speed ' + str(current_speed) + ' angle:' + str(current_angle))
    wait(150)

def turn_to_direction( gyro, target_angle, speed_mm_s = DEFAULT_SPEED):
    start_angle = gyro.angle()
    angle_change = target_angle - start_angle

    if (angle_change >180 ):
        angle_change = angle_change - 360
    if (angle_change < -180 ):
        angle_change = angle_change + 360
    target_angle = angle_change + start_angle


    robot.drive_time(0, 0.9 * angle_change, 1000)
    robot.stop(stop_type=Stop.BRAKE)

    max_attempts=10 # limit oscialltions to 10, not forever
    while ( abs(target_angle - gyro.angle()) > 1 and max_attempts >0):
        error=target_angle - gyro.angle()
        adj_angular_speed = error * 1.5
        robot.drive(0, adj_angular_speed)
        wait(100)
        max_attempts -= 1

    robot.stop(stop_type=Stop.BRAKE)

    adjusted_angle = gyro.angle()
    log_string('turn_to_direction -- Adjusted target: ' + str(target_angle) 
        + ' now: ' + str(adjusted_angle)
        + ' remain attempts : ' + str(max_attempts)
        )




def turn( angle, speed_mm_s = DEFAULT_SPEED):

    if angle > 0:    # right turns are a bit under-steered
        angle = int(1.1 * angle)
    else:
        angle = int(angle / 1)

    robot.drive_time(0, angle, 1000)
    robot.stop(stop_type=Stop.BRAKE)

def move_reverse(
    max_distance, 
    speed_mm_s = DEFAULT_SPEED):
    move_straight( -1 * max_distance, speed_mm_s)

def move_straight(
    max_distance, 
    speed_mm_s = DEFAULT_SPEED):

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    log_string('Move stratight at speed '+ str(speed_mm_s) + ' dist ' + str(max_distance))
    if (max_distance < 0 ):
        # moving in reverse
        speed_mm_s = -1 * speed_mm_s

    duration = abs(int(1000 * max_distance / speed_mm_s))
    robot.drive_time(speed_mm_s, 0, duration)
    robot.stop(stop_type=Stop.BRAKE)
    log_string('Move stratight Done left motangle '+ str(left_motor.angle()) + ' right motangle ' + str(right_motor.angle()))
 

def turn_to_color(
    color_sensor,
    stop_on_color,
    rotate_dir = 1,
    angular_speed_deg_s = DEFAULT_ANGULAR_SPEED):
 
    robot.drive(0, rotate_dir * angular_speed_deg_s)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)

def turn_to_color_right(
    color_sensor,
    stop_on_color,
    angular_speed_deg_s = DEFAULT_ANGULAR_SPEED):
 
    turn_to_color( color_sensor, stop_on_color, 1, angular_speed_deg_s)

def turn_to_color_left(
    color_sensor,
    stop_on_color,
    angular_speed_deg_s = DEFAULT_ANGULAR_SPEED):
 
    turn_to_color( color_sensor, stop_on_color, -1, angular_speed_deg_s )


def move_to_color(
    color_sensor,
    stop_on_color,
    speed_mm_s = DEFAULT_COLOR_FIND_SPEED):
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        log_string('color: ' + str(color_sensor.color()) + ' intens: ' + str(color_sensor.reflection()))
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)



def move_to_color_reverse(
    color_sensor,
    stop_on_color,
    speed_mm_s = DEFAULT_COLOR_FIND_SPEED):
    move_to_color(
        color_sensor,
        stop_on_color,
        speed_mm_s = -1 * speed_mm_s)

def move_to_obstacle(
    obstacle_sensor,
    stop_on_obstacle_at,
    speed_mm_s = DEFAULT_SPEED):

    log_string('Driving to obstacle' + str(stop_on_obstacle_at))
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while obstacle_sensor.distance() > stop_on_obstacle_at:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)
    
    log_string('Reached obstacle' + str(obstacle_sensor.distance()))



# sweep and steop forward till color is found
def search_for_color(
    color_sensor,
    stop_on_color):

    if  color_sensor.color() == stop_on_color: # if already there
        return True

 
    forward_steps =0 
    while forward_steps < 3:
        sweep_width = 1
        sweep_attempts = 0
        sweep_speed = 45

        while sweep_attempts < 5:
            log_string('Sweep sweep_width ' + str(sweep_width))
            robot.drive_time(0, sweep_speed, sweep_width * 100) #sweep right
            if  color_sensor.color() == stop_on_color:
                robot.stop(stop_type=Stop.BRAKE)
                return True
            robot.drive_time(0, -1 * sweep_speed, sweep_width * 100) #sweep left
            if  color_sensor.color() == stop_on_color:
                robot.stop(stop_type=Stop.BRAKE)
                return True
           
            sweep_width += 1
            sweep_attempts += 1
        
        # reset to point at mid point
        robot.drive_time(0, sweep_speed, int(sweep_width * 100 / 2))
        # step forward by 1 cm to sweep again
        robot.drive_time(100, 0, 100)
        forward_steps += 1

    sound_alarm()
    return False



# Used by line follower to align with the general direction of the line
def align_with_line_to_left(
    color_sensor,
    line_color = Color.BLACK,
    border_color = Color.WHITE):

    #Find left white border of line
    move_to_color( color_sensor, border_color)
    move_to_color( color_sensor, line_color)
    move_to_color( color_sensor, border_color)
 
    #move forward half the length of tank and rotate
    move_straight( SENSOR_TO_AXLE)    
    turn_to_color_right( color_sensor, border_color) 

def align_with_line_to_right(
    color_sensor,
    line_color = Color.BLACK,
    border_color = Color.WHITE):

    #Find left white border of line
    move_to_color(color_sensor=color_sensor,
        stop_on_color=border_color)

    #move forward half the length of tank and rotate
    move_straight( SENSOR_TO_AXLE)    
    turn_to_color_left( color_sensor, line_color) 
    turn_to_color_left( color_sensor, border_color) 




#must be on the left border to start
def follow_line_border(
    color_sensor,
    max_distance = 0, 
    stop_on_color=None,
    line_color = Color.BLACK,
    border_color = Color.WHITE,
    speed_mm_s = DEFAULT_LINEFOLLOW_SPEED):

    if ( True != search_for_color( color_sensor, border_color)):
        log_string('follow_line_border :Could not find line to follow')
        return False
    
    target_intensity = color_sensor.reflection()

    follow_speed_mm_s = min(speed_mm_s, DEFAULT_LINEFOLLOW_SPEED) # line follow speed is slower
    sample_distance_mm = 10  # sample every 1cm to check on track
    interval = sample_distance_mm / (speed_mm_s/1000) # millisecpnds to sample
    max_duration = 1000 * int(max_distance / speed_mm_s)
    cum_duration = 0
    intensity = color_sensor.reflection()

    while True:
        intensity = color_sensor.reflection()
        current_color = color_sensor.color()
        error = target_intensity - intensity
        if current_color != line_color and current_color != border_color:
            turn = 1 #right
        elif current_color == border_color:
            turn = 0
        elif current_color == line_color :
            turn = -1 #left

        robot.drive(follow_speed_mm_s, turn * abs(error))
        wait(interval)
        cum_duration += interval
        log_string('follow_line_border intensity ' + str(intensity)
                + ' error ' + str(error)
                + ' color ' + str(current_color)
                + ' turned ' + str(turn)
                + ' cum_dist ' + str(int((cum_duration * speed_mm_s)/1000))
            )

        # Check any endng conditions being met
        if ((max_distance > 0 and cum_duration >= max_duration) or 
            (stop_on_color and color_sensor.color() == stop_on_color)):
            robot.stop(stop_type=Stop.BRAKE)
            log_string('follow_line_border Stopping as end met')
            sound_happy()
            return True

        prev_intensity = intensity
        prev_turn = turn








def move_crane_to_floor( crane_motor):
    crane_motor.run_until_stalled(-180, Stop.COAST, 50)
    move_crane_up( crane_motor, degrees = 10)


def move_crane_up( crane_motor, degrees):
    log_string('Angle at start ' + str(crane_motor.angle()))
    wait(100)
    crane_motor.run_angle(90,  degrees, Stop.BRAKE)
    log_string('Angle at end ' + str(crane_motor.angle()))

def move_crane_down( crane_motor, degrees):
    log_string('Down Angle at start ' + str(crane_motor.angle()))
    wait(100)
    crane_motor.run_angle(90,  -1 * degrees)
    log_string('down Angle at end ' + str(crane_motor.angle()))

### Run this at start up
calibrate_gyro()
