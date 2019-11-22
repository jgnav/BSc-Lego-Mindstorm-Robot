#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
import math.pi as PI

class robot:
    #Inicialización
    def __init__(self , left_motor, right_motor, wheel_diameter, axle_distance, sonar, color_sensor):
        self.left_motor = Motor(left_motor)
        self.right_motor = Motor(right_motor)
        self.r=wheel_diameter
        self.d=axle_distance
        self.sonar = UltrasonicSensor(sonar)
        self.color_sensor = ColorSensor(color_sensor)

    #Métodos motores por separado
    def run_rightMotor(self, speed):
        self.right_motor.on(SpeedRPS(speed/(2*PI)))

    def run_leftMotor(self, speed):
        self.left_motor.on(SpeedRPS(speed/(2*PI)))

    def run_time_rightMotor(self, speed, seconds):
        self.right_motor.on_for_seconds(SpeedRPS(speed/(2*PI)), seconds)

    def run_time_leftMotor(self, speed, seconds):
        self.left_motor.on_for_seconds(SpeedRPS(speed/(2*PI)), seconds)

    def dc_rightMotor(self, duty):
        self.right_motor.duty_cycle_sp=duty
        self.right_motor.run_direc()

    def dc_leftMotor(self, duty):
        #self.left_motor.duty_cycle_sp=duty
        #self.left_motor.run_direc()

    #Métodos motores juntos

    #Métodos motores generales
    def stop(self):

