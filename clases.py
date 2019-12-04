#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.power import PowerSupply
from math import pi

class robot:
    #Inicialización
    def __init__(self , left_motor, right_motor, sonar, color_sensor, wheel_diameter, axle_distance):
        self._left_motor = Motor(left_motor)
        self._right_motor = Motor(right_motor)
        self._both_motors = MoveTank(left_motor, right_motor)
        self._sonar = UltrasonicSensor(sonar)
        self._color = ColorSensor(color_sensor)
        self._battery = PowerSupply()
        self._r = wheel_diameter
        self._d = axle_distance

    def __SpeedRadPS(self, value):
        return SpeedRPS(value/(2*pi))

    #Motores separados
    @property
    def rightMotor_speed(self):
        return 2*pi*(self._right_motor.speed/self._right_motor.count_per_rot)

    @rightMotor_speed.setter
    def rightMotor_speed(self, speed):
        self._right_motor.on(self.__SpeedRadPS(speed))

    @property
    def leftMotor_speed(self):
        return 2*pi*(self._left_motor.speed/self._left_motor.count_per_rot)

    @leftMotor_speed.setter
    def leftMotor_speed(self, speed):
        self._left_motor.on(self.__SpeedRadPS(speed))

    @property
    def rightMotor_dc(self):
        return self._right_motor.duty_cycle

    @rightMotor_dc.setter
    def rightMotor_dc(self, duty):
        self._right_motor.run_direct(duty_cycle_sp = duty)

    @property
    def leftMotor_dc(self):
        return self._left_motor.duty_cycle

    @leftMotor_dc.setter
    def leftMotor_dc(self, duty):
        self._left_motor.run_direct(duty_cycle_sp = duty)

    #Motores juntos
    def run(self, linear, angular):
        self._right = ((linear)+((angular*self._d)/2))/self._r
        self._left = ((linear)-((angular*self._d)/2))/self._r
        self._both_motors.on(self.__SpeedRadPS(self._left), self.__SpeedRadPS(self._right))

    def run_time(self, linear, angular, seconds):
        self._right = ((linear)+((angular*self._d)/2))/self._r
        self._left = ((linear)-((angular*self._d)/2))/self._r
        self._both_motors.on(self.__SpeedRadPS(self._left), self.__SpeedRadPS(self._right), seconds)

    def stop(self):
        self._both_motors.off()

    @property
    def linear_speed(self):
        return ((self.rightMotor_speed+self.leftMotor_speed)/2)*self._r

    @property
    def angular_speed(self):
        return ((self.rightMotor_speed-self.leftMotor_speed)*self._r)/self._d

    #Batería
    @property
    def battery_voltage(self):
        return self._battery.measured_volts

    @property
    def battery_current(self):
        return self._battery.measured_amps

    #Sensor sonar
    @property
    def distance_sonar(self):
        return (self._sonar.distance_centimeters / 100)

    @property
    def other_sensor_present(self):
        return self._sonar.other_sensor_present

    #Sensor de color
    def calibrate_white(self):
        self._color.calibrate_white()

    @property
    def color(self):
        return self._color.color

    @property
    def color_name(self):
        return self._color.color_name

    @property
    def ambient(self):
        return self._color.ambient_light_intensity

    @property
    def reflection(self):
        return self._color.reflected_light_intensity

    @property
    def rgb(self):
        return self._color.rgb


