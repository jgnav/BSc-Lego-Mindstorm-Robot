#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.power import PowerSupply
from math import pi, sin, cos, atan
from threading import Thread
import numpy as np
from time import sleep

class sensors_and_battery:
    def __init__(self, sonar, color_sensor):
        self._sonar = UltrasonicSensor(sonar)
        self._color = ColorSensor(color_sensor)
        self._battery = PowerSupply()

    #Battery
    @property
    def battery_voltage(self):
        return self._battery.measured_volts

    @property
    def battery_current(self):
        return self._battery.measured_amps

    #Sonar sensor
    @property
    def distance_sonar(self):
        return (self._sonar.distance_centimeters / 100)

    @property
    def other_sensor_present(self):
        return self._sonar.other_sensor_present

    #Color sensor
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

class movement:
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_distance):
        self._left_motor = Motor(left_motor)
        self._right_motor = Motor(right_motor)
        self._both_motors = MoveTank(left_motor, right_motor)
        self._r = wheel_diameter
        self._d = axle_distance

    def _SpeedRadPS(self, value):
        return SpeedRPS(value/(2*pi))

    #Separate motor
    @property
    def rightMotor_speed(self):
        return 2*pi*(self._right_motor.speed/self._right_motor.count_per_rot)

    @rightMotor_speed.setter
    def rightMotor_speed(self, speed):
        self._right_motor.on(self._SpeedRadPS(speed))

    @property
    def leftMotor_speed(self):
        return 2*pi*(self._left_motor.speed/self._left_motor.count_per_rot)

    @leftMotor_speed.setter
    def leftMotor_speed(self, speed):
        self._left_motor.on(self._SpeedRadPS(speed))

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

    #Both motors
    def run(self, linear, angular):
        self._right = ((linear)+((angular*self._d)/2))/self._r
        self._left = ((linear)-((angular*self._d)/2))/self._r
        self._both_motors.on(self._SpeedRadPS(self._left), self._SpeedRadPS(self._right))

    def run_time(self, linear, angular, seconds):
        self._right = ((linear)+((angular*self._d)/2))/self._r
        self._left = ((linear)-((angular*self._d)/2))/self._r
        self._both_motors.on(self._SpeedRadPS(self._left), self._SpeedRadPS(self._right), seconds)

    def stop(self):
        self._both_motors.off()

    @property
    def linear_speed(self):
        return ((self.rightMotor_speed+self.leftMotor_speed)/2)*self._r

    @property
    def angular_speed(self):
        return ((self.rightMotor_speed-self.leftMotor_speed)*self._r)/self._d


class odometry(movement):
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_distance):
        movement.__init__(self, left_motor, right_motor, wheel_diameter, axle_distance)

        self._wheel_circunference = pi * self._r

        self._odometry_thread_id = None
        self._odometry_thread_run = False
        self._sleep_time = 0.01

        self._universal_zero = np.array([0, 0, 0])
        self._universal_axis = np.array([[1, 0, 0],
                                         [0, 1, 0],
                                         [0, 0, 1]])

        self._robot_zero = np.array([0, 0, 0])
        self._robot_axis = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])

    def _axis_to_theta(self, axis):
        if ((axis[0][1] > 0) and (axis[1][1] >= 0)):

            theta = atan(axis[1][1]/axis[0][1])

        elif ((axis[0][1] < 0) and (axis[1][1] > 0)):

            theta = atan(axis[1][1]/axis[0][1]) + pi

        elif ((axis[0][1] < 0) and (axis[1][1] <= 0)):

            theta = atan(axis[1][1]/axis[0][1]) + pi

        elif ((axis[0][1] > 0) and (axis[1][1] < 0)):

            theta = 2*pi + atan(axis[1][1]/axis[0][1])

        elif ((axis[0][1] == 0) and (axis[1][1] > 0)):

            theta = pi/2

        elif ((axis[0][1] == 0) and (axis[1][1] < 0)):

            theta = (3/2)*pi

        return theta

    def _theta_to_axis(self, theta):
        axis = np.array([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])

        if ((theta > 0) and (theta < pi/2)):

            axis[0][0] = 1
            axis[0][1] = 1
            axis[1][0] = 1
            axis[1][1] = 1

        elif ((theta > pi/2) and (theta < pi)):

            axis[0][0] = 1
            axis[0][1] = 1
            axis[1][0] = 1
            axis[1][1] = 1

        elif ((theta > pi) and (theta < (3/2)*pi)):

            axis[0][0] = 1
            axis[0][1] = 1
            axis[1][0] = 1
            axis[1][1] = 1

        elif ((theta > (3/2)*pi) and (theta < 2*pi)):

            axis[0][0] = 1
            axis[0][1] = 1
            axis[1][0] = 1
            axis[1][1] = 1

        return axis

    def odometry_start(self, zero, axis):
        self._robot_zero = zero
        self._robot_axis = axis

        def _odometry_thread():
            left_previous = 0
            right_previous = 0
            theta  = self._axis_to_theta(self._robot_axis)

            while self._odometry_thread_run:

                left_current = self._left_motor.position
                right_current = self._right_motor.position

                left_ticks = left_current - left_previous
                right_ticks = right_current - right_previous

                if not left_ticks and not right_ticks:
                    if self._sleep_time:
                        sleep(self._sleep_time)
                    continue

                left_previous = left_current
                right_previous = right_current

                left_rotations = float(left_ticks / self._left_motor.count_per_rot)
                right_rotations = float(right_ticks / self._right_motor.count_per_rot)

                left_distance = float(left_rotations * self._wheel_circunference)
                right_distance = float(right_rotations * self._wheel_circunference)

                total_distance = (left_distance + right_distance) / 2.0

                theta += (right_distance - left_distance) / self._d

                self._robot_zero[0] += total_distance * cos(theta)
                self._robot_zero[1] += total_distance * sin(theta)
                self._robot_axis = self._theta_to_axis(theta)

                if self._sleep_time:
                    sleep(self._sleep_time)

            self.odometry_thread_id = None

        self._odometry_thread_id = Thread(target = _odometry_thread)
        self._odometry_thread_id.start()

    def odometry_stop(self):
        if self._odometry_thread_id:
            self._odometry_thread_run = False

            while self._odometry_thread_id:
                pass

