import paho.mqtt.client as mqtt
import time
import datetime
import sys
import threading
from PiStorms import PiStorms
import numpy as np
import RPi.GPIO as gpio
import time
import math

led_pin = 21
gpio.setmode(gpio.BCM)
gpio.setup(led_pin, gpio.OUT)

MQTT_SERVER = '10.0.0.1'

MQTT_MOTOR_RESET = 'reset'
MQTT_MOTOR_RECTIFY = 'rectify'
MQTT_MOTOR_POWER = 'motor_power'
MQTT_PENDULUM_STATE_FOR_RECTIFY = 'pendulum_state_for_rectify'

MQTT_MOTOR_RESET_COMPLETE = 'reset_complete'
MQTT_MOTOR_RECTIFY_COMPLETE = 'rectify_complete'
MQTT_MOTOR_TO_PENDULUM = 'motor_angle'
MQTT_MOTOR_STILL_RECTIFY = 'motor_still_rectify'

sub_topic_list = [MQTT_MOTOR_RESET, MQTT_MOTOR_RECTIFY, MQTT_MOTOR_POWER, MQTT_PENDULUM_STATE_FOR_RECTIFY]
pub_topic_list = [MQTT_MOTOR_RESET_COMPLETE, MQTT_MOTOR_RECTIFY_COMPLETE, MQTT_MOTOR_TO_PENDULUM, MQTT_MOTOR_STILL_RECTIFY]

self_motor = None

PI = 3.14
MOTOR_FREQ = 30.0

class Motor:
    def __init__(self, pub):
        global self_motor
        self_motor = self
        self.psm = PiStorms()
        self.speed = 0
        self.pub = pub
        self.pendulum_state = None
        self.time = 0.0
        self.led_flag = True

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_MOTOR_RESET)
        client.subscribe(topic=MQTT_MOTOR_RECTIFY)
        client.subscribe(topic=MQTT_MOTOR_POWER)
        client.subscribe(topic=MQTT_PENDULUM_STATE_FOR_RECTIFY)

    @staticmethod
    def on_message(client, useradta, msg):
        print(msg.topic)
        if msg.topic == MQTT_MOTOR_RESET:
            if str(msg.payload) == MQTT_MOTOR_RESET:
                self_motor.reset()

        # if msg.topic == MQTT_MOTOR_RECTIFY:
        #     if str(msg.payload) == MQTT_MOTOR_RECTIFY:
        #         self_motor.rectify()

        if msg.topic == MQTT_MOTOR_POWER:
            # gpio.output(led_pin, self_motor.led_flag)
            # if self_motor.led_flag:
            #     self_motor.led_flag=False
            # else:
            #     self_motor.led_flag=True
            self_motor.speed = str(msg.payload)
            self_motor.psm.BAM1.setSpeed(self_motor.speed)
            # time.sleep(0.05)
            self_motor.pub.publish(topic=MQTT_MOTOR_TO_PENDULUM, payload=self_motor.get_angle())

        if msg.topic == MQTT_PENDULUM_STATE_FOR_RECTIFY:
            # gpio.output(led_pin, self_motor.led_flag)
            # if self_motor.led_flag:
            #     self_motor.led_flag = False
            # else:
            #     self_motor.led_flag = True
            pendulum_state = str(msg.payload.decode("utf-8")).split('|')
            pendulum_angle = int(pendulum_state[0])
            pendulum_speed = int(pendulum_state[1])
            self_motor.rectify2(pendulum_angle, pendulum_speed)

    def reset(self):
        isError = True
        # reset position
        self.psm.BAM1.brake()
        while isError:
            try:
                angle = self.psm.BAM1.pos()
                isError = False
            except TypeError as e:
                print("error: {0}".format(str(e)))
                isError = True
                time.sleep(0.001)

        self.psm.BAM1.runDegs(-angle, 30, True, True)
        time.sleep(3)
        self.pub.publish(topic=MQTT_MOTOR_RESET_COMPLETE, payload=MQTT_MOTOR_RESET_COMPLETE)

    def rectify(self):
        self.psm.BAM1.brake()
        cnt_sleep = 0
        while cnt_sleep < 1:
            print ".",
            sys.stdout.flush()
            time.sleep(1)
            cnt_sleep += 1
        print()

        self.psm.BAM1.setSpeed(100)
        time.sleep(0.3)

        self.psm.BAM1.setSpeed(0)
        time.sleep(0.65)

        self.psm.BAM1.setSpeed(100)
        time.sleep(0.22)

        self.psm.BAM1.setSpeed(-100)
        time.sleep(0.19)

        # self.psm.BAM1.setSpeed(0)
        # time.sleep(0.05)
        self.pub.publish(topic=MQTT_MOTOR_RECTIFY_COMPLETE, payload=self.get_angle())

    def rectify2(self, angle, speed):
        pen_angle = angle
        pen_speed = speed
        print(pen_angle, pen_speed)
        if pen_angle <= 10 and pen_angle >= -10:
            self.psm.BAM1.hold()
            if pen_speed < 0:
                self.psm.BAM1.setSpeed(100)
            else:
                self.psm.BAM1.setSpeed(-100)
            time.sleep(0.02)
            self.pub.publish(topic=MQTT_MOTOR_RECTIFY_COMPLETE, payload=self.get_angle())
            print("successfully rectified")
            return
        abs_pen_angle = abs(pen_angle)
        if abs_pen_angle >= 140:
            if pen_speed < 0:
                self.psm.BAM1.setSpeed(-40)
                time.sleep(0.1)
            else:
                self.psm.BAM1.setSpeed(40)
                time.sleep(0.1)
        elif abs_pen_angle < 140 and abs_pen_angle >= 110:
            if pen_speed < 0:
                self.psm.BAM1.setSpeed(-30)
                time.sleep(0.1)
            else:
                self.psm.BAM1.setSpeed(30)
                time.sleep(0.1)
        elif abs_pen_angle < 110 and abs_pen_angle >= 90:
            if pen_speed < 0:
                self.psm.BAM1.setSpeed(-10)
                time.sleep(0.1)
            else:
                self.psm.BAM1.setSpeed(10)
                time.sleep(0.1)
        # else:
        #     if pen_speed < 0:
        #         self.psm.BAM1.setSpeed(-15)
        #         time.sleep(0.1)
        #     else:
        #         self.psm.BAM1.setSpeed(15)
        #         time.sleep(0.1)
        # abs_pen_angle = abs(pen_angle)
        #
        # print("Fire Pub -", abs_pen_angle)

        ####################TESET#########################################################
        # gpio.output(led_pin, self_motor.led_flag)
        # if self_motor.led_flag:
        #     self.led_flag=False
        # else:
        #     self.led_flag=True
        # # self.time += 0.01
        # # setting_speed = 30.0*math.sin(2 * PI * MOTOR_FREQ * self.time)
        # abs_pen_angle = abs(pen_angle)
        #
        # if self.led_flag:
        #     self.psm.BAM1.setSpeed(100)
        # else:
        #     self.psm.BAM1.setSpeed(-100)

        # if pen_angle <= 10 and pen_angle >= -10:
        #     self.psm.BAM1.hold()
        #     self.pub.publish(topic=MQTT_MOTOR_RECTIFY_COMPLETE, payload=self.get_angle())
        #     print("successfully rectified")
        #     return
        # else:
        #     # if pen_speed < 0:
        #     self.psm.BAM1.setSpeed(setting_speed)
        #     # else:
        #     #     self.psm.BAM1.setSpeed(setting_speed)
        ###################################################################################

        self.pub.publish(topic=MQTT_MOTOR_STILL_RECTIFY, payload=MQTT_MOTOR_STILL_RECTIFY)

    def get_angle(self):
        isError = True
        while isError:
            try:
                angle = self.psm.BAM1.pos()
                isError = False if angle < 1000 and angle > -1000 else True
            except TypeError as e:
                isError = True
                time.sleep(0.001)
        return angle


if __name__ == "__main__":
    pub = mqtt.Client(client_id="motor_pub", transport="TCP")
    pub.connect(MQTT_SERVER, 1883, 60)

    motor = Motor(pub)

    sub = mqtt.Client(client_id="motor_sub", transport="TCP")
    sub.on_connect = motor.on_connect
    sub.on_message = motor.on_message
    sub.connect(MQTT_SERVER, 1883, 60)

    try:
        print("Sub thread started!")
        sub.loop_forever()
    except KeyboardInterrupt:
        print("Sub Interrupted!")
        sub.unsubscribe(sub_topic_list)
        sub.disconnect()
        motor.psm.BAM1.brake()

