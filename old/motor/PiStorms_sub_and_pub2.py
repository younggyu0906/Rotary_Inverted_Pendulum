import paho.mqtt.client as mqtt
import time
import datetime
import sys
import threading
from PiStorms import PiStorms

MQTT_SERVER = '10.0.0.1'

MQTT_MOTOR_RESET = 'reset'
MQTT_MOTOR_RECTIFY = 'rectify'
MQTT_MOTOR_POWER = 'motor_power'

MQTT_MOTOR_RESET_COMPLETE = 'reset_complete'
MQTT_MOTOR_RECTIFY_COMPLETE = 'rectify_complete'
MQTT_MOTOR_TO_PENDULUM = 'motor_angle'

sub_topic_list = [MQTT_MOTOR_RESET, MQTT_MOTOR_RECTIFY, MQTT_MOTOR_POWER]
pub_topic_list = [MQTT_MOTOR_RESET_COMPLETE, MQTT_MOTOR_RECTIFY_COMPLETE, MQTT_MOTOR_TO_PENDULUM]

self_motor = None

class Motor:
    def __init__(self, pub):
        global self_motor
        self_motor = self
        self.psm = PiStorms()
        self.speed = 0
        self.pub = pub

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_MOTOR_RESET)
        client.subscribe(topic=MQTT_MOTOR_RECTIFY)
        client.subscribe(topic=MQTT_MOTOR_POWER)

    @staticmethod
    def on_message(client, useradta, msg):
        print(msg.topic)
        if msg.topic == MQTT_MOTOR_RESET:
            if str(msg.payload) == MQTT_MOTOR_RESET:
                self_motor.reset()

        if msg.topic == MQTT_MOTOR_RECTIFY:
            if str(msg.payload) == MQTT_MOTOR_RECTIFY:
                self_motor.rectify()

        if msg.topic == MQTT_MOTOR_POWER:
            self_motor.speed = str(msg.payload)
            self_motor.psm.BAM1.setSpeed(self_motor.speed)
            time.sleep(0.05)
            self_motor.pub.publish(topic=MQTT_MOTOR_TO_PENDULUM, payload=self_motor.get_angle())

    def reset(self):
        isError = True
        # reset position
        self.psm.BAM1.brake()

        while isError:
            try:
                angle = self.psm.BAM1.pos()
                isError = False if angle < 600 and angle > -600 else True
            except TypeError as e:
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

    def get_angle(self):
        isError = True
        while isError:
            try:
                angle = self.psm.BAM1.pos()
                isError = False if angle < 600 and angle > -600 else True
                if angle > 360 or angle < -360 and isError == False:
                    self.psm.BAM1.brake()
            except TypeError as e:
                isError = True
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

