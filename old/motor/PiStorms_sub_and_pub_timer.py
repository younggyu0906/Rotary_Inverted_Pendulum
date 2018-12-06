import paho.mqtt.client as mqtt
from datetime import datetime
import threading
from PiStorms import PiStorms
import RPi.GPIO as gpio
import time

led_pin = 21
gpio.setmode(gpio.BCM)
gpio.setup(led_pin, gpio.OUT)

MQTT_SERVER = '10.0.0.1'

MQTT_MOTOR_RESET = 'motor_reset_position'
MQTT_MOTOR_POWER = 'motor_power'

MQTT_MOTOR_RESET_COMPLETE = 'reset_complete'
MQTT_MOTOR_STATE = 'motor_state_info'

sub_topic_list = [MQTT_MOTOR_RESET, MQTT_MOTOR_POWER]
pub_topic_list = [MQTT_MOTOR_RESET_COMPLETE, MQTT_MOTOR_STATE]

self_motor = None

class Motor:
    def __init__(self):
        global self_motor
        self_motor = self

        self.psm = PiStorms()
        self.angle = 0
        self.isReady = False

        self.pub = mqtt.Client(client_id="motor_pub", transport="TCP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.sub = mqtt.Client(client_id="motor_sub", transport="TCP")
        self.sub.on_connect = self.on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        thread = threading.Thread(target=self.__sub, args=(self.sub,))
        thread.daemon = True
        thread.start()

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_MOTOR_RESET)
        client.subscribe(topic=MQTT_MOTOR_POWER)

    @staticmethod
    def __sub(sub):
        try:
            print("***** sub motor power started! *****")
            sub.loop_forever()
        except KeyboardInterrupt as e:
            self_motor.psm.BAM1.brake()
            print("KeyboardInterrupted: {0}".format(e))
            sub.unsubscribe(sub_topic_list)
            sub.disconnect()

    @staticmethod
    def on_message(client, useradta, msg):
        if msg.topic == MQTT_MOTOR_RESET:
            print("***** reset started! *****")
            self_motor.reset()
            print("***** reset complete! *****")

        if msg.topic == MQTT_MOTOR_POWER:
            print("----- setSpeed {0} -----".format(str(msg.payload)))
            self_motor.psm.BAM1.setSpeed(str(msg.payload))

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
                time.sleep(0.0001)

        self.psm.BAM1.runDegs(-angle, 100, True, True)
        time.sleep(10)
        self.pub.publish(topic=MQTT_MOTOR_RESET_COMPLETE, payload=MQTT_MOTOR_RESET_COMPLETE)

    def timer_thread(self):
        print("***** pub motor angle started! (30 ms) *****")
        while True:
            self.isReady = True
            # time.sleep(0.015)
            time.sleep(0.03)

    def update_and_pub_thread(self):
        while True:
            if self.isReady:
                self.isReady = False
                isError = True
                while isError:
                    try:
                        self.angle = self.psm.BAM1.pos()
                        isError = False
                        # if self.angle < 5000 and self.angle > -5000:
                        #     isError = True
                    except TypeError as e:
                        print("get angle error: {0}".format(str(e)))
                        isError = True
                        time.sleep(0.0001)
                self.pub.publish(
                    topic=MQTT_MOTOR_STATE,
                    payload=str(self.angle) + '|' + datetime.utcnow().strftime('%S.%f')[:-3]
                )
            else:
                time.sleep(0.0001)


if __name__ == "__main__":
    print

    motor = Motor()

    timer_thread = threading.Thread(target=motor.timer_thread)
    update_and_pub_thread = threading.Thread(target=motor.update_and_pub_thread)

    timer_thread.daemon = True
    update_and_pub_thread.daemon = True

    timer_thread.start()
    update_and_pub_thread.start()

    while True:
        time.sleep(1)
