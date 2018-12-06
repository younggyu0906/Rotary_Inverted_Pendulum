import paho.mqtt.client as mqtt
import threading
import spidev
import time
import math
import sys
from datetime import datetime

"""
input_data
    0 = mode 
    1 = padding byte = 0x00
    2 = write mask
    3 = Red LED MSB (0x0000 - 0x03E7, 0 - 999)
    4 = Red LED LSB (0x0000 - 0x03E7, 0 - 999)
    5 = Green LED MSB (0x0000 - 0x03E7, 0 - 999)
    6 = Green LED LSB (0x0000 - 0x03E7, 0 - 999)
    7 = Blue LED MSB (0x0000 - 0x03E7, 0 - 999)
    8 = Blue LED LSB (0x0000 - 0x03E7, 0 - 999)
    9 = Set Encoder 0 (23-16)
    10 = Set Encoder 0 (15-8)
    11 = Set Encoder 0 (7-0)
    12 = Set Encoder 1 (23-16)
    13 = Set Encoder 1 (15-8)
    14 = Set Encoder 1 (7-0)
    15 = Motor Command (15-8)
    16 = Motor Command (7-0)

    * mode = 0x00(read only) || 0x01(transmit all command)
    
    * write mask
        7 = -
        6 = Set Encoder 1 (if 1, set position 0)
        5 = Set Encoder 0 (if 1, set position 0)
        4 = write blue led
        3 = write green led
        2 = write red led
        1 = write motor enable
        0 = write motor
"""

MQTT_SERVER = '10.0.0.1'

MQTT_SUB_FROM_ENV_POWER = 'motor_power'
MQTT_PUB_MOTOR_LIMIT = 'motor_limit_info'
MQTT_PUB_TO_ENV = 'servo_info'

CHANGE_TIME = 0.002
MOTOR_PROTECTION_VOLTAGE = 500

self_servo = None

class QubeServo2:
    def __init__(self):
        global self_servo
        self_servo = self

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.mode = 0b10
        self.spi.max_speed_hz = 1000000

        self.pi = math.pi

        self.pub = mqtt.Client(client_id="servo_pub", transport="TCP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.sub = mqtt.Client(client_id="servo_sub", transport="TCP")
        self.sub.on_connect = self.on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        thread = threading.Thread(target=self.__sub, args=(self.sub,))
        thread.daemon = True
        thread.start()

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        print()
        client.subscribe(topic=MQTT_SUB_FROM_ENV_POWER)

    @staticmethod
    def __sub(sub):
        try:
            print()
            print("***** Sub motor command and Pub started!!! *****")
            sub.loop_forever()
        except KeyboardInterrupt:
            print("Sub thread KeyboardInterrupted")
            self_servo.stop()
            sub.unsubscribe(MQTT_SUB_FROM_ENV_POWER)
            sub.disconnect()

    @staticmethod
    def on_message(client, useradta, msg):
        # print("--->> sub")
        if msg.topic == MQTT_SUB_FROM_ENV_POWER:
            motor_power_info = str(msg.payload.decode("utf-8")).split('|')
            motor_power = int(motor_power_info[0])
            info = motor_power_info[1]
            pub_id = motor_power_info[2]
            if info == "swingup":
                self_servo.motor_command_and_pub(motor_power, info, pub_id)
            elif info == "balance":
                self_servo.motor_command_and_pub(motor_power, info, pub_id)
            elif info == "wait":
                self_servo.wait_and_pub(pub_id)
            elif info == "pendulum_reset":
                self_servo.pendulum_reset(pub_id)

    def data_conversion(self, data):
        # Devoid ID
        device_id = ((data[0] & 0xff) << 8) | (data[1] & 0xff)

        # Motor Encoder Counts
        encoder0 = ((data[2] & 0xff) << 16) | ((data[3] & 0xff) << 8) | (data[4] & 0xff)
        if encoder0 & 0x00800000:
            encoder0 = encoder0 | 0xFF000000
            encoder0 = (0x100000000 - encoder0) * (-1)

        # convert the arm encoder counts to angle theta in radians
        motor_position = encoder0 * (-2.0 * self.pi / 2048.0)

        # Pendulum Encoder Counts
        encoder1 = ((data[5] & 0xff) << 16) | ((data[6] & 0xff) << 8) | (data[7] & 0xff)
        if encoder1 & 0x00800000:
            encoder1 = encoder1 | 0xFF000000
            encoder1 = (0x100000000 - encoder1) * (-1)

        # wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
        encoder1 = encoder1 % 2048
        if encoder1 < 0:
            encoder1 += 2048

        # convert the arm encoder counts to angle theta in radians
        pendulum_angle = encoder1 * (2.0 * self.pi / 2048.0) - self.pi

        return device_id, motor_position, pendulum_angle

    def motor_command_split(self, motor_command):
        # to signed
        if motor_command & 0x0400:
            motor_command = motor_command | 0xfc00

        # add amplifier bit
        motor_command = (motor_command & 0x7fff) | 0x8000

        # separate into 2 bytes
        motor_command_h = (motor_command & 0xff00) >> 8
        motor_command_l = (motor_command & 0xff)
        return motor_command_h, motor_command_l

    # 999 = 0x03e7
    def motor_command_and_pub(self, motor_command, info, pub_id):
        if motor_command > 999 or motor_command < -999:
            print("***** bad motor value!!! *****")

        motor_command_h, motor_command_l = self.motor_command_split(motor_command)

        if info == "swingup":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x00, 0x00, 0x03, 0xe7
        else:
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x03, 0xe7, 0x00, 0x00

        start_data = self.spi.xfer2([
            0x01,
            0x00,
            0x1f,
            red_h, red_l, green_h, green_l, blue_h, blue_l,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])
        _, start_motor_radian, start_pendulum_radian = self.data_conversion(start_data)

        time.sleep(CHANGE_TIME)

        end_data = self.spi.xfer2([
            0x01,
            0x00,
            0x1f,
            red_h, red_l, green_h, green_l, blue_h, blue_l,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])
        _, end_motor_radian, end_pendulum_radian = self.data_conversion(end_data)

        motor_velocity = end_motor_radian - start_motor_radian
        pendulum_velocity = end_pendulum_radian - start_pendulum_radian

        if pendulum_velocity > 2.5:
            pendulum_velocity -= math.pi * 2
        elif pendulum_velocity < -2.5:
            pendulum_velocity += math.pi * 2

        self.limit_check(end_motor_radian)

        self.pub.publish(
            topic=MQTT_PUB_TO_ENV,
            payload="{0}|{1}|{2}|{3}|{4}".format(
                end_motor_radian, motor_velocity, end_pendulum_radian, pendulum_velocity, pub_id),
            qos=0
        )
        # print("<<=== pub step")

    def wait_and_pub(self, pub_id):
        start_data = self.spi.xfer2([
            0x01,
            0x00,
            0x1f,
            0x00, 0x87, 0x00, 0x87, 0x00, 0x87,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        _, start_motor_radian, start_pendulum_radian = self.data_conversion(start_data)

        time.sleep(CHANGE_TIME)

        end_data = self.spi.xfer2([
            0x01,
            0x00,
            0x1f,
            0x00, 0x87, 0x00, 0x87, 0x00, 0x87,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        _, end_motor_radian, end_pendulum_radian = self.data_conversion(end_data)

        motor_velocity = end_motor_radian - start_motor_radian
        pendulum_velocity = end_pendulum_radian - start_pendulum_radian

        if pendulum_velocity > 2.5:
            pendulum_velocity -= math.pi * 2
        elif pendulum_velocity < -2.5:
            pendulum_velocity += math.pi * 2

        self.limit_check(end_motor_radian)

        self.pub.publish(
            topic=MQTT_PUB_TO_ENV,
            payload="{0}|{1}|{2}|{3}|{4}".format(
                end_motor_radian, motor_velocity, end_pendulum_radian, pendulum_velocity, pub_id),
            qos=0
        )
        # print("<<=== pub wait")

    def limit_check(self, motor_radian):
        if abs(motor_radian) > self.pi / 2:
            self.pub.publish(topic=MQTT_PUB_MOTOR_LIMIT, payload="limit_position", qos=0)
            # print("<<=== pub limit position")
            self.protection()
            self.pub.publish(topic=MQTT_PUB_MOTOR_LIMIT, payload="reset_complete", qos=0)
            # print("<<=== pub reset complete")
        else:
            return

    def protection(self):
        is_protect = True
        while is_protect:
            start_data = self.spi.xfer2([
                0x01,
                0x00,
                0x1f,
                0x03, 0xe7, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00
            ])
            _, start_motor_radian, _ = self.data_conversion(start_data)

            time.sleep(CHANGE_TIME)

            motor_command = 80 if start_motor_radian > 0 else -80
            motor_command_h, motor_command_l = self.motor_command_split(motor_command)
            end_data = self.spi.xfer2([
                0x01,
                0x00,
                0x1f,
                0x03, 0xe7, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                motor_command_h, motor_command_l
            ])
            _, end_motor_radian, _ = self.data_conversion(end_data)

            motor_velocity = end_motor_radian - start_motor_radian
            print("limit m_v:", motor_velocity)

            motor_command = 0
            if end_motor_radian > 0:
                if motor_velocity > 0:
                    motor_command = MOTOR_PROTECTION_VOLTAGE
            else:
                if motor_velocity < 0:
                    motor_command = -MOTOR_PROTECTION_VOLTAGE

            motor_command_h, motor_command_l = self.motor_command_split(motor_command)

            protect_data = self.spi.xfer2([
                0x01,
                0x00,
                0x1f,
                0x03, 0xe7, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                motor_command_h, motor_command_l
            ])
            _, protect_motor_radian, _ = self.data_conversion(protect_data)
            time.sleep(CHANGE_TIME)

            if abs(protect_motor_radian) < 8 * self.pi / 18:
                is_protect = False

    def pendulum_reset(self, pub_id):
        self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        time.sleep(CHANGE_TIME)
        data = self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        _, motor_radian, pendulum_radian = self.data_conversion(data)
        self.pub.publish(
            topic=MQTT_PUB_TO_ENV,
            payload="{0}|{1}|{2}|{3}|{4}".format(
                motor_radian, 0, pendulum_radian, 0, pub_id),
            qos=0
        )
        print("***** Pendulum Reset Complete!!! pub_id : {} ***** ".format(pub_id))

    def stop(self):
        self.spi.xfer2([
            0x01,
            0x00,
            0b00011111,
            0x03, 0xe7, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])

if __name__ == "__main__":
    qs = QubeServo2()
    while True:
        try:
            time.sleep(100)
        except KeyboardInterrupt:
            print()
            print("Main thread KeyboardInterrupted")
            qs.stop()
            time.sleep(1)
            break
