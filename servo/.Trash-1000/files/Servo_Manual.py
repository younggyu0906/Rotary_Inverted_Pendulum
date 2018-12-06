import paho.mqtt.client as mqtt
import numpy as np
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

# ********** SUB **********
MQTT_SUB_FROM_ENV_RESET = 'reset'
MQTT_SUB_FROM_ENV_MOTOR_POWER = 'motor_power'

# ********** PUB **********
MQTT_PUB_TO_ENV_RESET_COMPLETE = 'reset_complete'
MQTT_PUB_TO_ENV_INFO = 'info'

SAMPLE_TIME = 1 / 1000
self_servo = None


class QubeServo2:
    def __init__(self):
        global self_servo
        self_servo = self

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.mode = 0b10
        self.spi.max_speed_hz = 1000000

        self.pub = mqtt.Client(client_id="servo_pub", transport="TCP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.sub = mqtt.Client(client_id="servo_sub", transport="TCP")
        self.sub.on_connect = self.__on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        thread = threading.Thread(target=self.__sub, args=(self.sub,))
        thread.daemon = True
        thread.start()

    @staticmethod
    def __on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc) + "\n")
        client.subscribe(topic=MQTT_SUB_FROM_ENV_MOTOR_POWER)
        client.subscribe(topic=MQTT_SUB_FROM_ENV_RESET)

    def __sub(self, sub):
        try:
            print("\n***** Sub thread started!!! *****")
            sub.loop_forever()
        except KeyboardInterrupt:
            print("Sub thread KeyboardInterrupted")
            # LED Red and Motor Stop
            self.send_and_receive(0x03, 0xe7, 0x00, 0x00, 0x00, 0x00, 0)
            sub.unsubscribe(MQTT_SUB_FROM_ENV_MOTOR_POWER)
            sub.unsubscribe(MQTT_SUB_FROM_ENV_RESET)
            sub.disconnect()

    def on_message(self, client, useradta, msg):
        if msg.topic == MQTT_SUB_FROM_ENV_RESET:
            time.sleep(15)

            self.reset()

            self.swing_up_by_manual()

            self.balance_control_by_manual()

            # LED Purple
            _, motor_radian, _, pendulum_radian = self.protection(0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7, 0)

            self.pub.publish(
                topic=MQTT_PUB_TO_ENV_RESET_COMPLETE,
                payload="{0}|{1}".format(motor_radian, pendulum_radian)
            )

        if msg.topic == MQTT_SUB_FROM_ENV_MOTOR_POWER:
            info = str(msg.payload.decode("utf-8")).split('|')
            motor_power = int(info[0])

            # LED Green + Blue
            previous_motor_angle, _, _, _ = self.send_and_receive(0x00, 0x00, 0x03, 0xe7, 0x03, 0xe7, motor_power)

            time.sleep(SAMPLE_TIME)

            # LED Green + Blue
            _, motor_radian, _, pendulum_radian = self.protection(0x00, 0x00, 0x03, 0xe7, 0x03, 0xe7, 0)

            self.pub.publish(
                topic=MQTT_PUB_TO_ENV_INFO,
                payload="{0}|{1}|{2}".format(motor_radian, pendulum_radian, previous_motor_angle)
            )

    def reset(self):
        self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            # LED Purple
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        data = self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            # LED Purple
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        time.sleep(1)
        motor_angle, motor_radian, pendulum_angle, pendulum_radian = self.data_conversion(data)
        print("\n***** Reset Complete --> || motor_angle:{0:3.1f} || pendulum_angle:{1:3.1f} || *****"
              .format(motor_angle, pendulum_angle))

    def send_and_receive(self, red_h, red_l, green_h, green_l, blue_h, blue_l, motor_command):
        # 2's complement calculate
        if motor_command & 0x0400:
            motor_command = motor_command | 0xfc00

        # add amplifier bit
        motor_command = (motor_command & 0x7fff) | 0x8000

        # separate into 2 bytes
        motor_command_h = (motor_command & 0xff00) >> 8
        motor_command_l = (motor_command & 0xff)

        data = self.spi.xfer2([
            0x01,
            0x00,
            0b00011111,
            red_h, red_l, green_h, green_l, blue_h, blue_l,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])

        motor_angle, motor_radian, pendulum_angle, pendulum_radian = self.data_conversion(data)

        self.limit_check(motor_angle)

        return motor_angle, motor_radian, pendulum_angle, pendulum_radian

        # self.pub.publish(
        #     topic=MQTT_PUB_TO_ENV_INFO,
        #     payload="{0}|{1}".format(motor_position, pendulum_angle),
        # )

    @staticmethod
    def data_conversion(data):
        # Motor Encoder Counts
        encoder0 = ((data[2] & 0xff) << 16) | ((data[3] & 0xff) << 8) | (data[4] & 0xff)
        if encoder0 & 0x00800000:
            encoder0 = encoder0 | 0xFF000000
            # 2's complement calculate
            encoder0 = (0x100000000 - encoder0) * (-1)

        # convert the arm encoder counts to angle theta in radians
        motor_radian = encoder0 * (-2.0 * math.pi / 2048.0)
        motor_angle = motor_radian * 57.295779513082320876798154814105

        # Pendulum Encoder Counts
        encoder1 = ((data[5] & 0xff) << 16) | ((data[6] & 0xff) << 8) | (data[7] & 0xff)
        if encoder1 & 0x00800000:
            encoder1 = encoder1 | 0xFF000000
            # 2's complement calculate
            encoder1 = (0x100000000 - encoder1) * (-1)

        # wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
        encoder1 = encoder1 % 2048
        if encoder1 < 0:
            encoder1 += 2048

        # convert the arm encoder counts to angle theta in radians
        pendulum_radian = encoder1 * (2.0 * math.pi / 2048.0) - math.pi
        pendulum_angle = pendulum_radian * 57.295779513082320876798154814105

        return motor_angle, motor_radian, pendulum_angle, pendulum_radian

    def limit_check(self, motor_angle):
        if abs(motor_angle) > 180:
            # State of Emergency, LED Red
            self.protection(0x03, 0xe7, 0x00, 0x00, 0x00, 0x00, 0)
            print("\n***** State of Emergency --> Motor Angle: {0:3.1f} *****".format(
                motor_angle
            ))
            sys.exit()
        elif abs(motor_angle) > 110:
            # LEB Red
            if motor_angle > 0:
                self.protection(0x03, 0xe7, 0x00, 0x00, 0x00, 0x00, 100)
            else:
                self.protection(0x03, 0xe7, 0x00, 0x00, 0x00, 0x00, -100)
        elif abs(motor_angle) > 90:
            # LEB Orange
            if motor_angle > 0:
                self.protection(0x03, 0xe7, 0x01, 0xf4, 0x00, 0x00, 50)
            else:
                self.protection(0x03, 0xe7, 0x01, 0xf4, 0x00, 0x00, -50)

    def protection(self, red_h, red_l, green_h, green_l, blue_h, blue_l, motor_command):
        # 2's complement calculate
        if motor_command & 0x0400:
            motor_command = motor_command | 0xfc00

        # add amplifier bit
        motor_command = (motor_command & 0x7fff) | 0x8000

        # separate into 2 bytes
        motor_command_h = (motor_command & 0xff00) >> 8
        motor_command_l = (motor_command & 0xff)

        data = self.spi.xfer2([
            0x01,
            0x00,
            0b00011111,
            red_h, red_l, green_h, green_l, blue_h, blue_l,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])

        return self.data_conversion(data)

    def swing_up_by_manual(self):
        print("\n***** Manual Swing Up Start!!! *****")

        previousTime = time.perf_counter()
        last_pendulum_radian = 0
        motorPWM = 0

        while True:
            # if the difference between the current time and the last time an SPI transaction
            # occurred is greater than the sample time, start a new SPI transaction
            currentTime = time.perf_counter()
            if currentTime - previousTime >= SAMPLE_TIME:
                # print("|| Time difference: {0} s ||".format(currentTime - previousTime))

                previousTime = currentTime

                # LED Blue
                motor_angle, motor_radian, pendulum_angle, pendulum_radian = \
                    self.send_and_receive(0x00, 0x00, 0x00, 0x00, 0x03, 0xe7, 0)

                if abs(pendulum_angle) < 30:
                    break

                angular_variation = (pendulum_radian - last_pendulum_radian)
                # angular variation filtering
                if angular_variation > 2.5:
                    angular_variation -= math.pi * 2
                elif angular_variation < -2.5:
                    angular_variation += math.pi * 2

                pendulum_angular_velocity = angular_variation / SAMPLE_TIME

                last_pendulum_radian = pendulum_radian

                voltage = 46
                if abs(pendulum_angular_velocity) > 28:
                    voltage /= int(10 * np.log(abs(pendulum_angular_velocity)))

                if pendulum_radian >= 0:
                    pendulum_radian = math.pi - pendulum_radian
                else:
                    pendulum_radian = - math.pi + abs(pendulum_radian)

                # if abs(pendulum_angle) > 90:
                if pendulum_angular_velocity < 0:
                    motorPWM = int(-2 * math.cos(pendulum_radian) * voltage)
                else:
                    motorPWM = int(2 * math.cos(pendulum_radian) * voltage)

                # LED Blue
                self.send_and_receive(0x00, 0x00, 0x00, 0x00, 0x03, 0xe7, int(motorPWM))

        print("\n***** Manual Swing Up complete!!! *****")

    def balance_control_by_manual(self):
        print("\n***** Manual Balance Control Start!!! *****")

        theta_n_k1 = 0.0
        theta_dot_k1 = 0.0
        alpha_n_k1 = 0.0
        alpha_dot_k1 = 0.0

        kp_theta = 2.0
        kd_theta = -2.0
        kp_alpha = -30.0
        kd_alpha = 2.5

        previousTime = time.perf_counter()

        count = 0

        while count < 3000:
            # if the difference between the current time and the last time an SPI transaction
            # occurred is greater than the sample time, start a new SPI transaction
            currentTime = time.perf_counter()
            if currentTime - previousTime >= SAMPLE_TIME:
                # print("|| Time difference: {0} s ||".format(currentTime - previousTime))

                previousTime = currentTime

                # LED Green
                _, theta, _, alpha = self.send_and_receive(0x00, 0x00, 0x03, 0xe7, 0x00, 0x00, 0)

                # if the pendulum is within +/-30 degrees of upright, enable balance control
                if abs(alpha) <= (30.0 * math.pi / 180.0):
                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    theta_n = -theta
                    theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.9512 * theta_dot_k1)
                    theta_n_k1 = theta_n
                    theta_dot_k1 = theta_dot

                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    alpha_n = -alpha
                    alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.9512 * alpha_dot_k1)
                    alpha_n_k1 = alpha_n
                    alpha_dot_k1 = alpha_dot

                    # multiply by proportional and derivative gains
                    motorVoltage = (theta * kp_theta) + (theta_dot * kd_theta) + (alpha * kp_alpha) + (
                                alpha_dot * kd_alpha)

                    # set the saturation limit to +/- 15V
                    if motorVoltage > 15.0:
                        motorVoltage = 15.0
                    elif motorVoltage < -15.0:
                        motorVoltage = -15.0

                    # invert for positive CCW
                    motorVoltage = -motorVoltage

                    # convert the analog value to the PWM duty cycle that will produce the same average voltage
                    motorPWM = motorVoltage * (625.0 / 15.0)

                    motorPWM = int(motorPWM)

                    # if abs(motorPWM) > 100:
                    #     print("|| Time: {0} || Theta: {1:.5f} || "
                    #           "Alpha: {2:.5f} || motorPWM: {3} ||".format(
                    #             datetime.utcnow().strftime('%S.%f')[:-3], theta, alpha, motorPWM
                    #             ))

                    # LED Green
                    self.send_and_receive(0x00, 0x00, 0x03, 0xe7, 0x00, 0x00, int(motorPWM))

                    count += 1

                else:
                    # Turn on the red and green LEDs for Orange
                    self.send_and_receive(0x03, 0xe7, 0x01, 0xf4, 0x00, 0x00, 0)
                    break
        print("\n***** Manual Balance Control Finished!!! *****")


if __name__ == "__main__":
    servo = QubeServo2()

    while True:
        try:
            time.sleep(100)
        except KeyboardInterrupt:
            print("\nMain thread KeyboardInterrupted")
            # LED Red
            servo.protection(0x03, 0xe7, 0x00, 0x00, 0x00, 0x00, 0)
            sys.exit()
