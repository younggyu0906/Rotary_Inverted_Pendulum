import paho.mqtt.client as mqtt
from datetime import datetime
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
is_ready = False
motor_power = 0
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
            self.send(0, 'Red')
            sub.unsubscribe(MQTT_SUB_FROM_ENV_MOTOR_POWER)
            sub.unsubscribe(MQTT_SUB_FROM_ENV_RESET)
            sub.disconnect()

    def on_message(self, client, useradta, msg):
        global is_ready, motor_power

        if msg.topic == MQTT_SUB_FROM_ENV_RESET:
            self.send(0, 'Yellow')

            # for reset
            time.sleep(15)

            # LED Magenta
            self.reset()

            # LED Cyan
            self.swing_up_by_manual()

            # LED Blue
            self.balance_control_by_manual()

            _, motor_radian, _, pendulum_radian = self.receive()

            self.pub.publish(
                topic=MQTT_PUB_TO_ENV_RESET_COMPLETE,
                payload="{0}|{1}".format(motor_radian, pendulum_radian)
            )

            is_ready = True

        if msg.topic == MQTT_SUB_FROM_ENV_MOTOR_POWER:
            info = str(msg.payload.decode("utf-8")).split('|')
            if info[0] == "done":
                is_ready = False
                motor_power = 0
                print("\n*--*--*--*--* episode is done!!! *--*--*--*--*")
                print("--------------------------------\n")
                self.send(0, 'Orange')
            else:
                motor_power = int(info[0])
                # print("*--*--* Set Motor power: {0} || time: {1} *--*--*".format(
                #     motor_power, datetime.utcnow().strftime('%S.%f')[:-1]
                # ))

    def reset(self):
        self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            # LED Magenta
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        data = self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            # LED Magenta
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        motor_angle, motor_radian, pendulum_angle, pendulum_radian = self.data_conversion(data)
        print("\n\n*--*--*--*--* Pendulum Reset Complete --> || motor_angle:{0:3.1f} || "
              "pendulum_angle:{1:3.1f} || *--*--*--*--*\n"
              .format(motor_angle, pendulum_angle))

    @staticmethod
    def motor_command_calculation(motor_command):
        # 2's complement calculation
        if motor_command & 0x0400:
            motor_command = motor_command | 0xfc00

        # add amplifier bit
        motor_command = (motor_command & 0x7fff) | 0x8000

        # separate into 2 bytes
        motor_command_h = (motor_command & 0xff00) >> 8
        motor_command_l = (motor_command & 0xff)

        return motor_command_h, motor_command_l

    @staticmethod
    def color_command_calculation(color_command):
        if color_command == 'Red':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x00, 0x00, 0x00, 0x00
        elif color_command == 'Green':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x03, 0xe7, 0x00, 0x00
        elif color_command == 'Blue':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x00, 0x00, 0x03, 0xe7
        elif color_command == 'Cyan':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x03, 0xe7, 0x03, 0xe7
        elif color_command == 'Magenta':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7
        elif color_command == 'Yellow':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x03, 0xe7, 0x00, 0x00
        elif color_command == 'Orange':
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x01, 0xf4, 0x00, 0x00

        return red_h, red_l, green_h, green_l, blue_h, blue_l

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

    def send_and_receive(self, motor_command, color_command):
        motor_command_h, motor_command_l = self.motor_command_calculation(motor_command)
        red_h, red_l, green_h, green_l, blue_h, blue_l = self.color_command_calculation(color_command)

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

    def limit_check(self, motor_angle):
        if abs(motor_angle) > 180:
            # State of Emergency
            self.send(0, 'Red')
            print("\n***** State of Emergency --> Motor Angle: {0:3.1f} *****".format(motor_angle))
            sys.exit()
        elif abs(motor_angle) > 110:
            if motor_angle > 0:
                self.send(100, 'Red')
            else:
                self.send(-100, 'Red')
        elif abs(motor_angle) > 90:
            if motor_angle > 0:
                self.send(50, 'Orange')
            else:
                self.send(-50, 'Orange')

    def send(self, motor_command, color_command):
        motor_command_h, motor_command_l = self.motor_command_calculation(motor_command)
        red_h, red_l, green_h, green_l, blue_h, blue_l = self.color_command_calculation(color_command)

        self.spi.xfer2([
            0x01,
            0x00,
            0b00011111,
            red_h, red_l, green_h, green_l, blue_h, blue_l,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])

    def receive(self):
        data = self.spi.xfer2([
            0x01,
            0x00,
            0b00000011,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0, 0
        ])

        return self.data_conversion(data)

    def swing_up_by_manual(self):
        print("\n***** Manual Swing Up Start!!! *****")

        previousTime = time.perf_counter()
        last_pendulum_radian = math.pi
        motorPWM = 0

        while True:
            # if the difference between the current time and the last time an SPI transaction
            # occurred is greater than the sample time, start a new SPI transaction
            currentTime = time.perf_counter()
            if currentTime - previousTime >= SAMPLE_TIME:
                # print("|| Time difference: {0} s ||".format(currentTime - previousTime))

                previousTime = currentTime

                _, _, pendulum_angle, pendulum_radian = self.receive()

                if -25 < pendulum_angle < 0:
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
                    voltage /= int(8 * np.log(abs(pendulum_angular_velocity)))

                if pendulum_radian >= 0:
                    pendulum_radian = math.pi - pendulum_radian
                else:
                    pendulum_radian = - math.pi + abs(pendulum_radian)

                # if abs(pendulum_angle) > 90:
                if pendulum_angular_velocity < 0:
                    motorPWM = int(-2 * math.cos(pendulum_radian) * voltage)
                else:
                    motorPWM = int(2 * math.cos(pendulum_radian) * voltage)

                self.send(int(motorPWM), 'Cyan')

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

                _, theta, _, alpha = self.receive()

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

                    self.send(int(motorPWM), 'Blue')

                    count += 1
                else:
                    self.send(0, 'Orange')
                    break
        print("\n***** Manual Balance Control Finished!!! *****\n")


if __name__ == "__main__":
    global is_ready, motor_power

    servo = QubeServo2()

    is_ready = False
    motor_power = 0

    while True:
        try:
            previousTime = time.perf_counter()

            while is_ready:
                # if the difference between the current time and the last time an SPI transaction
                # occurred is greater than the sample time, start a new SPI transaction
                currentTime = time.perf_counter()
                if currentTime - previousTime >= SAMPLE_TIME:
                    # print("|| Time difference: {0} s ||".format(currentTime - previousTime))
                    previousTime = currentTime

                    servo.receive()

                    time.sleep(SAMPLE_TIME)

                    motor_angle, motor_radian, _, pendulum_radian = servo.send_and_receive(motor_power, 'Green')

                    servo.pub.publish(
                        topic=MQTT_PUB_TO_ENV_INFO,
                        payload="{0}|{1}|{2}".format(motor_radian, pendulum_radian, motor_angle)
                    )
                    print("|| time: {0} || motor power: {1} ||".format(
                        datetime.utcnow().strftime('%S.%f')[:-1], motor_power
                    ), flush=True)

            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nMain thread KeyboardInterrupted")
            servo.send(0, 'Red')
            sys.exit()
