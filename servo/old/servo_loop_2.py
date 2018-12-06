import paho.mqtt.client as mqtt
import threading
import spidev
import time
import math
import numpy as np
from datetime import datetime

MQTT_SERVER = '10.0.0.1'

MQTT_SUB_FROM_ENV_POWER = 'motor_power'
MQTT_SUB_RESET = 'reset'
MQTT_PUB_RESET_COMPLETE = 'reset_complete'
MQTT_PUB_TO_ENV = 'servo_info'
MQTT_PUB_MOTOR_LIMIT = 'motor_limit_info'

self_servo7 = None

PI = math.pi
MOTOR_PROTECTION_VOLTAGE = 500
UNIT_TIME = 1 / 1000

class QubeServo2:
    def __init__(self):
        global self_servo
        self_servo = self

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.mode = 0b10
        self.spi.max_speed_hz = 1000000

        self.pub_id = 0
        self.last_pub_id = 0
        self.motor_command = 0
        self.is_swing_up = True
        self.is_reset = False
        self.last_motor_radian = 0
        self.last_pendulum_radian = 0
        self.is_action = False

        self.last_sub_time = 0.0
        self.last_pub_time = 0.0

        self.pub = mqtt.Client(client_id="servo_pub", transport="TCP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.sub = mqtt.Client(client_id="servo_sub", transport="TCP")
        self.sub.on_connect = self.on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        thread = threading.Thread(target=self.__sub, args=(self.sub,))
        thread.daemon = True
        thread.start()

        self.reset()

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        print()
        client.subscribe(topic=MQTT_SUB_FROM_ENV_POWER)
        client.subscribe(topic=MQTT_SUB_RESET)

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
            sub.unsubscribe(MQTT_SUB_RESET)
            sub.disconnect()

    @staticmethod
    def on_message(client, useradta, msg):
        if msg.topic == MQTT_SUB_FROM_ENV_POWER:
            # currentTime = float(datetime.utcnow().strftime('%S.%f')[:-1])
            # print(" --> sub time : ", currentTime - self_servo.last_sub_time, flush=True)
            # self_servo.last_sub_time = currentTime

            motor_power_info = str(msg.payload.decode("utf-8")).split('|')
            self_servo.motor_command = int(motor_power_info[0])
            info = motor_power_info[1]
            self_servo.pub_id = motor_power_info[2]

            if info == "swingup":
                self_servo.is_swing_up = True
            elif info == "balance":
                self_servo.is_swing_up = False
                self_servo.set_motor_command()
                self_servo.read_and_pub()
            elif info == "wait":
                self_servo.set_wait()
            elif info == "pendulum_reset":
                print("pendulum_reset")
                self_servo.pendulum_reset(self_servo.pub_id)
        elif msg.topic == MQTT_SUB_RESET:
            motor_power_info = str(msg.payload.decode("utf-8")).split('|')
            self_servo.pub_id = motor_power_info[1]
            self_servo.is_reset = True

    def __data_conversion(self, data):
        # Devoid ID
        device_id = ((data[0] & 0xff) << 8) | (data[1] & 0xff)

        # Motor Encoder Counts
        encoder0 = ((data[2] & 0xff) << 16) | ((data[3] & 0xff) << 8) | (data[4] & 0xff)
        if encoder0 & 0x00800000:
            encoder0 = encoder0 | 0xFF000000
            encoder0 = (0x100000000 - encoder0) * (-1)

        # convert the arm encoder counts to angle theta in radians
        motor_position = encoder0 * (-2.0 * PI / 2048.0)

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
        pendulum_angle = encoder1 * (2.0 * PI / 2048.0) - PI

        return device_id, motor_position, pendulum_angle

    def __motor_command_split(self, motor_command):
        # to signed
        if motor_command & 0x0400:
            motor_command = motor_command | 0xfc00

        # add amplifier bit
        motor_command = (motor_command & 0x7fff) | 0x8000

        # separate into 2 bytes
        motor_command_h = (motor_command & 0xff00) >> 8
        motor_command_l = (motor_command & 0xff)
        return motor_command_h, motor_command_l

    def read_data(self):
        data = self.spi.xfer2([
            0x01,
            0x00,
            0x1f,
            0x00, 0xff, 0x00, 0xff, 0x00, 0xff,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        _, motor_radian, pendulum_radian = self.__data_conversion(data)
        return motor_radian, pendulum_radian

    def __set_motor_command(self, motor_command, color):
        if color == "red":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x00, 0x00, 0x00, 0x00
        elif color == "green":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x03, 0xe7, 0x00, 0x00
        elif color == "blue":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x00, 0x00, 0x03, 0xe7
        elif color == "cyan":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x03, 0xe7, 0x03, 0xe7
        elif color == "magenta":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7
        elif color == "yellow":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x03, 0xe7, 0x00, 0x00
        elif color == "white":
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x03, 0xe7, 0x03, 0xe7, 0x03, 0xe7
        else:
            red_h, red_l, green_h, green_l, blue_h, blue_l = 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

        motor_command_h, motor_command_l = self.__motor_command_split(motor_command)

        data = self.spi.xfer2([
            0x01,
            0x00,
            0x1f,
            red_h, red_l, green_h, green_l, blue_h, blue_l,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])
        _, motor_radian, pendulum_radian = self.__data_conversion(data)

        return motor_radian, pendulum_radian

    def pendulum_reset(self, pub_id):
        self.spi.xfer2([
            0x01,
            0x00,
            0b01011111,
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])

        print("***** Pendulum Reset Complete!!! pub_id : {} ***** ".format(pub_id))

    def reset(self):
        self.spi.xfer2([
            0x01,
            0x00,
            0b01111111,
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        data = self.spi.xfer2([
            0x01,
            0x00,
            0b00011111,
            0x03, 0xe7, 0x00, 0x00, 0x03, 0xe7,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ])
        _, motor_radian, pendulum_radian = self.__data_conversion(data)

    def limit_check(self):
        motor_radian, _ = self.__set_motor_command(self.motor_command, "green")

        if abs(motor_radian) > PI / 2:
            self.pub.publish(topic=MQTT_PUB_MOTOR_LIMIT,
                             payload="limit_position|{0}".format(self.pub_id), qos=0)
            # print("<<=== pub limit position")
            self.protection()
            self.pub.publish(topic=MQTT_PUB_MOTOR_LIMIT,
                             payload="reset_complete|{0}".format(self.pub_id), qos=0)
            # print("<<=== pub reset complete")
        else:
            return

    def protection(self):
        is_protect = True
        while is_protect:
            start_motor_radian, _ = self.read_data()

            time.sleep(UNIT_TIME)

            motor_command = 150 if start_motor_radian > 0 else -150
            end_motor_radian, _ = self.__set_motor_command(motor_command, "red")

            motor_velocity = end_motor_radian - start_motor_radian
            # print("limit m_v:", motor_velocity)

            motor_command = 0
            if end_motor_radian > 0:
                if motor_velocity > 0:
                    motor_command = MOTOR_PROTECTION_VOLTAGE
            else:
                if motor_velocity < 0:
                    motor_command = -MOTOR_PROTECTION_VOLTAGE

            protect_motor_radian, _ = self.__set_motor_command(motor_command, "red")

            time.sleep(UNIT_TIME)

            if abs(protect_motor_radian) < 8 * PI / 18:
                is_protect = False

    def manual_swing_up(self):
        print("\n***** Swing Up Start!!! *****")

        previousTime = time.perf_counter()
        last_pendulum_radian = 0
        motorPWM = 0

        while True:
            # if the difference between the current time and the last time an SPI transaction
            # occurred is greater than the sample time, start a new SPI transaction
            currentTime = time.perf_counter()
            if currentTime - previousTime >= UNIT_TIME:
                # print("|| Time difference: {0} s ||".format(currentTime - previousTime))

                previousTime = currentTime

                motor_radian, pendulum_radian = self.read_data()

                angular_variation = (pendulum_radian - last_pendulum_radian)
                # angular variation filtering
                if angular_variation > 2.5:
                    angular_variation -= math.pi * 2
                elif angular_variation < -2.5:
                    angular_variation += math.pi * 2

                pendulum_angular_velocity = angular_variation / UNIT_TIME

                if -PI * 20 / 180 < pendulum_radian < PI * 20 / 180:
                    self.is_reset = False
                    self.pub.publish(
                        topic=MQTT_PUB_RESET_COMPLETE,
                        payload="{0}|{1}|{2}|{3}|{4}".format(
                            motor_radian, 0, pendulum_radian, pendulum_angular_velocity, self.pub_id
                        ),
                        qos=0
                    )
                    break

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

                self.__set_motor_command(motorPWM, "blue")

        print("\n***** Swing Up complete!!! *****")

    def manual_balance(self):
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

        while count < 5000 / 5:
            # if the difference between the current time and the last time an SPI transaction
            # occurred is greater than the sample time, start a new SPI transaction
            currentTime = time.perf_counter()
            if currentTime - previousTime >= UNIT_TIME*5:
                # print("|| Time difference: {0} s ||".format(currentTime - previousTime))

                previousTime = currentTime

                # LED Blue
                theta, alpha = self.read_data()

                # if the pendulum is within +/-30 degrees of upright, enable balance control
                if abs(alpha) <= (30.0 * math.pi / 180.0):
                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    theta_n = -theta
                    theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.7612 * theta_dot_k1)
                    theta_n_k1 = theta_n
                    theta_dot_k1 = theta_dot

                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    alpha_n = -alpha
                    alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.7612 * alpha_dot_k1)
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
                    motorPWM = int(motorVoltage * (625.0 / 15.0))
                    if motorPWM > 280:
                        motorPWM = 280
                    elif motorPWM < -280:
                        motorPWM = -280

                    # print(motorPWM)

                    self.__set_motor_command(motorPWM, "cyan")

                    count += 1

                else:
                    self.read_data()
                    break
        self.last_motor_radian = theta
        self.last_pendulum_radian = alpha
        self.is_reset = False

        self.pub.publish(
            topic=MQTT_PUB_RESET_COMPLETE,
            payload="{0}|{1}|{2}|{3}|{4}|{5}|{6}|{7}|{8}".format(
                theta, 0, alpha, 0, self.pub_id, theta_n_k1, theta_dot_k1, alpha_n_k1, alpha_dot_k1
            ),
            qos=0
        )


    # read radian and if pub_id is changed, publish to env.
    def read_and_pub(self):
        if self.pub_id != self.last_pub_id and self.is_action:
            motor_radian, pendulum_radian = self.__set_motor_command(self.motor_command, "green")

            motor_velocity = (motor_radian - self.last_motor_radian) / (UNIT_TIME * 5)
            pendulum_velocity = (pendulum_radian - self.last_pendulum_radian) / (UNIT_TIME * 5)

            self.last_motor_radian = motor_radian
            self.last_pendulum_radian = pendulum_radian

            self.is_action = False
            self.last_pub_id = self.pub_id

            self.pub.publish(
                topic=MQTT_PUB_TO_ENV,
                payload="{0}|{1}|{2}|{3}|{4}".format(
                    motor_radian, motor_velocity, pendulum_radian, pendulum_velocity, self.pub_id),
                qos=0
            )
            # currentTime = float(datetime.utcnow().strftime('%S.%f')[:-1])
            # print(" <== pub time : ", currentTime - self.last_pub_time, flush=True)
            # self.last_pub_time = currentTime

    # set motor command to last subscribe command.
    def set_motor_command(self):
        self.is_action = True
        color = "blue" if self.is_swing_up else "green"
        #print(datetime.utcnow().strftime('%S.%f')[:-1], self.motor_command, flush=True)
        self.__set_motor_command(self.motor_command, color)

    def set_wait(self):
        self.is_action = False
        color = "white"
        self.__set_motor_command(0, color)

        motor_radian, pendulum_radian = self.read_data()

        self.pub.publish(
            topic=MQTT_PUB_TO_ENV,
            payload="{0}|{1}|{2}|{3}|{4}".format(
                motor_radian, 0, pendulum_radian, 0, self.pub_id),
            qos=0
        )

if __name__ == "__main__":
    qs = QubeServo2()

    while True:
        try:
            if qs.is_reset:
                qs.manual_swing_up()
            else:
                qs.limit_check()
                time.sleep(0.0001)
        except KeyboardInterrupt:
            print("Main thread KeyboardInterrupted")
            qs.reset()
            break
