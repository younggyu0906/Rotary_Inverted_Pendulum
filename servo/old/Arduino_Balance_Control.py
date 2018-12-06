from datetime import datetime
import spidev
import time
import math

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

    def data_conversion(self, data):
        # Devoid ID
        device_id = ((data[0] & 0xff) << 8) | (data[1] & 0xff)

        # Motor Encoder Counts
        encoder0 = ((data[2] & 0xff) << 16) | ((data[3] & 0xff) << 8) | (data[4] & 0xff)
        if encoder0 & 0x00800000:
            encoder0 = encoder0 | 0xFF000000
            # 2's complement calculate
            encoder0 = (0x100000000 - encoder0) * (-1)

        # convert the arm encoder counts to angle theta in radians
        motor_position = encoder0 * (-2.0 * self.pi / 2048.0)

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
        pendulum_angle = encoder1 * (2.0 * self.pi / 2048.0) - self.pi

        return device_id, motor_position, pendulum_angle

    def send_and_receive(self, a, b, c, d, e, f, motor_command):
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
            a, b, c, d, e, f,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            motor_command_h, motor_command_l
        ])
        return self.data_conversion(data)


if __name__ == "__main__":
    servo = QubeServo2()

    sampleTime = 5 / 1000

    theta_n_k1 = 0.0
    theta_dot_k1 = 0.0
    alpha_n_k1 = 0.0
    alpha_dot_k1 = 0.0

    kp_theta = 2.0
    kd_theta = -2.0
    kp_alpha = -30.0
    kd_alpha = 2.5

    previousTime = time.perf_counter()

    while True:
        # if the difference between the current time and the last time an SPI transaction
        # occurred is greater than the sample time, start a new SPI transaction
        currentTime = time.perf_counter()
        if currentTime - previousTime >= sampleTime:
            # print("|| Time difference: {0} s ||".format(currentTime - previousTime))

            previousTime = currentTime

            # LED Green
            _, theta, alpha = servo.send_and_receive(0x00, 0x00, 0x03, 0xe7, 0x00, 0x00, 0)

            # if the pendulum is within +/-30 degrees of upright, enable balance control
            if abs(alpha) <= (30.0 * math.pi / 180.0):
                # transfer function = 50s/(s+50)
                # z-transform at 1ms = (50z - 50)/(z-0.9512)
                theta_n = -theta
                # theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.6065 * theta_dot_k1)   # 10ms
                theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.7560 * theta_dot_k1)  # 5ms
                #theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.8050 * theta_dot_k1)  # 4ms
                #theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.9512 * theta_dot_k1)  # 1ms
                theta_n_k1 = theta_n
                theta_dot_k1 = theta_dot

                # transfer function = 50s/(s+50)
                # z-transform at 1ms = (50z - 50)/(z-0.9512)
                alpha_n = -alpha
                # alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.6065 * alpha_dot_k1)   # 10ms
                alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.7560 * alpha_dot_k1)  # 5ms
                #alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.8050 * alpha_dot_k1)  # 4ms
                #alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.9512 * alpha_dot_k1)  # 1ms
                alpha_n_k1 = alpha_n
                alpha_dot_k1 = alpha_dot

                # multiply by proportional and derivative gains
                motorVoltage = (theta * kp_theta) + (theta_dot * kd_theta) + (alpha * kp_alpha) + (alpha_dot * kd_alpha)

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

                # motorPWM += 40 - (motorPWM % 40)
                # motorPWM = int(motorPWM / 60) * 60
                # print(alpha, motorPWM, "!!!", flush=True)

                # print("|| Time: {0} || Theta: {1:.5f} || "
                #       "Alpha: {2:.5f} || motorPWM: {3} ||".format(
                #         datetime.utcnow().strftime('%S.%f')[:-3], theta, alpha, motorPWM
                #         ))
                print(theta_n_k1, theta_dot_k1, alpha_n_k1, alpha_dot_k1)

                # time.sleep(0.0001)

                # LED Green
                servo.send_and_receive(0x00, 0x00, 0x03, 0xe7, 0x00, 0x00, int(motorPWM))

            else:
                # turn on the red and green LEDs for orange
                servo.send_and_receive(0x03, 0xe7, 0x01, 0xf4, 0x00, 0x00, 0)