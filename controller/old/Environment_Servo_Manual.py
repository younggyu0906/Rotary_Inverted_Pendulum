import paho.mqtt.client as mqtt
import numpy as np
import threading
import math
import time
import sys
from datetime import datetime

MQTT_SERVER = 'localhost'

# ********** PUB **********
MQTT_PUB_TO_SERVO_RESET = 'reset'
MQTT_PUB_TO_SERVO_MOTOR_POWER = 'motor_power'

# ********** SUB **********
MQTT_SUB_FROM_SERVO_RESET_COMPLETE = 'reset_complete'
MQTT_SUB_FROM_SERVO_INFO = 'info'

ACTION_TIME = 5 / 1000
STATE_SIZE = 6
# BUFFER_SIZE = 5

motor_power_list = [-20, -10, 0, 10, 20]

self_env = None


class Env:
    def __init__(self):
        global self_env

        self_env = self

        self.state_space_shape = (STATE_SIZE,)
        self.action_space_shape = (len(motor_power_list),)

        self.motor_power = 0

        self.last_motor_radian = 0
        self.last_pendulum_radian = 0

        self.reward = 0
        self.steps = 0
        self.state_buffer = []
        self.limit_check_buffer = []

        self.sub = mqtt.Client(client_id="env_sub", transport="TCP")
        self.sub.on_connect = self.on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        sub_thread = threading.Thread(target=self.__sub, args=(self.sub,))
        sub_thread.daemon = True
        sub_thread.start()

        self.pub = mqtt.Client(client_id="env_pub", transport="TCP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("\nmqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_SUB_FROM_SERVO_RESET_COMPLETE)
        client.subscribe(topic=MQTT_SUB_FROM_SERVO_INFO)

    @staticmethod
    def __sub(sub):
        try:
            print("\n***** Sub Thread Start!!! *****")
            sub.loop_forever()
        except KeyboardInterrupt:
            print("\nSub thread KeyboardInterrupted")
            sub.unsubscribe(topic=MQTT_SUB_FROM_SERVO_RESET_COMPLETE)
            sub.unsubscribe(topic=MQTT_SUB_FROM_SERVO_INFO)
            sub.disconnect()

    def on_message(self, client, userdata, msg):
        if msg.topic == MQTT_SUB_FROM_SERVO_RESET_COMPLETE:
            servo_info = str(msg.payload.decode("utf-8")).split('|')
            motor_radian = float(servo_info[0])
            pendulum_radian = float(servo_info[1])

            self.insert_state_to_buffer(motor_radian, pendulum_radian)

        elif msg.topic == MQTT_SUB_FROM_SERVO_INFO:
            servo_info = str(msg.payload.decode("utf-8")).split('|')
            motor_radian = float(servo_info[0])
            pendulum_radian = float(servo_info[1])
            previous_motor_angle = float(servo_info[2])

            self.insert_state_to_buffer(motor_radian, pendulum_radian)
            self.limit_check_buffer.append(previous_motor_angle)

    def insert_state_to_buffer(self, motor_radian, pendulum_radian):
        motor_cosine_theta = math.cos(motor_radian)
        motor_sine_theta = math.sin(motor_radian)
        motor_angular_velocity = (motor_radian - self.last_motor_radian) / ACTION_TIME

        pendulum_cosine_theta = math.cos(pendulum_radian)
        pendulum_sine_theta = math.sin(pendulum_radian)

        angular_variation = (pendulum_radian - self.last_pendulum_radian)
        # angular variation filtering
        if angular_variation > 2.5:
            angular_variation -= math.pi * 2
        elif angular_variation < -2.5:
            angular_variation += math.pi * 2

        pendulum_angular_velocity = angular_variation / ACTION_TIME

        self.last_motor_radian = motor_radian
        self.last_pendulum_radian = pendulum_radian

        # print("|| {0:.3f} || {1:.3f} || {2:.3f} || {3:.3f} || {4:.3f} || {5:.3f} || {6:.3f} || {7:.3f} ||".format(
        #     motor_cosine_theta, motor_sine_theta, motor_angular_velocity, motor_radian,
        #     pendulum_cosine_theta, pendulum_sine_theta, pendulum_angular_velocity, pendulum_radian
        # ), flush=True)
        current_state = [motor_cosine_theta, motor_sine_theta, motor_angular_velocity,
                         pendulum_cosine_theta, pendulum_sine_theta, pendulum_angular_velocity]

        self.state_buffer.append(current_state)

    def reset(self):
        self.steps = 0
        self.state_buffer.clear()
        self.limit_check_buffer.clear()

        self.pub.publish(topic=MQTT_PUB_TO_SERVO_RESET, payload="")

        while len(self.state_buffer) == 0:
            time.sleep(0.0001)

        return self.state_buffer.pop()

    def step(self, action_index):
        self.pub.publish(topic=MQTT_PUB_TO_SERVO_MOTOR_POWER,
                         payload=str(motor_power_list[action_index]))

        while len(self.state_buffer) == 0:
            time.sleep(0.0001)

        pendulum_radian = math.acos(self.state_buffer[0][3])
        pendulum_angular_velocity = self.state_buffer[0][5]

        # 30 degrees = 0.52359 radian
        # self.reward = 30 * (0.25 - pendulum_radian) - (0.01 * abs(pendulum_angular_velocity))
        self.reward = 1 + 0.1 * self.steps + 30 * (0.25 - pendulum_radian)

        self.steps += 1

        done, info = self.is_done(pendulum_radian)
        if done:
            self.pub.publish(topic=MQTT_PUB_TO_SERVO_MOTOR_POWER,
                             payload="done")

        return self.state_buffer.pop(), self.reward, done, info

    def is_done(self, pendulum_radian):
        if abs(self.limit_check_buffer.pop()) > 90:
            self.reward -= 100
            return True, "*** Exceeded Limit ***"
        elif self.steps >= 1000:
            self.reward += 1000
            return True, "*** Success!!! ***"
        elif pendulum_radian > (math.pi / 36):
            self.reward -= 100
            return True, "*** Fail!!! ***"
        else:
            return False, ""

    def close(self):
        print("*************** Close ***************")
        self.pub.publish(topic=MQTT_PUB_TO_SERVO_MOTOR_POWER, payload="done")
        sys.exit()


# if __name__ == "__main__":
#     try:
#         env = Env()
#         count = 0
#         for i in range(1000):
#             env.reset()
#             done = False
#             while not done:
#                 state, reward, done, info = env.step(5)
#                 print("|| state: {0:.5f} || reward: {1:.5f} || done: {2} || info: {3} || time: {4} ||".format(
#                     math.acos(state[3]), reward, done, info, datetime.utcnow().strftime('%S.%f')[:-3]
#                 ))
#                 time.sleep(0.005)
#         env.close()
#     except KeyboardInterrupt:
#         print("\nMain thread KeyboardInterrupted")
#         env.close()
