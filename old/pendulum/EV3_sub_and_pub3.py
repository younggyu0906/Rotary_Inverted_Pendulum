import paho.mqtt.client as mqtt
import time
import datetime
import ev3dev.ev3 as ev3
import sys
from smbus import SMBus
import threading
import ctypes

MQTT_SERVER = '192.168.137.4'

MQTT_MOTOR_AND_PENDULUM_STATE = 'state_info'
MQTT_PENDULUM_STATE_FOR_RECTIFY = 'pendulum_state_for_rectify'

MQTT_MOTOR_RECTIFY_COMPLETE = 'rectify_complete'
MQTT_MOTOR_TO_PENDULUM = 'motor_angle'
MQTT_MOTOR_RECTIFY = 'rectify'
MQTT_MOTOR_STILL_RECTIFY = 'motor_still_rectify'

sub_topic_list = [MQTT_MOTOR_RECTIFY_COMPLETE, MQTT_MOTOR_TO_PENDULUM, MQTT_MOTOR_RECTIFY, MQTT_MOTOR_STILL_RECTIFY]
pub_topic_list = [MQTT_MOTOR_AND_PENDULUM_STATE, MQTT_PENDULUM_STATE_FOR_RECTIFY]

MODE_NORMAL = 0x00
MODE_CALIBRATE = 0x43
MODE_RESET = 0x52


self_pendulum = None
class Pendulum:
    def __init__(self, pub, hta):
        global self_pendulum
        self_pendulum = self

        self.pub = pub
        self.hta = hta

        self.angle = 0
        self.speed = 0
        getting_sensor_data_forever = threading.Thread(target=self.get_pendulum_sensor_data)
        getting_sensor_data_forever.daemon = True
        getting_sensor_data_forever.start()

    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_MOTOR_RECTIFY_COMPLETE)
        client.subscribe(topic=MQTT_MOTOR_TO_PENDULUM)
        client.subscribe(topic=MQTT_MOTOR_RECTIFY)
        client.subscribe(topic=MQTT_MOTOR_STILL_RECTIFY)

    @staticmethod
    def on_message(client, useradta, msg):
        if msg.topic == MQTT_MOTOR_RECTIFY_COMPLETE or msg.topic == MQTT_MOTOR_TO_PENDULUM:
            print(msg.topic, end=',')
            motor_angle = str(msg.payload.decode("utf-8"))
            print(self_pendulum.angle)
            self_pendulum.pub.publish(
                topic=MQTT_MOTOR_AND_PENDULUM_STATE,
                payload=self_pendulum.angle+'|'+self_pendulum.speed+'|'+motor_angle
            )

        if msg.topic == MQTT_MOTOR_RECTIFY or msg.topic == MQTT_MOTOR_STILL_RECTIFY:
            print(msg.topic)
            self_pendulum.pub.publish(
                topic=MQTT_PENDULUM_STATE_FOR_RECTIFY,
                payload=self_pendulum.angle+'|'+self_pendulum.speed
            )

    def    (self):
        while True:
            angle, acc_angle, rpm = self.hta.update()  # [angle, acc_angle, rpm], isError
            # sensing_error_check = abs(abs(float(self.angle)/57) - abs(float(angle)/57))/0.01
            # if sensing_error_check <= 1.5*rpm/9.55 and sensing_error_check >= 0.5*rpm/9.55:
            self.angle = str(angle - 180)
            self.speed = str(rpm)
            time.sleep(0.01)


class HTAngle:
    def __init__(self, bus_no, address):
        self.bus = SMBus(bus_no)
        self.address = address

        self.angle = 0              # angle, degrees(0 - 359)
        self.acc_angle = 0          # accumulated angle, degrees(-2147483648 - 2147483647)
        self.rpm = 0                # rotations per minute (-1000 to 1000)

        self.update()

    def set_mode(self, mode):
        self.bus.write_byte_data(self.address, 0x41, mode)

    def get_mode(self):
        return self.bus.read_byte_data(self.address, 0x41)

    def reset(self):
        # reset accumulated angle
        self.set_mode(MODE_RESET)
        time.sleep(0.1)

        # calibrate angle
        self.set_mode(MODE_CALIBRATE)
        time.sleep(0.1)

        self.update()

    def update(self):
        isError = True
        while isError:
            try:
                data = self.bus.read_i2c_block_data(self.address, 0x41, 10)
                isError = False
            except (TypeError, OSError) as e:
                print(e)
                isError = True

        # error filtering
        if data[1] == data[2] == data[3] == data[4] == data[5] == data[6] == data[7] == data[8] == 0xff:
            return (self.angle, self.acc_angle, self.rpm)

        # angle
        self.angle = (data[1] & 0xff) * 2 + (data[2] & 0x01) + ((data[2] & 0x80) << 1)

        # accumulated angle
        acc = (((data[3] & 0x7f) + (data[4] & 0x80)) * 0x1000000)\
              + (((data[4] & 0x7f) + (data[5] & 0x80)) * 0x10000)\
              + (((data[5] & 0x7f) + (data[6] & 0x80)) * 0x100)\
              + ((data[6] & 0x7f) + (data[7] & 0x80))

        # convert to signed
        if acc > 0x7fffffff:
            self.acc_angle = (0x100000000 - acc) * (-1)
        else:
            self.acc_angle = acc

        # rpm
        rpm = (data[7] & 0x7f) * 0x100 + (data[8] & 0x7f) + (data[9] & 0x80)

        # convert to signed
        if rpm > 0x3fff:
            self.rpm = (0x8000 - rpm) * (-1)
        else:
            self.rpm = rpm
        return (self.angle, self.acc_angle, self.rpm)


if __name__ == "__main__":
    while True:
        try:
            port = ev3.LegoPort("pistorms:BBS2")
            if port.mode != "i2c-thru":
                port.mode = "i2c-thru"
            hta = HTAngle(1, 0x01)
            break
        except Exception as e:
            print("I/O exception.")
            time.sleep(0.1)
    hta.reset()

    pub = mqtt.Client(client_id="pendulum_pub", transport="TCP")
    pub.connect(MQTT_SERVER, 1883, 60)

    pendulum = Pendulum(pub, hta)

    sub = mqtt.Client(client_id="pendulum_sub", transport="TCP")
    sub.on_connect = pendulum.on_connect
    sub.on_message = pendulum.on_message
    sub.connect(MQTT_SERVER, 1883, 60)

    try:
        print("Sub thread started!")
        sub.loop_forever()
    except KeyboardInterrupt:
        print("Sub Interrupted!")
        sub.unsubscribe(sub_topic_list)
        sub.disconnect()
