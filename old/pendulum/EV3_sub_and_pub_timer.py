import paho.mqtt.client as mqtt
import time
from datetime import datetime
import ev3dev.ev3 as ev3
from smbus import SMBus
import threading

MQTT_SERVER = '192.168.137.4'

MQTT_PENDULUM_STATE = 'pendulum_state_info'

MODE_NORMAL = 0x00
MODE_CALIBRATE = 0x43
MODE_RESET = 0x52

self_pendulum = None


class HTAngle:
    def __init__(self, bus_no, address):
        self.bus = SMBus(bus_no)
        self.address = address

        self.angle = 0              # angle, degrees(datetime.utcnow().strftime('%H-%M-%S.%f')[:-3]0 - 359)
        self.acc_angle = 0          # accumulated angle, degrees(-2147483648 - 2147483647)
        self.rpm = 0                # rotations per minute (-1000 to 1000)

        self.current_data1_bit = False
        self.current_data2_bit = False
        self.last_data1_bit = False
        self.last_data2_bit = False

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
        ########################################################################
        ### * ERROR FILTERING!                                              ###
        ### (1) data = 0xff                                                  ###
        ### (2) data[1]의 최상위 비트 반전 (data[0]의 최상위비트)                 ###
        ### (3) data[2]의 최상위 비트 반전                                      ###
        ########################################################################
        isError = True
        while isError:
            try:
                data = self.bus.read_i2c_block_data(self.address, 0x41, 10)
                # error filtering (1)
                if data[1] == data[2] == data[3] == data[4] == data[5] == data[6] == data[7] == data[8] == 0xff:
                    isError = True
                    time.sleep(0.0001)
                    continue
                if data[2] == 0xff:
                    isError = True
                    time.sleep(0.0001)
                    continue
                # not error
                else:
                    isError = False
            except (TypeError, OSError) as e:
                print("update error: {0}".format(str(e)))
                isError = True
                time.sleep(0.0001)

        # error filtering (3)
        self.current_data1_bit = True if data[1] & 0x20 == 0x20 else False
        self.current_data2_bit = True if data[2] & 0x80 == 0x80 else False
        if self.last_data1_bit and self.last_data2_bit:
            if self.current_data1_bit and not self.current_data2_bit:
                data[2] = data[2] | 0x80
        self.last_data1_bit = self.current_data1_bit
        self.last_data2_bit = self.current_data2_bit

        # angle
        self.angle = (data[1] & 0x7f) * 2 + (data[2] & 0x01) + ((data[2] & 0x80) << 1)  # error filtering (2)

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


class Pendulum:
    def __init__(self, pub, hta):
        global self_pendulum
        self_pendulum = self

        self.pub = pub
        self.hta = hta

        self.angle = 0

        self.isReady = False

    def timer_thread(self):
        print()
        print("***** update and pub pendulum angle started! (30 ms) *****")
        while True:
            self.isReady = True
            # time.sleep(0.015)
            time.sleep(0.03)

    def update_and_pub_thread(self):
        while True:
            if self.isReady:
                self.isReady = False
                self.angle, _, _ = self.hta.update()    # [angle, acc_angle, rpm]
                self.angle = str(self.angle - 180)
                self.pub.publish(
                    topic=MQTT_PENDULUM_STATE,
                    payload=self.angle + '|' + datetime.utcnow().strftime('%S.%f')[:-3]
                )
            else:
                time.sleep(0.0001)


if __name__ == "__main__":
    while True:
        try:
            port = ev3.LegoPort("pistorms:BBS2")
            if port.mode != "i2c-thru":
                port.mode = "i2c-thru"
            break
        except Exception as e:
            print("I/O exception: {0}".format(str(e)))
            time.sleep(0.1)

    hta = HTAngle(1, 0x01)
    hta.reset()

    pub = mqtt.Client(client_id="pendulum_pub", transport="TCP")
    pub.connect(MQTT_SERVER, 1883, 60)

    pendulum = Pendulum(pub, hta)

    timer_thread = threading.Thread(target=pendulum.timer_thread)
    update_and_pub_thread = threading.Thread(target=pendulum.update_and_pub_thread)

    timer_thread.daemon = True
    update_and_pub_thread.daemon = True

    timer_thread.start()
    update_and_pub_thread.start()

    while True:
        time.sleep(1)
