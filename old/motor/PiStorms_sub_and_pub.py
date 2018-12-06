import paho.mqtt.client as mqtt
import time
import datetime
import sys
from PiStorms import PiStorms
psm = PiStorms()
speed = 0

MQTT_SERVER = '10.0.0.1'
MQTT_MOTOR_POWER_TOPIC = 'Motor/power'
MQTT_MOTOR_ANGLE_TOPIC = 'Motor/angle'

pub = mqtt.Client(client_id="motor_angle_pub", transport="TCP")
pub.connect(MQTT_SERVER, 1883, 60)

def on_connect(client, userdata, flags, rc) :
    print("connected with result code " + str(rc))
    client.subscribe(MQTT_MOTOR_POWER_TOPIC)

def on_message(client, useradta, msg):
    #print(msg.payload)
    global speed
    if msg.payload == "rectify":
        psm.BAM1.brake()
        cnt_cleep = 0
        while cnt_cleep < 8:
            print ".",
            sys.stdout.flush()
            time.sleep(1)
            cnt_cleep += 1
        print() 

        psm.BAM1.setSpeed(100)
        time.sleep(0.3)

        psm.BAM1.setSpeed(0)
        time.sleep(0.85)

        psm.BAM1.setSpeed(100)
        time.sleep(0.2)

        psm.BAM1.setSpeed(-100)
        time.sleep(0.2)

        psm.BAM1.setSpeed(0)
        time.sleep(0.05)
    else:
        speed = int(str(msg.payload).split(":")[1])
        isError = True
    
        # reset position
        if speed == 999999:
            psm.BAM1.brake()

            while isError:
                try:
                    angle = psm.BAM1.pos()
                    isError = False if angle < 600 and angle > -600 else True
                except TypeError as e:
                    isError = True
                time.sleep(0.001)

            psm.BAM1.runDegs(-angle, 30, True, True)
            time.sleep(3)
            #psm.BAM1.resetPos()
        psm.BAM1.setSpeed(speed)
        time.sleep(0.05)

    isError = True
    while isError:
        try:
            angle = psm.BAM1.pos()
            isError = False if angle < 600 and angle > -600 else True
            if angle > 360 or angle < -360 and isError == False:
                psm.BAM1.brake()
        except TypeError as e:
            isError = True
    pub.publish(topic=MQTT_MOTOR_ANGLE_TOPIC, payload="angle:" + str(angle))
    #psm.BAM1.floatSync()

sub = mqtt.Client()
sub.on_connect = on_connect
sub.on_message = on_message

sub.connect(MQTT_SERVER, 1883, 60)

try:
    sub.loop_forever()
except KeyboardInterrupt :
    sub.unsubscribe([MQTT_MOTOR_POWER_TOPIC])
    sub.disconnect()
    psm.BAM1.brake()

