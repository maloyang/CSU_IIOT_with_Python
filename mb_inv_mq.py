#!/usr/bin/python
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt  #import the client1
import time

import serial
import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
import time
from struct import *
import random

mbComPort = 'COM8' # for linux: '/dev/ttyUSB0'
baudrate = 9600
databit = 8
parity = 'N'
stopbit = 1
mbTimeout = 100 # ms

def on_connect(client, userdata, flags, rc):
    m="Connected flags"+str(flags)+", result code "+str(rc)+", client_id  "+str(client)
    print(m)

    # first value OFF
    print('set light off!')
    control_light(0)
    client1.publish(topic,0)    

def on_message(client1, userdata, message):
    print("message received  "  ,str(message.payload.decode("utf-8")), message.topic, message.qos, message.retain)
    if message.topic == topic:
        my_message = str(message.payload.decode("utf-8"))
        print("set light: ", my_message)
        if my_message=='1' or my_message==1:
            control_light(1)
        else:
            control_light(0)

def on_log(client, userdata, level, buf):
    print("log: ",buf)

def control_light(value):
    mb_port = serial.Serial(port=mbComPort, baudrate=baudrate, bytesize=databit, parity=parity, stopbits=stopbit)
    master = modbus_rtu.RtuMaster(mb_port)
    master.set_timeout(mbTimeout/1000.0)

    mbId = 1
    addr = 2 #base0

    try:
        #-- FC5: write multi-coils
        rr = master.execute(mbId, cst.WRITE_SINGLE_COIL, addr, output_value=value)
        print("Write(addr, value)=%s" %(str(rr)))

    except Exception as e:#Exception, e:
        print("modbus test Error: " + str(e))

    master._do_close()


def read_power_meter():
    mb_port = serial.Serial(port=mbComPort, baudrate=baudrate, bytesize=databit, parity=parity, stopbits=stopbit)
    master = modbus_rtu.RtuMaster(mb_port)
    master.set_timeout(mbTimeout/1000.0)

    mbId = 4
    #[0x1000-0x1001]=VIn_a
    addr = 0x1000#4096

    v_a = None
    try:
        # FC3
        rr = master.execute(mbId, cst.READ_INPUT_REGISTERS, addr, 4)
        print('read value:', rr)

        # convert to float:
        # IEEE754 ==> (Hi word Hi Bite, Hi word Lo Byte, Lo word Hi Byte, Lo word Lo Byte)
        try:
            v_a_hi = rr[1]
            v_a_lo = rr[0]
            v_a = unpack('>f', pack('>HH', v_a_hi, v_a_lo))
            print('v_a=', v_a)
        except Exception as e:#Exception, e:
            print(e)
    except Exception as e:#Exception, e:
        print("modbus test Error: " + str(e))

    master._do_close()
    if v_a==None:
        v_a=None
    else:
        v_a=v_a[0]
    return v_a

def read_inv(mb_id=1, port='/dev/ttyUSB0', br=9600, databit=8, parity='N', stopbit=1, timeout=1000):
    data = {'time':time.strftime("%Y-%m-%d %H:%M:%S"), 'total_energy':0, 't':[0], 'ac_vawf':[0,0,0,0],
        'dc_vaw':[0,0,0], 'error':[0]}

    try:
        mb_port = serial.Serial(port=port, baudrate=br, bytesize=databit, parity=parity, stopbits=stopbit)
        master = modbus_rtu.RtuMaster(mb_port)
        master.set_timeout(timeout/1000.0)

        #-- start to poll
        addr = 132-1
        # 132, 0: 2B, Daily Energy, Wh (IEEE32 float)
        # 134, 2: 2B, Total Energy, kWh
        # 144, 12: 2B, AC Voltage, V
        # 146, 14: 2B, AC Current, A
        # 148, 16: 2B, AC Power, W
        # 150, 18: 2B, AC Frequency, Hz
        # 152, 20: 2B, DC Power 1, W
        # 154, 22: 2B, DC Voltage 1, v
        # 156, 24: 2B, DC Current 1, A
        # 158, 26: 2B, DC Power 2, W
        # 160, 28: 2B, DC Voltage 2, V
        # 162, 30: 2B, DC Current 2, A
        # 164, 32: 2B, Temperature, oC
        for j in range(3): 
            try:
                rr = master.execute(mb_id, cst.ANALOG_INPUTS, addr, 34)

                today_energy = unpack('>f', pack('>HH', rr[1], rr[0]))[0]/1000

                ac_v = unpack('>f', pack('>HH', rr[13], rr[12]))[0]
                ac_a = unpack('>f', pack('>HH', rr[15], rr[14]))[0]
                ac_w = unpack('>f', pack('>HH', rr[17], rr[16]))[0]
                ac_f = unpack('>f', pack('>HH', rr[19], rr[18]))[0]

                dc_w = unpack('>f', pack('>HH', rr[21], rr[20]))[0]
                dc_v = unpack('>f', pack('>HH', rr[23], rr[22]))[0]
                dc_a = unpack('>f', pack('>HH', rr[25], rr[24]))[0]
                dc_w2 = unpack('>f', pack('>HH', rr[27], rr[26]))[0]
                dc_v2 = unpack('>f', pack('>HH', rr[29], rr[28]))[0]
                dc_a2 = unpack('>f', pack('>HH', rr[31], rr[30]))[0]
                
                data['today_energy'] = today_energy # kWh
                #- ac_vawf (for R, S, T)
                data['ac_vawf'] = [ac_v, ac_a, ac_w, ac_f] # V, A, W, Hz
                #- dc_vaw, (for MPPT1, MPPT2)
                data['dc_vaw'] = [dc_v, dc_a, dc_w, dc_v2, dc_a2, dc_w2] # V, A, W
                
                break #success-->exit to next
            except Exception as e:
                print('poll all error', e)

        master._do_close()
    except Exception as e:
        print("Error: " + str(e))

    return data

# some online free broker:
#   iot.eclipse.org
#   test.mosquitto.org
#   broker.hivemq.com
broker_address= 'broker.hivemq.com' # "iot.eclipse.org"
topic = "malo-iot/light"
client1 = mqtt.Client()    #create new instance
#client1.on_connect = on_connect        #attach function to callback
#client1.on_message = on_message        #attach function to callback
#client1.on_log=on_log

time.sleep(1)
client1.connect(broker_address, 1883, 60)      #connect to broker
client1.subscribe(topic)

#client1.loop_forever()
# 有自己的while loop，所以call loop_start()，不用loop_forever
client1.loop_start()    #start the loop
time.sleep(2)
print("loop start")

if True:#while True:
    #v = read_power_meter()
    #print('V=%s, type(V)=%s' %(v, type(v)))
    
    #v = random.randint(110, 115)
    #client1.publish("malo-iot/voltage", v)

    inv_data = read_inv(22, 'COM6')
    print('inv_data: ', inv_data)
    client1.publish("malo-iot/today_energy", round(inv_data['today_energy'], 2))
    client1.publish("malo-iot/ac_v", round(inv_data['ac_vawf'][0], 2))
    client1.publish("malo-iot/ac_a", round(inv_data['ac_vawf'][1], 2))
    client1.publish("malo-iot/ac_w", round(inv_data['ac_vawf'][2], 2))
    client1.publish("malo-iot/ac_f", round(inv_data['ac_vawf'][3], 2))

    time.sleep(2)
