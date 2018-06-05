#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import struct
import time
from bombel_msg import bombel_msg
from std_msgs.msg import String

global ser

def shutdown():
    ser.close()
    print "\n Shutdown! \n"


def serial_send(msg):
    line = []
    if msg > 99:
        print "Msg is defined as 8bit (int<256)"
        return
    if ser.isOpen():

        try:
            ser.flushInput() #flush input buffer, discarding all its contents
            ser.flushOutput()#flush output buffer, aborting current output 

            bytes_sent = ser.write(struct.pack('!B',msg))

            # time.sleep(0.1)  #give the serial port sometime to receive the data
            # response = ser.readline().decode('ascii')
            # # response = ser.readline()
            # print("read data: " + response)

        except Exception, e1:
            print "error communicating...: " + str(e1)

    else:
        print "Serial port is closed."

def loop():
    while not rospy.is_shutdown():
        disp_menu(0)
        option = int(raw_input('Choose option:'))
        if option == 1:
            disp_menu(1)            
            hw_config_cmd()   
        elif option == 2:
            disp_menu(2)
            joint_space_cmd()
        elif option == 'q':
            rospy.signal_shutdown('Keyboard interrupt')

        rate.sleep()

def disp_menu(option):
    if option == 0:
        print('======MAIN MENU========')
        print('[1]' + 'Hardware configuration')
        print('[2]' + 'JINT')
        print('[3]' + 'OINT')
        print('[Q]' + 'Quit')

    elif option == 1:
        print('======HW_CONFIG========')
        print('[1]' + 'Turn motors on')
        print('[2]' + 'Turn motors off')
        print('[3]' + 'Turn fans on')
        print('[4]' + 'Turn fans off')
    elif option == 2:
        print('======JOINT_SPACE========')
    else:
        print('Invalid option')

def joint_space_cmd():
    dir_flag=0;

    f0=float(raw_input('JOINT 0 :'))
    f1=float(raw_input('JOINT 1 :'))
    f2=float(raw_input('JOINT 2 :'))

    if f0 >= 10 or f1 >=10 or f2 >= 10:
        print("F0 or F1 or F2: Out of range")
        return

    if f0 < 0 :
        dir_flag=1;
        f0=-f0
    if f1 < 0 :
        dir_flag=dir_flag+2;
        f1=-f1
    if f2 < 0 :
        dir_flag=dir_flag+4;
        f2=-f2


    str0="%1.2f" % f0
    str1="%1.2f" % f1
    str2="%1.2f" % f2

    serial_send(10*bombel_msg.JOINT_SPACE+int(str0[0]))
    serial_send(10*int(str0[2])+int(str0[3]))
    serial_send(10*int(str1[0])+int(str1[2]))
    serial_send(10*int(str1[3])+int(str2[0]))
    serial_send(10*int(str2[2])+int(str2[3]))
    serial_send(dir_flag)

def hw_config_cmd():
    option=int(raw_input('Choose option: '))

    if option == bombel_msg.MOTORS_ON:
        action = bombel_msg.MOTORS_ON;
    elif option == bombel_msg.MOTORS_OFF:
        action = bombel_msg.MOTORS_OFF;
    elif option == bombel_msg.FANS_ON:
        action = bombel_msg.FANS_ON;
    elif option == bombel_msg.FANS_OFF:
        action = bombel_msg.FANS_OFF;

    serial_send(10*bombel_msg.HW_CONFIG);
    serial_send(10*(action%10))
    serial_send(00)
    serial_send(00)
    serial_send(00)
    serial_send(00)

def serial_init():
    global ser

    #possible timeout values:
    #    1. None: wait forever, block call
    #    2. 0: non-blocking mode, return immediately
    #    3. x, x is bigger than 0, float allowed, timeout block call

    ser = serial.Serial()
    ser.port = "/dev/ttyACM0"
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check: no parity
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    #ser.timeout = None          #block read
    ser.timeout = 1            #non-block read
    #ser.timeout = 2              #timeout block read
    ser.xonxoff = False     #disable software flow control
    ser.rtscts = False     #disable hardware (RTS/CTS) flow control
    ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
    ser.writeTimeout = 2     #timeout for write

    try: 
        ser.open()
    except Exception, e:
        print "Opening serial port error: " + str(e)
        rospy.signal_shutdown('Keyboard interrupt')

if __name__ == '__main__':
    try:
        rospy.init_node('test', anonymous=True)  
        rospy.on_shutdown(shutdown)
        serial_init()

        rate = rospy.Rate(10) # 10hz 
        loop()
    except rospy.ROSInterruptException:
        pass
