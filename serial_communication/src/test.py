#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import time
from bombel_msg import bombel_msg
from std_msgs.msg import String

global ser

def shutdown():
    ser.close()
    print "\n Shutdown! \n"

def serial_send(msg):
    line = []

    if ser.isOpen():

        try:
            ser.flushInput() #flush input buffer, discarding all its contents
            ser.flushOutput()#flush output buffer, aborting current output 
            #and discard all that is in buffer

            #write data
            # bytes_sent = ser.write(bytes(14, 'utf-8'))
            bytes_sent = ser.write(msg)
            print(str(bytes_sent) + "write data:" + msg)

            time.sleep(0.1)  #give the serial port sometime to receive the data
            response = ser.readline().decode('ascii')
            # response = ser.readline()
            print("read data: " + response)

        except Exception, e1:
            print "error communicating...: " + str(e1)

    else:
        print "Serial port is closed."

def loop():
    while not rospy.is_shutdown():
        # disp_menu(0)
        # msg_type = raw_input('Choose option:')
        disp_menu(1)
        msg_option = raw_input('Choose option:')
        # if int(msg_type) == bombel_msg.HW_CONFIG:
        hw_config_cmd(msg_option)   
        


        # if str1 == 'q':
        #     rospy.signal_shutdown('Keyboard interrupt')
        # else:
        #     serial_send(str1)


        rate.sleep()

def disp_menu(option):
    if option == 0:
        print('[1]' + 'Hardware configuration')
        print('[2]' + 'JINT')
        print('[3]' + 'OINT')
    elif option == 1:
        print('[1]' + 'Turn motors on')
        print('[2]' + 'Turn motors off')
        print('[3]' + 'Turn fans on')
        print('[4]' + 'Turn fans off')
    else:
        print('Invalid option')
def hw_config_cmd(cmd_type):
    order_type=str(bombel_msg.HW_CONFIG).zfill(2)
    hw_config_cmd=str(cmd_type).zfill(2)
    offset=str(0);

    print(bytesorder_type[0])
    # print(msg + '\n')
    # if len(msg) != bombel_msg.ORDER_LENGTH:
    #     print("Incorrect message format! \n")
    # else:
    #     serial_send(msg)

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
