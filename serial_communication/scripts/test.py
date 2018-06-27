#!/usr/bin/env python
# license removed for brevity
import rospy
import threading
import serial
import struct
import time
from bombel_msg import bombel_msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from math import *

ser=0
jState=0
pub=0
thread=0 

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def on_shutdown():
	ser.close()
	print(bcolors.FAIL+'SysQuitting \n'+bcolors.ENDC)

def shutdown(reason):
	ser.close()
	print(bcolors.FAIL+'Quitting \n'+bcolors.ENDC)
	rospy.signal_shutdown(reason)


def serial_send(msg):
	global ser 

	line = []
	if msg > 255:
		print "Msg is defined as 8bit (int<256) \n"
		return
	if ser.isOpen():

		try:
			# ser.flushInput() #flush input buffer, discarding all its contents
			# print "Sending"+str(msg)
			bytes_sent = ser.write(struct.pack('!B',msg))

		except Exception, e1:
			print "error communicating...: " + str(e1) + "\n"

	else:
		print "Serial port is closed.\n"

def loop():
	while not rospy.is_shutdown():
		disp_menu(0)
		try:
			option = int(raw_input('Choose option:'))
		except ValueError:
			shutdown('Invalid menu option')
		else:
			if option == 1:
				disp_menu(1)		
				try:
					option=int(raw_input('Choose option: '))	
				except Exception, e:
					print "HW MENU: "+str(e) + "\n"
				else:
					hw_config_cmd(option)
			elif option == 2:
				disp_menu(2)

				try:
					f0=float(raw_input('JOINT 0 :'))
					f1=float(raw_input('JOINT 1 :'))
					f2=float(raw_input('JOINT 2 :'))
					speed=int(raw_input('SPEED :'))
				except Exception, e:
					print "JINT MENU: "+str(e) + "\n"
				else:
					joint_space_cmd(f0,f1,f2,speed)

			rate.sleep()

def disp_menu(option):
	if option == 0:
		print('======MAIN MENU========')
		print('[1]' + 'Hardware configuration')
		print('[2]' + 'JINT')
		print('[3]' + 'OINT')
		print('[Q]' + 'Quit')

	elif option == 1:
		print('======MSG_HW_CONFIG========')
		print('[1]' + 'Turn motors on')
		print('[2]' + 'Turn motors off')
		print('[3]' + 'Turn fans on')
		print('[4]' + 'Turn fans off')
	elif option == 2:
		print('======MSG_JOINT_SPACE========')
	else:
		print('Invalid option')

def joint_space_cmd(f0,f1,f2,speed):
	dir_flag=0;

	try:

		f0=int(f0*2048/pi)
		f1=int(f1*2048/pi)
		f2=int(f2*2048/pi)
		

	except ValueError:
		print(bcolors.FAIL+'Incorrect input data type'+bcolors.ENDC)
	else:
		if abs(f0) >= pow(2,12)-1 or abs(f1) >= pow(2,12)-1 or abs(f2) >= pow(2,12)-1:
			print("F0 or F1 or F2: Out of range \n")
			return
		if speed <0 or speed>200 :
			print("Speed Out of range : "+str(speed)+" \n")
			return

		if f0 < 0 :
			dir_flag+=1;
			f0=-f0
		if f1 < 0 :
			dir_flag+=2;
			f1=-f1
		if f2 < 0 :
			dir_flag+=4;
			f2=-f2

		serial_send((dir_flag<<3) + bombel_msg.MSG_JOINT_SPACE)
		serial_send((f0 >> 4))
		serial_send(((f0 & 0x00F)<<4) + (f1>>8))
		serial_send((f1 & 0x0FF))
		serial_send((f2 >> 4))
		serial_send((f2 & 0x00F))
		serial_send(speed)
		serial_send(speed)
		serial_send(speed)


def hw_config_cmd(option):
	action=0

	if option == bombel_msg.HW_MOTORS_ON:
		action = bombel_msg.HW_MOTORS_ON;
	elif option == bombel_msg.HW_MOTORS_OFF:
		action = bombel_msg.HW_MOTORS_OFF
	elif option == bombel_msg.HW_FANS_ON:
		action = bombel_msg.HW_FANS_ON
	elif option == bombel_msg.HW_FANS_OFF:
		action = bombel_msg.HW_FANS_OFF
	else:
		print "Order unrecognized. \n"
		return

	serial_send(bombel_msg.MSG_HW_CONFIG)
	serial_send(action)
	serial_send(0)
	serial_send(0)
	serial_send(0)
	serial_send(0)
	serial_send(0)
	serial_send(0)
	serial_send(0)

def read():
	global ser, jState, pub

	order_type=-1

	while not rospy.is_shutdown():
		try:
			line = ser.readline().decode("ascii").split()

			order_type=int(line[0])

			if order_type == bombel_msg.RESP_ROBOT_STATE :
			
				f0=bin2rad(int(line[1]))
				f1=bin2rad(int(line[2]))
				f2=bin2rad(int(line[3]))

				jState.position[0]=f0
				jState.position[1]=f1
				jState.position[2]=f2

				jState.header.stamp=rospy.Time.now()
				pub.publish(jState)

			elif  order_type == bombel_msg.RESP_ORDER_ERROR :
				print "Order error \n"
			elif  order_type == bombel_msg.RESP_ORDER_BUFFER_OVERFLOW :
				print "Buffer overflow\n"				



		except Exception, e:
			# pass
			print "read_exception " + str(e) + "\n"


def serial_init():
	global ser

	#possible timeout values:
	#	1. None: wait forever, block call
	#	2. 0: non-blocking mode, return immediately
	#	3. x, x is bigger than 0, float allowed, timeout block call

	ser = serial.Serial()
	ser.port = "/dev/ttyACM0"
	ser.baudrate = 115200
	ser.bytesize = serial.EIGHTBITS #number of bits per bytes
	ser.parity = serial.PARITY_NONE #set parity check: no parity
	ser.stopbits = serial.STOPBITS_ONE #number of stop bits
	#ser.timeout = None		  #block read
	ser.timeout = 1			#non-block read
	#ser.timeout = 2			  #timeout block read
	ser.xonxoff = False	 #disable software flow control
	ser.rtscts = False	 #disable hardware (RTS/CTS) flow control
	ser.dsrdtr = False	   #disable hardware (DSR/DTR) flow control
	ser.writeTimeout = 2	 #timeout for write

	try: 
		ser.open()
	except Exception, e:
		print "Opening serial port error: " + str(e) + "\n"
		rospy.signal_shutdown('Keyboard interrupt')
		rospy.on_shutdown(on_shutdown)

def bin2rad(num):
	return num*pi/2048

if __name__ == '__main__':
	# global jState, pub

	f0=0
	f1=0
	f2=0
	delta_s=20
	delta_p=0.2
	speed=120

	try:
		rospy.init_node('test', anonymous=True)  

		serial_init()

		pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
		
		jState=JointState()
		jState.name=["joint0","joint1", "joint2"]
		jState.position=[0.0, 0.0, 0.0]

		rate = rospy.Rate(10) # 10hz 

		thread = threading.Thread(target=read)
		ser.flushInput()
		thread.start()

		# hw_config_cmd(bombel_msg.FANS_ON)
		# hw_config_cmd(bombel_msg.MOTORS_ON)
		
		# joint_space_cmd(f0,f1,f2,120)
		# speed=0
		# for i in range(5):
		# 	f0+=delta_p
		# 	f1+=delta_p
		# 	f2+=delta_p
		# 	speed+=delta_s
		# 	print str(i)+" "+str(speed)
		# 	joint_space_cmd(f0,f1,f2,speed)

		# speed=120
		# for i in range(5):		
		# 	f0+=delta_p
		# 	f1+=delta_p
		# 	f2+=delta_p
		# 	speed-=delta_s
		# 	print str(i)+" "+str(speed)
		# 	joint_space_cmd(f0,f1,f2,speed)

		loop()

	except rospy.ROSInterruptException:
		pass
