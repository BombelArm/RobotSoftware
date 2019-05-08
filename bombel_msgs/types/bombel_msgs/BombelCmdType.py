#!/usr/bin/env python

class BombelCmdType():

	def SoftStop(self):
		return 0

	def HardStop(self):
		return 1
	
	def Start(self):
		return 2
	
	def SetNextPosition(self):
		return 3
	
	def WriteEncodersToDriver(self):
		return 4
	
	def SetPosition(self):
		return 5

	def StopWhenError(self):
		return 6

	def DoNotStopWhenError(self):
		return 7
