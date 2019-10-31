from dronekit import connect,VehicleMode, Vehicle
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
#import jsonpickle
import math as m
from numpy import linalg as LA
from datetime import datetime, timedelta
import numpy as np
import copy
#from curtsies import Input
import defaultConfig
import json

#controlMode = VehicleMode("STABILIZE")

class Controller(threading.Thread):

    def __init__(self,loggingQueue,transmitQueue,receiveQueue,defaultParams,startTime,vehicle):
        threading.Thread.__init__(self)
        self.loggingQueue = loggingQueue
        self.receiveQueue = receiveQueue
	self.transmitQueue = transmitQueue
        self.stateVehicles = {}
	self.sendState = {}
        self.stoprequest = threading.Event()
        self.startTime = startTime
        self.vehicleState = FullVehicleState()
        self.vehicleState.ID = defaultParams.ID # !!!!!!!!!!IMPORTANT NOTE: Be sure to update this value for each vehicle!!!!!!!!
        self.vehicleState.startTime = datetime.now()
        self.vehicleState.parameters = defaultParams
	for i in range(1,defaultParams.expectedMAVs+1):
	    if (not i == self.vehicleState.ID):
		self.stateVehicles[i] = copy.deepcopy(BasicVehicleState())
#        print(self.stateVehicles[2].timestamp)
	self.vehicle = vehicle
        self.lastGCSContact = -1
        self.prepTakeoff()

    def stop(self):
        self.stoprequest.set()
        print "Stop flag set - Control"
    def run(self):
        while(not self.stoprequest.is_set()):
            loopStartTime = datetime.now()
	    self.getVehicleState()
	    #self.pushStateToTxQueue()
	    while (not self.stoprequest.is_set()):
		try:
		    msg = self.receiveQueue.get(False)
		    self.parseMessage(msg)
		except Queue.Empty:
		    break
	    #self.getVehicleState()
            # Need to put a GPS conditional here to ensure we have position data
            if(not self.vehicleState.parameters.config['initPos']):
                self.vehicleState.parameters.config['initPos'] = self.setInitialPos()
                print('Getting Home Location')
            # This is where the switcher sequence will go
            if(self.vehicleState.parameters.config['isGPS'] and True):
                if(not self.checkAbort()):
                    self.switchFlightSequence()
            # Transmit and Log the data and control the rate of the loop
            #self.pushStateToTxQueue()
	    self.pushStateToLoggingQueue()
	    timeToWait = max(self.vehicleState.parameters.Ts - (datetime.now() - loopStartTime).total_seconds(),1E-6)
	    time.sleep(timeToWait)
	    #time.sleep(self.vehicleState.parameters.Ts)
        self.stop()
	#self.releaseControl()
        print "Control Stopped"


    def switchFlightSequence(self):
        arg = self.vehicleState.flightSeq
        flightSequence = {0: self.idleFunction, 1: self.takeoff, 2: self.hover, 3: self.RTL, 4: self.landing, 5: self.virtual_leader, 6: self.formation}
        Keyboard_Command_Handler = flightSequence.get(arg, lambda: 'Invalid Command')
        Keyboard_Command_Handler()

    def idleFunction(self):
        print("Waiting for Command")

    def hover(self):
	self.vehicleState.parameters.config['isFormation'] = False
	if(not self.vehicleState.parameters.config['isHovering']):
	    print('hello')
	    self.getHoverData()
	    self.vehicleState.parameters.config['isHovering'] = True
        self.vehicleState.leader['lat'] = self.vehicleState.hover['lat']
        self.vehicleState.leader['lon'] = self.vehicleState.hover['lon']
        self.vehicleState.leader['qgz'] = self.vehicleState.hover['z']
	self.vehicleState.leader['pgx'] = 0.0
	self.vehicleState.leader['pgy'] = 0.0
	self.vehicleState.leader['pgz'] = 0.0
	self.vehicleState.leader['ugx'] = 0.0
	self.vehicleState.leader['ugy'] = 0.0
	self.vehicleState.leader['ugz'] = 0.0
        if(not self.checkAbort()):
            self.computeControl()
            print('Hovering')

    def RTL(self):
	self.vehicleState.parameters.config['isHovering'] = False
	self.vehicleState.parameters.config['isFormation'] = False
	self.vehicleState.leader['lat'] = self.vehicleState.initPos['lat']
	self.vehicleState.leader['lon'] = self.vehicleState.initPos['lon']
	self.vehicleState.leader['qgz'] = -self.vehicleState.parameters.config['targetAltitude']
	self.vehicleState.leader['pgx'] = 0.0
	self.vehicleState.leader['pgy'] = 0.0
	self.vehicleState.leader['pgz'] = 0.0
	self.vehicleState.leader['ugx'] = 0.0
	self.vehicleState.leader['ugy'] = 0.0
	self.vehicleState.leader['ugz'] = 0.0
        if(not self.checkAbort()):
            self.computeControl()
            print('Commencing RTL')

    def virtual_leader(self):
	self.vehicleState.parameters.config['isHovering'] = False
	self.vehicleState.parameters.config['isFormation'] = False
	if(not self.checkAbort()):
	    self.computeControl()
	    print('Virtual Leader')
	self.vehicleState.parameters.config['hoverVel'] = False
    
    def formation(self):
	self.vehicleState.parameters.config['isHovering'] = False
	self.vehicleState.parameters.config['isFormation'] = True
	if(not self.checkAbort()):
	    self.computeFormationControl()
	    print('Formation Control')
	self.vehicleState.parameters.config['hoverVel'] = False


    def landing(self):
	self.vehicleState.parameters.config['isHovering'] = False
	self.vehicleState.parameters.config['isFormation'] = False
	self.vehicleState.leader['lat'] = self.vehicleState.initPos['lat']
	self.vehicleState.leader['lon'] = self.vehicleState.initPos['lon']
	self.vehicleState.leader['qgz'] = self.vehicleState.initPos['z']
	self.vehicleState.leader['pgx'] = 0.0
	self.vehicleState.leader['pgy'] = 0.0
	self.vehicleState.leader['ugx'] = 0.0
	self.vehicleState.leader['ugy'] = 0.0
	self.vehicleState.leader['ugz'] = 0.0
        if(not self.checkAbort()):
            desDest = self.vehicleState.position['z'] - self.vehicleState.initPos['z']
            self.vehicleState.leader['qgz'] = self.vehicleState.initPos['z']
            self.computeLandingVelocity(desDest)

    def takeoff(self):
	self.vehicleState.leader['lon'] = self.vehicleState.initPos['lon']
	self.vehicleState.leader['lat'] = self.vehicleState.initPos['lat']
	self.vehicleState.leader['qgz'] = -self.vehicleState.parameters.config['targetAltitude']
	self.vehicleState.leader['pgx'] = 0.0
	self.vehicleState.leader['pgy'] = 0.0
	self.vehicleState.leader['ugx'] = 0.0
	self.vehicleState.leader['ugy'] = 0.0
	self.vehicleState.leader['ugz'] = 0.0
	self.vehicleState.parameters.config['isFormation'] = False
        if(not self.vehicleState.parameters.config['isTakeoff']):
            if(not self.vehicleState.position['z'] <= -self.vehicleState.parameters.config['targetAltitude']*0.95):
                if(not self.checkAbort()):
                    desDest = self.vehicleState.position['z'] - self.vehicleState.leader['qgz']
                    self.computeTakeoffVelocity(desDest)
            else:
                print("Reached target altitude")
                self.vehicleState.parameters.config['isTakeoff'] = True
		self.getHoverData()
                self.vehicleState.parameters.config['isHovering'] = True
        else:
            if(self.vehicleState.parameters.config['isHovering']):
                if(not self.checkAbort()):
                    self.hover()
            else:
                if(not self.checkAbort()):
                    self.landing()


    def prepTakeoff(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        print("Arming motors")
        #self.vehicle.channels.overrides = {'3': 1000}
        time.sleep(2)
        #self.vehicle.armed = True

    def computeTakeoffVelocity(self,desDest):
	print(self.vehicleState.leader['qgz'])
        if(abs(desDest) >= self.vehicleState.parameters.config['stoppingDistance']):
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/abs(desDest)
            if (not self.checkAbort()):
                self.computeControl()
                print("Taking off")
        else:
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/self.vehicleState.parameters.config['stoppingDistance']
            if(not self.checkAbort()):
                self.computeControl()
                print("Approaching target altitude")

    def computeLandingVelocity(self,desDest):
        if(abs(desDest) >= self.vehicleState.parameters.config['stoppingDistance']):
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/abs(desDest)
            if(not self.checkAbort()):
                self.computeControl()
                print("Landing")
        elif(self.vehicleState.position['z'] >= (self.vehicleState.initPos['z'] - 0.05)):
            #self.vehicle.channels.overrides = {'3': 1000}
            self.vehicle.armed = False
            self.vehicleState.parameters.config['isTakeoff'] = False
            print("Vehicle landed")
        else:
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/self.vehicleState.parameters.config['stoppingDistance']
            if(not self.checkAbort()):
                self.computeControl()
                print("Approaching landing")

    def computeLeaderVelocity(self,qg):
        qg_prev = np.matrix([[self.vehicleState.leader['qgx_prev']],[self.vehicleState.leader['qgy_prev']],[self.vehicleState.leader['qgz_prev']]])
        pg_prev = np.matrix([[self.vehicleState.leader['pgx_prev']],[self.vehicleState.leader['pgy_prev']],[self.vehicleState.leader['pgz_prev']]])
        if(not self.vehicleState.parameters.config['hoverVel']):
	    temp = (1.0 - self.vehicleState.parameters.gains['leadVelGain'])/self.vehicleState.parameters.Ts
	    tempDelta = qg - qg_prev
	    pg = np.dot(temp,tempDelta) + np.dot(self.vehicleState.parameters.gains['leadVelGain'],pg_prev)
	    self.vehicleState.parameters.config['hoverVel'] = True
        else:
            pg =  np.matrix([[self.vehicleState.leader['pgx']],[self.vehicleState.leader['pgy']],[self.vehicleState.leader['pgz']]])
        #print(qg)
	#print(pg)
	self.vehicleState.leader['qgx_prev'] = self.vehicleState.leader['qgx']
        self.vehicleState.leader['qgy_prev'] = self.vehicleState.leader['qgy']
        self.vehicleState.leader['qgz_prev'] = self.vehicleState.leader['qgz']
        self.vehicleState.leader['pgx_prev'] = self.vehicleState.leader['pgx']
        self.vehicleState.leader['pgy_prev'] = self.vehicleState.leader['pgy']
        self.vehicleState.leader['pgz_prev'] = self.vehicleState.leader['pgz']
        self.vehicleState.leader['pgx'] = copy.copy(pg[0,0])
        self.vehicleState.leader['pgy'] = copy.copy(pg[1,0])
        self.vehicleState.leader['pgz'] = copy.copy(pg[2,0])
        return pg

    def computeRotationSequence(self):
	# Initiate the vectors
	att_g = np.matrix([[self.vehicleState.leader['roll']],[self.vehicleState.leader['pitch']],[self.vehicleState.leader['yaw']]])
	att_g_prev = np.matrix([[self.vehicleState.leader['roll_prev']],[self.vehicleState.leader['pitch_prev']],[self.vehicleState.leader['yaw_prev']]])
	att_g_rate_prev = np.matrix([[self.vehicleState.leader['roll_rate_prev']],[self.vehicleState.leader['pitch_rate_prev']],[self.vehicleState.leader['yaw_rate_prev']]])
	# Estimate the leader angular rates
	#dAtt_g = (att_g - att_g_prev)
	#dAtt_g[0,0] = self.wrapToPi(dAtt_g[0,0])
	#dAtt_g[1,0] = self.wrapToPi(dAtt_g[1,0])
	#dAtt_g[2,0] = self.wrapToPi(dAtt_g[2,0])
	#temp1 = 1.0 - self.vehicleState.parameters.gains['leadAttGain']
	#temp2 = np.dot(1.0/self.vehicleState.parameters.Ts,dAtt_g)
	#att_g_rate = np.dot(temp1,temp2) + np.dot(self.vehicleState.parameters.gains['leadAttGain'],att_g_rate_prev)
	#self.vehicleState.leader['omegaZ'] = att_g_rate[2,0]
	#print(self.vehicleState.leader['omegaZ'])
	# Set the body-fixed rate vector
	omega = np.matrix([[self.vehicleState.leader['omegaX']],[self.vehicleState.leader['omegaY']],[self.vehicleState.leader['omegaZ']]]) # Note that these are the body fixed rotation rates, NOT the Euler rates!!!!
	omega_dot = np.matrix([[self.vehicleState.leader['omegaX_dot']],[self.vehicleState.leader['omegaY_dot']],[self.vehicleState.leader['omegaZ_dot']]])
	# Compute the X, Y, Z rotation matrices and their derivatives
	RX = np.matrix([[1,0,0],[0,np.cos(att_g[0,0]),np.sin(att_g[0,0])],[0,-np.sin(att_g[0,0]),np.cos(att_g[0,0])]])
	RY = np.matrix([[np.cos(att_g[1,0]),0,-np.sin(att_g[1,0])],[0,1,0],[np.sin(att_g[1,0]),0,np.cos(att_g[1,0])]])
	RZ = np.matrix([[np.cos(att_g[2,0]),np.sin(att_g[2,0]),0],[-np.sin(att_g[2,0]),np.cos(att_g[2,0]),0],[0,0,1]])
        #RX_temp = np.matrix([[0,0,0],[0,-np.sin(att_g[0,0]),np.cos(att_g[0,0])],[0,-np.cos(att_g[0,0]),-np.sin(att_g[0,0])]])
        #RY_temp = np.matrix([[-np.sin(att_g[1,0]),0,-np.cos(att_g[1,0])],[0,0,0],[np.cos(att_g[1,0]),0,-np.sin(att_g[1,0])]])
        #RZ_temp = np.matrix([[-np.sin(att_g[2,0]),np.cos(att_g[2,0]),0],[-np.cos(att_g[2,0]),-np.sin(att_g[2,0]),0],[0,0,0]])
	#RX_dot = np.dot(att_g_rate[0,0],RX_temp)
	#RY_dot = np.dot(att_g_rate[1,0],RY_temp)
	#RZ_dot = np.dot(att_g_rate[2,0],RZ_temp)
	# Compute the 3-2-1 Euler Rotation Matrix and its derivative
	R_inertial_to_body = np.dot(np.dot(RX,RY),RZ)
	#Rg_dot = np.dot(np.dot(RZ_dot,RY),RX) + np.dot(np.dot(RZ,RY_dot),RX) + np.dot(np.dot(RZ,RY),RX_dot)
	Omega_body = np.matrix([[0,-omega[2,0],omega[1,0]],[omega[2,0],0,-omega[0,0]],[-omega[1,0],omega[0,0],0]])
	Omega_dot_body = np.matrix([[0,-omega_dot[2,0],omega_dot[1,0]],[omega_dot[2,0],0,-omega_dot[0,0]],[-omega_dot[1,0],omega_dot[0,0],0]])
	#print(Omega_body)
	# Update previous states
	self.vehicleState.leader['roll_prev'] = att_g[0,0]
	self.vehicleState.leader['pitch_prev'] = att_g[1,0]
	self.vehicleState.leader['yaw_prev'] = att_g[2,0]
        #self.vehicleState.leader['roll_rate_prev'] = att_g_rate[0,0]
        #self.vehicleState.leader['pitch_rate_prev'] = att_g_rate[1,0]
        #self.vehicleState.leader['yaw_rate_prev'] = att_g_rate[2,0]
	return R_inertial_to_body,Omega_body,Omega_dot_body


    def parseMessage(self,msg):
        ID = msg.content['ID']
        if (msg.content['type'] == 0):
		self.updateGlobalStateWithData(ID,msg)
	#elif (msg.content['type'] == 1):
	elif(msg.content['type'] == 1):
	    #self.stateVehicles[ID] = copy.deepcopy(BasicVehicleState())
	    	self.stateVehicles[ID].ID = ID
	    	self.stateVehicles[ID].counter += 1
	    	self.stateVehicles[ID].position = msg.content['position']
	    	self.stateVehicles[ID].velocity = msg.content['velocity']
	    	self.stateVehicles[ID].timestamp = datetime.now()
	#else:
	#	self.getVehicleState()
        #print(self.stateVehicles)
        
    def getVehicleState(self):
	self.vehicleState.position['lat'] = self.vehicle.location.global_relative_frame.lat
	self.vehicleState.position['lon'] = self.vehicle.location.global_relative_frame.lon
	self.vehicleState.position['z'] = -self.vehicle.location.global_relative_frame.alt # This needs to be verified that up is negative
	self.vehicleState.velocity['vx'] = self.vehicle.velocity[0]
	self.vehicleState.velocity['vy'] = self.vehicle.velocity[1]
	self.vehicleState.velocity['vz'] = -self.vehicle.velocity[2] # This needs to be verified that up is negative
	self.vehicleState.attitude['yaw'] = self.vehicle.attitude.yaw
	self.vehicleState.attitude['roll'] = self.vehicle.attitude.roll
	self.vehicleState.attitude['pitch'] = self.vehicle.attitude.pitch
	self.vehicleState.attitude['roll_rate'] = self.vehicle.attitude.rollspeed
        self.vehicleState.attitude['pitch_rate'] = self.vehicle.attitude.pitchspeed
        self.vehicleState.attitude['yaw_rate'] = self.vehicle.attitude.yawspeed
        self.vehicleState.attitude['time'] = self.vehicle.attitude.time
        self.vehicleState.attitude['rel_time'] = (self.vehicleState.attitude['time'] - self.startTime).total_seconds()
        self.vehicleState.channels['roll'] = self.vehicle.channels['1']
        self.vehicleState.channels['pitch'] = self.vehicle.channels['2']
        self.vehicleState.channels['throttle'] = self.vehicle.channels['3']
        self.vehicleState.channels['yaw'] = self.vehicle.channels['4']
        self.vehicleState.droneState['battVolt'] = self.vehicle.battery.voltage
        self.vehicleState.timestamp = self.vehicle.location.global_relative_frame.time

    def updateGlobalStateWithData(self,ID,msg):
        self.vehicleState.flightSeq = msg.content['flightSeq']
        # Update the leader states
        self.vehicleState.leader['qgx'] = msg.content['leader']['qg'][0]
        self.vehicleState.leader['qgy'] = msg.content['leader']['qg'][1]
        self.vehicleState.leader['qgz'] = msg.content['leader']['qg'][2]
	self.vehicleState.leader['roll'] = msg.content['leader']['roll']
	self.vehicleState.leader['pitch'] = msg.content['leader']['pitch']
	self.vehicleState.leader['yaw'] = msg.content['leader']['yaw']
	self.vehicleState.leader['omegaX'] = msg.content['leader']['omega'][0]
	self.vehicleState.leader['omegaY'] = msg.content['leader']['omega'][1]
	self.vehicleState.leader['omegaZ'] = msg.content['leader']['omega'][2]
	#print(msg.content['leader']['yaw'])
	#print(self.vehicleState.leader['qgz'])
	self.vehicleState.leader['omegaX_dot'] = msg.content['leader']['omega_dot'][0]
        self.vehicleState.leader['omegaY_dot'] = msg.content['leader']['omega_dot'][1]
        self.vehicleState.leader['omegaZ_dot'] = msg.content['leader']['omega_dot'][2]
	#print(self.vehicleState.leader['omegaZ_dot'])
	self.vehicleState.leader['pgx'] = msg.content['leader']['pg'][0]
	self.vehicleState.leader['pgy'] = msg.content['leader']['pg'][1]
	self.vehicleState.leader['pgz'] = msg.content['leader']['pg'][2]
        self.vehicleState.leader['ugx'] = msg.content['leader']['ug'][0]
        self.vehicleState.leader['ugy'] = msg.content['leader']['ug'][1]
        self.vehicleState.leader['ugz'] = msg.content['leader']['ug'][2]
	self.vehicleState.leader['lat'] = msg.content['leader']['lat']
	self.vehicleState.leader['lon'] = msg.content['leader']['lon']
	#print(self.vehicle.attitude)

        
    def pushStateToTxQueue(self):
	self.sendState['position'] = self.vehicleState.position
	self.sendState['velocity'] = self.vehicleState.velocity
	self.sendState['ID'] = self.vehicleState.ID
	self.sendState['type'] = 1 # 1 = UAV, 0 = Ground Station
	self.sendState['timestamp'] = self.vehicleState.timestamp
	#msg = Message()
	#msg.type = "UAV"
	#msg.sendTime = datetime.now()
	#msg.content = self.sendState
	#print(msg)
	self.transmitQueue.put(self.sendState)
	#return msg

    def pushStateToLoggingQueue(self):
        msg = Message()
        msg.type = "UAV_LOG"
        msg.sendTime = time.time()
        msg.content = {}
        msg.content['thisState'] = copy.deepcopy(self.vehicleState)
        msg.content['stateVehicles'] = copy.deepcopy(self.stateVehicles)
        self.loggingQueue.put(msg)

    def setInitialPos(self):
        initPosSet = False
        #print(self.vehicleState.position['z'])
        if(self.vehicleState.position['lat']): #and self.vehicleState.position['z']<-0.115):
            self.vehicleState.initPos = copy.copy(self.vehicleState.position)
            initPosSet = True
        return initPosSet

    def checkAbort(self):
        if(self.checkTimeouts()):
            self.vehicleState.abortReason = "Timeout"
        #    self.rigdBodyState.RCLatch = True
        #    self.rigidBodyState.isGPS = False
            self.releaseControl()
            return True
        #! Check the proper flight mode
        if(not self.vehicle.mode == 'STABILIZE'):
	    self.vehicleState.abortReason = "Wrong F.M."
            self.releaseControl()
            return True
	if(self.vehicle.channels['8'] == 1000 or self.vehicle.channels['6'] >= 1400):
	    self.vehicleState.abortReasion = "Pilot Override"
	    #self.vehicle.mode == VehicleMode('ALT_HOLD')
	    self.releaseControl()
	    return True
        return False

    def checkTimeouts(self):
	didTimeout = False
	# GCS Timeout
	if (False):
	    print('hello')
	    didTimeout = True
	return didTimeout

    def agentRotate(self):
	#dAtt = np.matrix([[0.0],[0.0],[0.0]])
	att = np.matrix([[self.vehicleState.attitude['roll']],[self.vehicleState.attitude['pitch']],[self.vehicleState.attitude['yaw']]])
	att_prev = np.matrix([[self.vehicleState.attitude['roll_prev']],[self.vehicleState.attitude['pitch_prev']],[self.vehicleState.attitude['yaw_prev']]])
	att_rate_prev = np.matrix([[self.vehicleState.attitude['roll_rate']],[self.vehicleState.attitude['pitch_rate']],[self.vehicleState.attitude['yaw_rate']]])
	# Compute Rotation rate of self
	dAtt = att - att_prev
        dAtt[0,0] = self.wrapToPi(dAtt[0,0])
        dAtt[1,0] = self.wrapToPi(dAtt[1,0])
        dAtt[2,0] = self.wrapToPi(dAtt[2,0])
	temp1 = 1.0 - self.vehicleState.parameters.gains['attRate']
	temp2 = np.dot(1.0/self.vehicleState.parameters.Ts,dAtt)
	att_rate = np.dot(temp1,temp2) + np.dot(self.vehicleState.parameters.gains['attRate'],att_rate_prev)
	self.vehicleState.attitude['roll_prev'] = att[0,0]
	self.vehicleState.attitude['pitch_prev'] = att[1,0]
	self.vehicleState.attitude['yaw_prev'] = att[2,0]
	self.vehicleState.attitude['roll_rate'] = att_rate[0,0]
	self.vehicleState.attitude['pitch_rate'] = att_rate[1,0]
	self.vehicleState.attitude['yaw_rate'] = att_rate[2,0]
	return att_rate


    def getHoverData(self):
        self.vehicleState.parameters.config['hoverVel'] = True
        self.vehicleState.hover = copy.copy(self.vehicleState.position)
        self.vehicleState.leader['pgx'] = 0
        self.vehicleState.leader['pgy'] = 0
        self.vehicleState.leader['pgz'] = 0
	print(self.vehicleState.hover)

    def computeControl(self):
	ID = self.vehicleState.ID
        gains = self.vehicleState.parameters.gains
        Ts = self.vehicleState.parameters.Ts
        qi = np.matrix([[self.vehicleState.position['lat']],[self.vehicleState.position['lon']],[self.vehicleState.position['z']]])
        pi = np.matrix([[self.vehicleState.velocity['vx']],[self.vehicleState.velocity['vy']],[self.vehicleState.velocity['vz']]])
	att_rate = self.agentRotate()
        # Compute the leader stuff
	qg =  np.matrix([[self.vehicleState.leader['lat']],[self.vehicleState.leader['lon']],[self.vehicleState.leader['qgz']]])
        pg =  np.matrix([[self.vehicleState.leader['pgx']],[self.vehicleState.leader['pgy']],[self.vehicleState.leader['pgz']]])
	ug =  np.matrix([[self.vehicleState.leader['ugx']],[self.vehicleState.leader['ugy']],[self.vehicleState.leader['ugz']]])
	qg_prime = np.matrix([[self.vehicleState.leader['qgx']],[self.vehicleState.leader['qgy']],[0.0]])
	#print((qg,pg))
	#pg = self.computeLeaderVelocity(qg)
	Rg,Omega,Omega_dot = self.computeRotationSequence()
	Rg_IB = np.transpose(Rg)
	Rg_dot = np.dot(Rg_IB,Omega)
	Rg_ddot = np.dot(Rg_dot,Omega) + np.dot(Rg_IB,Omega_dot)
	# Compute the relative position of the leader and the agent
	dq = self.getRelPos(qi,qg) #+ np.dot(Rg_IB,self.vehicleState.R2T[:,ID-1])
	#if (self.vehicleState.flightSeq == 5):
	#	dq = dq + qg_prime
	self.vehicleState.leader['dx'] = dq[0,0]
	self.vehicleState.leader['dy'] = dq[1,0]
	#print(np.dot(Rg,self.vehicleState.R2T[:,ID-1]))
	#print(self.vehicleState.leader['omegaY'])
        intPrep = np.dot(self.vehicleState.parameters.Ts,dq)
        kp = np.matrix([[gains['kpx'], 0, 0], [0, gains['kpy'], 0], [ 0, 0, gains['kpz']]])
        kd = np.matrix([[gains['kdx'], 0, 0], [0, gains['kdy'], 0], [ 0, 0, gains['kdz']]])
        ki = np.matrix([[gains['kix'], 0, 0], [0, gains['kiy'], 0], [ 0, 0, gains['kiz']]])
        # Integrate the position error
        accPosPrev = np.matrix([[self.vehicleState.accumulator['intXPosError']],[self.vehicleState.accumulator['intYPosError']],[self.vehicleState.accumulator['intZPosError']]])
        accPosError = self.antiWindupVec(qg,np.matrix([[-10],[-10],[-10]]),np.matrix([[10],[10],[10]]),accPosPrev,intPrep)
        # Compute the control (note the negative feedback gives us the following deltas)
        dp = pg - pi #+ np.dot(Rg_dot,self.vehicleState.R2T[:,ID-1])
	temp_dq = self.controlFunction(dq)
	temp_dp = self.controlFunction(dp)
        uk = ug + np.dot(kp,temp_dq) + np.dot(kd,temp_dp) + np.dot(ki,accPosError)
	# Add the collision avoidance term
	#avoid,scale = self.collisionAvoidance(qi,pi,Ts,ID)
	#uk = np.dot(scale,uk) + avoid
	self.vehicleState.leader['psi_d'] = 0.0
	# Estimate the desired velocity
        pkp = np.matrix([[self.vehicleState.controlState['vx_des']],[self.vehicleState.controlState['vy_des']],[self.vehicleState.controlState['vz_des']]])
        #ukp = np.matrix([[self.vehicleState.controlState['ux_des']],[self.vehicleState.controlState['uy_des']],[self.vehicleState.controlState['uz_des']]])
	#temp1 = (uk + ukp)/2.0
        #temp2 = Ts*temp1
        pk = pkp + np.dot(Ts,uk)
        self.updateControlState(pi,pk,uk,accPosError,Ts)
        #print(accPosError)
        


    def computeFormationControl(self):
	gains = self.vehicleState.parameters.gains
	Ts = self.vehicleState.parameters.Ts
	ID = self.vehicleState.ID
	MAVs = self.vehicleState.parameters.expectedMAVs + 1
	qi = np.matrix([[self.vehicleState.position['lat']],[self.vehicleState.position['lon']],[self.vehicleState.position['z']]])
        pi = np.matrix([[self.vehicleState.velocity['vx']],[self.vehicleState.velocity['vy']],[self.vehicleState.velocity['vz']]])
	att_rate = self.agentRotate()
	### Leader Stuff ###
	qg =  np.matrix([[self.vehicleState.leader['lat']],[self.vehicleState.leader['lon']],[self.vehicleState.leader['qgz']]])
        pg =  np.matrix([[self.vehicleState.leader['pgx']],[self.vehicleState.leader['pgy']],[self.vehicleState.leader['pgz']]])
        ug =  np.matrix([[self.vehicleState.leader['ugx']],[self.vehicleState.leader['ugy']],[self.vehicleState.leader['ugz']]])
        #pg = self.computeLeaderVelocity(qg)
	#print((qg,pg))
	# Compute the rotation sequence of the leader
	Rg,Omega,Omega_dot = self.computeRotationSequence()
	#print(Omega)
	#print('BREAK')
	Rg_IB = np.transpose(Rg)
	#print(np.dot(Rg_IB,self.vehicleState.R2T[:,ID-1]))
	Rg_dot = np.dot(Rg_IB,Omega)
	Rg_ddot = np.dot(Rg_dot,Omega) + np.dot(Rg_IB,Omega_dot)
	# Set up the control matrices
	kAlpha12 = np.matrix([[gains['alpha12'], 0, 0], [0, gains['alpha12'], 0], [0, 0, gains['alphaZ']]])
	kAlpha23 = np.matrix([[gains['alpha23'], 0, 0], [0, gains['alpha23'], 0], [0, 0, gains['alphaZ']]])
        kBeta12 = np.matrix([[gains['beta12'], 0, 0], [0, gains['beta12'], 0], [0, 0, gains['betaZ']]])
	kBeta23 = np.matrix([[gains['beta23'], 0, 0], [0, gains['beta23'], 0], [0, 0, gains['betaZ']]])
        kGamma = np.matrix([[gains['gammaX'], 0, 0], [0, gains['gammaY'], 0], [ 0, 0, gains['gammaZ']]])
	kEta = np.matrix([[gains['etaX'], 0, 0], [0, gains['etaY'], 0], [ 0, 0, gains['etaZ']]])
	interAgent = np.matrix([[0],[0],[0]])
	accPosError = np.matrix([[0],[0],[0]]) # Note that this is to ensure the updateControlState() function works
	### Compute the inter-agent control ###
	for i in range(1,MAVs):
	    if(not i == ID):
		if(True):
		    #print('Neighbor %d',i)
		    if (i == 1):
			kAlpha = kAlpha12
			kBeta = kBeta12
		    else:
			kAlpha = kAlpha23
			kBeta = kBeta23
		    #print(kAlpha,i)
		    #print(kBeta,i)
		    j_stamp = self.stateVehicles[i].timestamp
		    #print(j_stamp)
		    qj = np.matrix([[self.stateVehicles[i].position['lat']],[self.stateVehicles[i].position['lon']],[self.stateVehicles[i].position['z']]])
		    pj = np.matrix([[self.stateVehicles[i].velocity['vx']],[self.stateVehicles[i].velocity['vy']],[self.stateVehicles[i].velocity['vz']]])
		    #qj_prime = qj + (j_stamp - self.vehicleState.timestamp)*pj
		    #qi,qj = self.vectorIntegratePosition(qi,pi,qj,pj,i)
		    dij = (self.vehicleState.R2T[:,ID-1] - self.vehicleState.R2T[:,i-1])
		    dqj = self.getRelPos(qi,qj) + np.dot(Rg_IB,dij)
		    dpj = pj - pi + np.dot(Rg_dot,dij)
		    #####################
		    #! Use this for linear control
		    #interAgent = interAgent + np.dot(kAlpha,dqj) + np.dot(kBeta,dpj)
		    #####################
		    #! Use this for nonlinear control
		    temp_dqj = self.controlFunction(dqj)
		    temp_dpj = self.controlFunction(dpj)
		    #print(temp_dqj)
		    interAgent = interAgent + np.dot(kAlpha,temp_dqj) + np.dot(kBeta,dpj)
		    self.stateVehicles[i].position['dx'] = dqj[0,0]
		    self.stateVehicles[i].position['dy'] = dqj[1,0]
		    ####################
	### Compute the leader control ###
	dia = self.vehicleState.R2T[:,ID-1]
	dqg = self.getRelPos(qi,qg) + np.dot(Rg_IB,dia)
	dpg = pg - pi + np.dot(Rg_dot,dia)
	self.vehicleState.leader['dx'] = dqg[0,0]
	self.vehicleState.leader['dy'] = dqg[1,0]
	################################
	#! Use this for linear control
	#leader = np.dot(kGamma,dqg) + np.dot(kEta,dpg)
	################################
	#! Use this for nonlinear control
	temp_dqg = self.controlFunction(dqg)
	temp_dpg = self.controlFunction(dpg)
	#print(temp_dqg)
	leader = np.dot(kGamma,temp_dqg) + np.dot(kEta,temp_dpg)
	### Compute desired yaw angle ###
	#rel_pos = self.getRelPos(qg,qi) # ===> dqi = qi - qg
	dig = np.dot(Rg_IB,-dia)
	self.vehicleState.leader['psi_d'] = m.atan2(dig[1,0],dig[0,0])
	### Compute the formation control ###
	uk = ug + np.dot(Rg_ddot,dia) + interAgent + leader
	### Add the collision avoidance term ###
	#avoid,scale = self.collisionAvoidance(qi,pi,Ts,ID)
	#uk = np.dot(scale,uk) + avoid
	### Esitmate the desired velocity ###
	pkp = np.matrix([[self.vehicleState.controlState['vx_des']],[self.vehicleState.controlState['vy_des']],[self.vehicleState.controlState['vz_des']]])
        pk = pkp + np.dot(Ts,uk)
        self.updateControlState(pi,pk,uk,accPosError,Ts)

    def collisionAvoidance(self,qi,pi,Ts,ID):
	MAVs = self.vehicleState.parameters.expectedMAVs + 1
	gains = self.vehicleState.parameters.gains
	Ts = 0.2
	config = self.vehicleState.parameters.config
	ui = np.matrix([[0.0],[0.0],[0.0]])
	uj = np.matrix([[0.0],[0.0],[0.0]])
	qi_hat = np.dot(Ts,pi) + qi
	scale = 1.0
	for i in range(1,MAVs):
	    if(i != ID):
	        qj = np.matrix([[self.stateVehicles[i].position['x']],[self.stateVehicles[i].position['y']],[self.stateVehicles[i].position['z']]])
                pj = np.matrix([[self.stateVehicles[i].velocity['vx']],[self.stateVehicles[i].velocity['vy']],[self.stateVehicles[i].velocity['vz']]])
	        qj_hat = np.dot(Ts,pj) + qj
	        dq_hat = qj_hat[0:2,0] - qi_hat[0:2,0]
	        mag = np.linalg.norm(dq_hat)
	        if(mag < config['collision_radius']):
		    dq = qj[0:2,0] - qi[0:2,0]
		    dp =  - pi[0:2,0] # This term acts as a damper that drives the velocity to zero so that we can avoid the "spring" effect
		    temp1 = np.power(mag,2)/np.power(config['collision_radius'],2)
		    Phi = gains['ca2']*(1/(gains['ca1'] + 1) - 1/(gains['ca1'] + temp1))
		    uj[0:2,0] = np.dot(Phi,dq) + np.dot(gains['ca_vel'],dp)
		    scale = min(scale,temp1)
	        else:
		    uj = np.matrix([[0.0],[0.0],[0.0]])
	        ui = ui + uj
	self.vehicleState.avoid['ux'] = ui[0,0]
	self.vehicleState.avoid['uy'] = ui[1,0]
	self.vehicleState.avoid['uz'] = ui[2,0]
	return ui,scale

    def updateControlState(self,pi,pk,uk,accPosError,Ts):
        config = self.vehicleState.parameters.config
        gains = self.vehicleState.parameters.gains
        self.vehicleState.accumulator['intXPosError'] = accPosError[0,0]
        self.vehicleState.accumulator['intYPosError'] = accPosError[1,0]
        self.vehicleState.accumulator['intZPosError'] = accPosError[2,0]
        self.vehicleState.controlState['vx_des'] = pk[0,0]
        self.vehicleState.controlState['vy_des'] = pk[1,0]
        self.vehicleState.controlState['vz_des'] = pk[2,0]
        self.vehicleState.controlState['ux_des'] = uk[0,0]
        self.vehicleState.controlState['uy_des'] = uk[1,0]
        self.vehicleState.controlState['uz_des'] = uk[2,0]
	if(not self.vehicleState.parameters.config['isFormation']):
	    ku = gains['ku_vel_pid']
	    kv = gains['kv_vel_pid']
	    kw = gains['kw_vel_pid']
	    intGain = gains['intGain']
	else:
	    ku = gains['ku_vel_form']
	    kv = gains['kv_vel_form']
	    kw = gains['kw_vel_form']
	    intGain = gains['intGain']
	#print(ku,kv,kw,intGain)
        # Update the integrator for velocity term
        accVelError = (self.vehicleState.velocity['vz'] - self.vehicleState.controlState['vz_des'])*Ts
        self.vehicleState.accumulator['intZVelError'] = self.antiWindup(pk[2,0],-10,10,self.vehicleState.accumulator['intZVelError'],accVelError)
        # Compute the attitude and thrust commands
	rp_command,thrust = self.computeMiddleLoopControl(config,gains,uk,pk,pi)
        self.vehicleState.controlState['thrust'] = config['quadMass']*(config['grav'] - uk[2,0] + kw*(pi[2,0] - pk[2,0]) + intGain*self.vehicleState.accumulator['intZVelError'])/(np.cos(self.vehicleState.attitude['roll'])*np.cos(self.vehicleState.attitude['pitch']))
        self.vehicleState.controlState['pitch'] = np.arctan(((uk[0,0] - ku*(pi[0,0] - pk[0,0]))*np.cos(self.vehicleState.attitude['yaw']) + (uk[1,0] - kv*(pi[1,0] - pk[1,0]))*np.sin(self.vehicleState.attitude['yaw']))/(-config['grav'] + uk[2,0] - kw*(pi[2,0] - pk[2,0]) - intGain*self.vehicleState.accumulator['intZVelError']))
        self.vehicleState.controlState['roll'] = np.arctan(((uk[0,0] - ku*(pi[0,0] - pk[0,0]))*np.cos(self.vehicleState.controlState['pitch'])*np.sin(self.vehicleState.attitude['yaw']) - (uk[1,0] - kv*(pi[1,0] - pk[1,0]))*np.cos(self.vehicleState.controlState['pitch'])*np.cos(self.vehicleState.attitude['yaw']))/(-config['grav'] + uk[2,0] - kw*(pi[2,0] - pk[2,0]) - intGain*self.vehicleState.accumulator['intZVelError']))
        #print(rp_command)
	#print((self.vehicleState.controlState['pitch'],self.vehicleState.controlState['roll']))
	#print(thrust)
	#print(self.vehicleState.controlState['thrust'])
	#dPsi = self.wrapToPi((self.vehicleState.leader['psi_d'] - self.vehicleState.attitude['yaw']))
	dPsi =  self.wrapToPi(0.0 - self.vehicleState.attitude['yaw'])
	self.vehicleState.controlState['yaw_rate'] = gains['yawP']*dPsi
	self.scaleAndSendControl()

    def scaleAndSendControl(self):
        config = self.vehicleState.parameters.config
        y = self.vehicleState.controlState['thrust']
        x = self.vehicle.battery.voltage
        #x = 1
        ROLL = 1500 + (500/config['rollLimit'])*self.vehicleState.controlState['roll']
        PITCH = 1500 + (500/config['pitchLimit'])*self.vehicleState.controlState['pitch']
        THROTTLE = 1000*(1.0 + (y - 4.869)/(0.2067*(x*x - 23.71)))
        YAW = 1500 + (500/config['yawLimit'])*self.vehicleState.controlState['yaw_rate']
	#print(YAW)
	# Saturate the commands to keep in range of the spec'd input values
        self.vehicleState.controlState['roll_PWM'] = self.saturate(ROLL,1000,2000)
        self.vehicleState.controlState['pitch_PWM'] = self.saturate(PITCH,1000,2000)
        self.vehicleState.controlState['throttle_PWM'] = self.saturate(THROTTLE,1000,2000)
	self.vehicleState.controlState['yaw_rate_PWM'] = self.saturate(YAW,1000,2000)
	#if (not self.vehicleState.attitude['time'] == self.vehicleState.attitude['prev_time']):
	if( True):
		self.vehicleState.attitude['prev_time'] = self.vehicleState.attitude['time']
		#print('hello')
        	#self.vehicle.channels.overrides = {'1': self.vehicleState.controlState['roll_PWM'], '2': self.vehicleState.controlState['pitch_PWM'], '3':self.vehicleState.controlState['throttle_PWM'], '4':self.vehicleState.controlState['yaw_rate_PWM']}


    def controlFunction(self,delta):
	#! Linear Control
	#val = delta

	#! Normalized Dynamics (3-Dims arecoupled) ---> output vector has magnitude 1
	#scale = 1.0
	#num = delta
	#temp1 = LA.norm(delta) ** 2
	#den = np.sqrt(scale + temp1)
	#val = num / den

	#! Normalized Dynamics ---> Magnitude of each direction approaches 1
	scale = 1.0
	lin_prop = 0.25
	num = delta
	temp1 = np.power(delta,2)
	den = np.sqrt(scale + lin_prop*temp1)
	val = num / den

	#! Hyperbolic Tangent ---> Magnitude of each direction approaches 1
	#val = np.empty(3)
	#val[0] = m.tanh(delta[0])
	#val[1] = m.tanh(delta[1])
	#val[2] = m.tanh(delta[2])
	return val

    def computeMiddleLoopControl(self,config,gains,uk,pk,pi):
	pi_prime = np.matrix([[0.0],[0.0]])
	uk_prime = np.matrix([[0.0],[0.0]])
	pk_prime = np.matrix([[0.0],[0.0]])
	pi_prime[0,0] = pi[0,0]
	pi_prime[1,0] = pi[1,0]
	uk_prime[0,0] = uk[0,0]
	uk_prime[1,0] = uk[1,0]
	pk_prime[0,0] = pk[0,0]
	pk_prime[1,0] = pk[1,0]
	theta_tilde = np.matrix([[0.0],[0.0]])
	rp_att = np.matrix([[self.vehicleState.attitude['pitch']],[self.vehicleState.attitude['roll']]])
	#! Determine the middle-loop velocity gains
	if(not self.vehicleState.parameters.config['isFormation']):
            ku = gains['ku_vel_pid']
            kv = gains['kv_vel_pid']
            kw = gains['kw_vel_pid']
            intGain = gains['intGain']
        else:
            ku = gains['ku_vel_form']
            kv = gains['kv_vel_form']
            kw = gains['kw_vel_form']
            intGain = gains['intGain']
	#! Compute the desired Thrust
	thrust =  config['quadMass']*(config['grav'] - uk[2,0] + kw*(pi[2,0] - pk[2,0]) + intGain*self.vehicleState.accumulator['intZVelError'])/(np.cos(self.vehicleState.attitude['roll'])*np.cos(self.vehicleState.attitude['pitch']))
	#! Compute matrix values for middle-loop control
	Br = np.matrix([[-np.cos(self.vehicleState.attitude['yaw']), -np.sin(self.vehicleState.attitude['yaw'])],[-np.sin(self.vehicleState.attitude['yaw']), np.cos(self.vehicleState.attitude['yaw'])]])
	temp_Br_dot = np.matrix([[np.sin(self.vehicleState.attitude['yaw']), -np.cos(self.vehicleState.attitude['yaw'])],[-np.cos(self.vehicleState.attitude['yaw']), -np.sin(self.vehicleState.attitude['yaw'])]])
	Br_dot_inv = np.dot(self.vehicleState.attitude['yaw_rate'],temp_Br_dot)
	#! Set the gain matrices for middle-loop
	A_d_tilde = np.matrix([[-ku , 0.0], [0.0, -kv]])
	A_2_tilde = np.dot(-gains['mid_loop_converge'],np.eye(2,2))
	#! Estimate the acceleration state
	temp_thrust = thrust/config['quadMass']
	x_ddot = np.dot(temp_thrust,np.dot(Br,rp_att)) #Note: x_ddot[0,0] = u_dot and x_ddot[1,0] = v_dot
	#! Compute some errors
	vel_error = pi_prime - pk_prime
	acc_error = x_ddot - uk_prime
	#! Compute the desired roll and pitch
	temp_thrust_inv = 1.0/temp_thrust
	Br_Ad_tilde = np.dot(Br,A_d_tilde)
	theta_d = np.dot(temp_thrust_inv,np.dot(Br_Ad_tilde,vel_error)) + np.dot(temp_thrust_inv,np.dot(Br,uk_prime)) #theta_d[0,0] = pitch_d and theta_d[1,0] = roll_d
	theta_tilde[0,0] = self.wrapToPi(rp_att[0,0] - theta_d[0,0])
	theta_tilde[1,0] = self.wrapToPi(rp_att[1,0] - theta_d[1,0])
	#! Compute the desired roll and pitch rates
	Br_dot_inv_Ad_tilde = np.dot(Br_dot_inv,A_d_tilde)
	theta_dot_d = np.dot(temp_thrust_inv,np.dot(Br_dot_inv_Ad_tilde,vel_error)) + np.dot(temp_thrust_inv,np.dot(Br_Ad_tilde,acc_error)) + np.dot(temp_thrust_inv,np.dot(Br_dot_inv,uk_prime))
	#! Compute the attitude commands
	inv_a = 1.0/gains['att_settle_time']
	inv_b = 1.0/gains['mid_vel_scale']
	inv_b_thrust = inv_b*temp_thrust
	rp_command = - np.dot(inv_b_thrust,np.dot(Br,vel_error)) + np.dot(inv_a,theta_dot_d) + theta_d + np.dot(inv_a,np.dot(A_2_tilde,theta_tilde))
	self.vehicleState.controlState['lin_thrust'] = thrust
	self.vehicleState.controlState['lin_pitch'] = rp_command[0,0]
	self.vehicleState.controlState['lin_roll'] = rp_command[1,0]
	return rp_command,thrust



    def antiWindup(self,value,lowLimit,highLimit,accumulator,toAdd):
        if(value>=highLimit): #Saturation and anti-windup
            if(toAdd < 0):
                accumulator = accumulator + toAdd
        elif(value<=lowLimit):
            if(toAdd > 0):
                accumulator = accumulator + toAdd		
        else:
            accumulator = accumulator + toAdd
        return accumulator

    def antiWindupVec(self,value,lowLimit,highLimit,accumulator,toAdd):
        for i in range(0,3):
            if(value[i,0]>=highLimit[i,0]): #Saturation and anti-windup
                if(toAdd[i,0] < 0):
                    accumulator[i,0] = accumulator[i,0] + toAdd[i,0]
            elif(value[i,0]<=lowLimit[i,0]):
                if(toAdd[i,0] > 0):
                    accumulator[i,0] = accumulator[i,0] + toAdd[i,0]		
            else:
                accumulator[i,0] = accumulator[i,0] + toAdd[i,0]
        return accumulator

    def vectorNorm(self,vec):
        eta = np.sqrt(m.pow(vec[0,0],2) + m.pow(vec[1,0],2) + m.pow(vec[2,0],2))
        return eta

    def saturate(self, value, minimum, maximum):
        out = max(value,minimum)
        out = min(out,maximum)
        return out

    def vectorIntegratePosition(self, qi, pi, qj, pj, id):
	print('Neigbor %d: %d',id,self.stateVehicles[id].timestamp)
	print('Neigbor %d: %d',self.vehicleState.ID,self.vehicleState.timestamp)
	if (self.stateVehicles[id].timestamp > self.vehicleState.timestamp):
	    qi_prime = qi + (self.stateVehicles[id].timestamp - self.vehicleState.timestamp)*pi
	    qj_prime = qj
	    print('Time for i')
	else:
	    qi_prime = qi
	    qj_prime = qj + (self.vehicleState.timestamp - self.stateVehicles[id].timestamp)*pj
	    print('Time for j')
	return qi_prime,qj_prime

    def releaseControl(self):
        self.vehicle.channels.overrides = {}
        print self.vehicle.channels.overrides
        print "Channels Cleared"

    def diffFunction(self,pos1,pos2):
        delta = pos2 - pos1
        return delta

    def wrapToPi(self,value):
	return self.wrapTo2Pi(value + m.pi) - m.pi

    def wrapTo2Pi(self,value):
	if(value<0):
	    n = m.ceil(abs(value / (2*m.pi)))
	    value += n*2.*m.pi
	    positiveInput = False
	else:
	    positiveInput = True
	value = m.fmod(value, 2*m.pi);
	if (value == 0 and positiveInput):
	    value = 2*m.pi
	return value

    # The getRelPos function gives the relative position of agent 2 relative to agent 1 using GPS coordinates
    # This function assumes the earth is flat over short diatnces
    # Accuracy of the this function gets worse as we move closer to the north and south poles
    # (+ dx) = agent 2 is north of agent 1 and (- dx) = agent 2 is south of agent 1
    # (+ dy) = agent 2 is east of agent 1 and (- dy) = agent 2 is west of agent 1
    # Particularly useful for NED coordinate frame
    def getRelPos(self,pos1,pos2): #returns the x y delta position of p2-p1
	c = 40074784 # from https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
#	print pos1
#	print pos2
#	print pos2[0,0]
#	print pos2[0,1]
	dy = (pos2[1,0]-pos1[1,0]) * c * m.cos(m.radians( (pos1[0,0]+pos2[0,0])/ 2))/360
#	print dx
	dx = (pos2[0,0]-pos1[0,0]) * c /360
#	dz = pos2['alt']-pas1['alt']
	dz = pos2[2,0] - pos1[2,0]
	return np.matrix([[dx], [dy],[dz]])



    # Velocity commands with respect to home location directionally
    # Be sure tp set up the home location and know your bearings; update the table below before you fly
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend
    ###
    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        #    """
        #    Move vehicle in direction based on specified velocity vectors.
        #    """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode( #msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle on 1 Hz cycle
        self.vehicle.send_mavlink(msg)
        #for x in range(0,duration):
            #self.vehicle.send_mavlink(msg)
            #time.sleep(self.rigidBodyState.parameters.Ts)
