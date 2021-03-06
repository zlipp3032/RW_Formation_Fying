print('Vehicle State')
from recordtype import recordtype
import numpy as np
from collections import OrderedDict

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None) 
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw',
                                'ux','uy','uz','vel_est_x','vel_est_y','vel_est_z',
                                    ('accVelZError',0),('AR',0),('VC',0),('GT',0),
                                    ('FC',0),('accPosXError',0),('accPosYError',0),
                                    ('accPosZError',0)],default = None)

PreviousState = recordtype('PreviousState',[('velPrev_x',0),
                                            ('velPrev_y',0),('velPrev_z',0),('accPrev_x',0),
                                                ('accPrev_y',0),('accPrev_z',0)], default = None)

Position = recordtype('Position',['x','y','z'], default=None)
Velocity = recordtype('Velocity',[('vx',0),('vy',0),('vz',0)], default = None)


#ComputeControl = recordtype('ComputerControl',[('ux',0),('uy'),('uz',0)], default = None)

Flocking = recordtype('Flocking',['qgx','qgy','qgz',('pgx',0),('pgy',0),('pgz',0)], default = None)
Leader = recordtype('Leader',['qgx','qgy','qgz','pgx','pgy','pgz',('flocking',Flocking())], default = None)

Attitude = recordtype('Attitude',[('roll',0),('pitch',0),('yaw',0)], default = None)
AttThrust = recordtype('AttThrust',['roll','pitch','throttle','yaw'], default = None)

InitialPosition = recordtype('InitialPosition',['xo','yo','zo'], default = None)

#Parameter = recordtype('Parameter',['Ts','peerTimeout','GPSTimeout',
#                                    'expectedMAVs','isComplete','TargetAltitude','kpx','kdx',
#                                        'kpy','kdy','kpz','kdz','targetAltitude','quadMass',
#                                        'gravity','ku_vel','kv_vel','kw_vel','rollLimit','pitchLimit',
#                                        'throttleLimit','stoppingDistance','desiredSpeed','isTakeoff',
#                                        'intGain','InitPos','isLanding','isHovering','isFlocking','alpha1',
#                                        'alpha2','beta','gamma1','gamma2','gamma3','gamma4','desiredDistance',
#                                        'kix','kiy','kiz'], default = None)

Parameter = recordtype('Parameter',['Ts','receivedTime','expectedMAVs','ID','isComplete','GPSTimeout','gains','config','txStateType'], default = None)

#RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID',
#                                                   'batt','time','channels',('test',AttThrust()),
#                                                   ('attitude',Attitude()),('RCLatch',False),('isGPS',False),
#                                                   'lastGPSContact',('position',Position()),('velocity',Velocity()),
#                                                   ('initPos',InitialPosition()),('command',Command()),
#                                                   ('parameters',Parameter()),('timeout',Timeout()),
#                                                   ('leader',Leader()),('previousState',PreviousState()),
#                                                   'flightSeq'], default = None)

MessageState = recordtype('MessageState',['ID',('position',Position()),('velocity',Velocity()),
                                          ('attitude',Attitude())],default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)

class BasicVehicleState(object):
    def __init__(self,other=None):
        self.ID = None
        self.timestamp = None
        self.position = {'x': None, 'y': None, 'z': None}
        self.velocity =  {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.isPropagated = False
        self.counter = 0
        self.isFlocking = False
        #if other is not None:
        #    for k in self.__dict__.keys():
        #        self.__dict__[k] = other.__dict__[k]

    def getCSVLists(self):
        headers = []
        values = []

        headers.append('ID')
        values.append(self.ID)
        headers.append('Counter')
        values.append(self.counter)
        headers.append('Timestamp')
        values.append(self.timestamp)

        headers += ['xPos','yPos','zPos']
        values += [self.position['x'], self.position['y'], self.position['z']]

        headers += ['xVel', 'yVel', 'zVel']
        values += [self.velocity['vx'], self.velocity['vy'], self.velocity['vz']]

        out = OrderedDict(zip(headers,values))
        return out

        #return

class FullVehicleState(BasicVehicleState):
    def __init__(self):
        super(FullVehicleState, self).__init__()
        self.time = 0.00
        self.leader = {'qgx': None, 'qgy': None, 'qgz': None, 'pgx': 0.0, 'pgy': 0.0, 'pgz': 0.0, 'ugx': 0.0, 'ugy': 0.0, 'ugz': 0.0,
                           'qgx_prev': None, 'qgy_prev': None, 'qgz_prev': None, 'pgx_prev': 0.0, 'pgy_prev': 0.0, 'pgz_prev': 0.0,
			    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'roll_prev': 0.0, 'pitch_prev': 0.0, 'yaw_prev': 0.0,
			    'omegaX': 0.0, 'omegaY': 0.0, 'omegaZ': 0.0, 'omegaX_dot': 0.0, 'omegaY_dot': 0.0, 'omegaZ_dot': 0.0,
			    'roll_rate_prev': 0.0, 'pitch_rate_prev': 0.0, 'yaw_rate_prev': 0.0, 'psi_d': 0.0}
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'roll_prev': 0.0, 'pitch_prev': 0.0, 'yaw_prev': 0.0, 'roll_rate': 0.0, 'pitch_rate': 0.0, 'yaw_rate': 0.0}
        self.initPos = {'x': None, 'y': None, 'z': None}
        self.accumulator = {'intXPosError': 0.0, 'intYPosError': 0.0, 'intZPosError': 0.0, 'intZVelError': 0.0}
        self.controlState = {'vx_des': 0.0, 'vy_des': 0.0, 'vz_des': 0.0, 'ux_des': 0.0, 'uy_des': 0.0, 'uz_des': 0.0,
                                 'thrust': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw_rate': 0.0, 'lin_thrust': 0.0, 'lin_roll': 0.0, 'lin_pitch': 0.0,
				 'roll_PWM': None, 'pitch_PWM': None,'throttle_PWM': None, 'yaw_rate_PWM': None}
        self.droneState = {'battVolt': None}
	self.channels = {'roll': None, 'pitch': None, 'throttle': None, 'yaw': None}
	self.avoid = {'ux': 0.0, 'uy': 0.0, 'uz':0.0}
	self.flightSeq = 0
        self.hover = {'x': None, 'y': None, 'z': None}
	self.R2T = np.matrix([[1.5, -1.5, 0.0],[0.43, 0.43, -1.2],[-0.2, -0.2, -0.2]])
        #self.isFlocking = False
        
    def getCSVLists(self):
        base = super(FullVehicleState,self).getCSVLists()
        headers = base.keys()
        values = base.values()

        headers += ['roll', 'pitch', 'yaw']
        values += [self.attitude['roll'], self.attitude['pitch'], self.attitude['yaw']]

	headers += ['roll_rate','pitch_rate','yaw_rate']
	values += [self.attitude['roll_rate'],self.attitude['pitch_rate'],self.attitude['yaw_rate']]

        headers += ['leadXPos', 'leadYPos', 'leadZPos']
        values += [self.leader['qgx'], self.leader['qgy'], self.leader['qgz']]

        headers += ['leadXVel', 'leadYVel','leadZVel']
        values += [self.leader['pgx'], self.leader['pgy'], self.leader['pgz']]

	headers += ['lead_ugx', 'lead_ugy','lead_ugz']
        values += [self.leader['ugx'], self.leader['ugy'], self.leader['ugz']]

	headers += ['leadRoll', 'leadPitch', 'leadYaw','psi_d']
	values += [self.leader['roll'], self.leader['pitch'], self.leader['yaw'],self.leader['psi_d']]

	headers += ['lead_wX','lead_wZ','lead_wY']
	values += [self.leader['omegaX'],self.leader['omegaZ'],self.leader['omegaY']]

	headers += ['lead_wX_dot','lead_wZ_dot','lead_wY_dot']
        values += [self.leader['omegaX_dot'],self.leader['omegaZ_dot'],self.leader['omegaY_dot']]

        headers += ['flightSequence']
        values += [self.flightSeq]

        headers += ['xhover', 'yhover', 'zhover']
        values += [self.hover['x'], self.hover['y'],self.hover['z']]

        headers += ['intXPosError','intYPosError','intZPosError','intZVelError']
        values += [self.accumulator['intXPosError'], self.accumulator['intYPosError'],self.accumulator['intZPosError'],self.accumulator['intZVelError']]

        headers += ['vx_des','vy_des','vz_des','ux','uy','uz']
        values += [self.controlState['vx_des'], self.controlState['vy_des'], self.controlState['vz_des'], self.controlState['ux_des'], self.controlState['uy_des'], self.controlState['uz_des']]  

        headers += ['thrust_des','roll_des','pitch_des','yaw_rate_des']
        values += [self.controlState['thrust'], self.controlState['roll'], self.controlState['pitch'],self.controlState['yaw_rate']]

	headers += ['lin_thrust_des','lin_roll_des','lin_pitch_des']
	values += [self.controlState['lin_thrust'],self.controlState['lin_roll'], self.controlState['lin_pitch']]

        headers += ['tPWM_des','rPWM_des','pPWM_des','yRatePWM']
        values += [self.controlState['throttle_PWM'], self.controlState['roll_PWM'], self.controlState['pitch_PWM'],self.controlState['yaw_rate_PWM']]
        
	headers += ['avoid_ux','avoid_uy','avoid_uz']
	values += [self.avoid['ux'],self.avoid['uy'],self.avoid['uz']]

	headers += ['battVolt']
	values += [self.droneState['battVolt']]

	headers += ['roll_chan','pitch_chan','thrtle_chan','yaw_chan']
	values += [self.channels['roll'],self.channels['pitch'],self.channels['throttle'],self.channels['yaw']]

        out = OrderedDict(zip(headers,values))
        return out
        
    


def recordTypeToLists(rt,prefix =''):
	headers = []
	values = []
	d = rt._asdict()
	for k in d.keys():
		item = d[k]
		if isinstance(item,np.matrix):
			(h,v)=vecToCSV(item,k)
			headers+=h
			values+=v
		elif isinstance(item,rtTypes):	#If this is a valid recordtype (hack)
			(h,v) = recordTypeToLists(item,prefix + k+'_')
			headers+=h
			values += v
		elif isinstance(item,dict):
			for k2 in item.keys():
				if isinstance(item[k2],np.matrix):
					(h,v) = vecToCSV(item[k2],str(k)+str(k2))
					headers+=h
					values+=v
				else:
					headers.append(prefix + k2)
					values.append(item[k2])
		else:
			headers.append(prefix+k)
			values.append(item)

	return (headers,values)

def vecToCSV(mat,prefix):
	outKey = []
	outValue = []		
	for j in range(0,len(mat)):
		outKey.append(prefix+'_'+str(j))
		outValue.append(mat[j,0])
	return (outKey,outValue)
