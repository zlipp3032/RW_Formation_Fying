from vehicleState import Parameter
import time
import numpy as np
import math as m
#from dronekit import VehicleMode

def getParams():
    defaultParams = Parameter()
    defaultParams.IP = '192.168.2.1'
    defaultParams.receivedTime = time.time()
    defaultParams.Ts = 0.05
    defaultParams.expectedMAVs = 2
    defaultParams.ID = 1
    defaultParams.isComplete = True
    defaultParams.GPSTimeout = 1.0
    defaultParams.txStateType = 'basic'
    #defaultParams.desiredPosition = np.array([[-1],[-0.5],[0]])
    defaultParams.gains = {'intGain': 0.2, 'velGain': 0.1, 'attRate': 0.2, 'yawP': 1.0, 'mid_loop_converge': 0.01, 'att_settle_time': 7.0, 'mid_vel_scale': 1500.0,
			   'leadAttGain': 0.5, 'leadVelGain': 0.2, # Used to smooth leader estimates
			   'ca1': 0.001, 'ca2': 1.2, 'ca_vel': 0.4, # Collision avoidance gains
			   'ku_vel_pid': 2.5, 'kv_vel_pid': 2.5, 'kw_vel_pid': 1.5, # Middle-loop velocity controller gains for the PID controller
                           'kpx': 0.3, 'kdx': 1.3, 'kix': 0, # PID gains
                           'kpy': 0.3, 'kdy': 1.3, 'kiy': 0, # PID gains
                           'kpz': 0.8, 'kdz': 1.8, 'kiz': 0, # PID gains
                           'ku_vel_form': 1.75, 'kv_vel_form': 1.75, 'kw_vel_form': 1.5, # Middle-Loop velocity controller gains for the Formation controller
                           'alpha12': 0.15, 'beta12': 0.8, 'gammaX': 0.0, 'etaX': 0.0, # R2T gains
			   'alpha23': 0.15, 'beta23': 0.8, 'gammaY': 0.0, 'etaY': 0.0, # R2T gains
			   'alphaZ': 0.0, 'betaZ': 0.0, 'gammaZ': 0.75, 'etaZ': 1.75, # R2T gains
			   'alpha1': 0.001, 'alpha2': 0.09, # << Flocking gains
                           'beta': 0.25, # Flocking gains
                           'gamma1': 0.3, 'gamma2': 1.2, 'gamma3': 0.3, 'gamma4': 1.3, # Flocking gains
                           'd': 1.7 # Flocking gains>>
                           }
    defaultParams.config = {'rollLimit': 0.3491, #0.7845, # Used in the PWM scaling function (this is the roll angle maxin radians)
                            'pitchLimit': 0.3491, #0.7845, # Used in the PWM scaling function (this is is the angle max in radians)
			    'yawLimit': 1.0, # Used in the PWM scaling function (this is the yaw rate limit)
                            'stoppingDistance': 0.2, # Used to switch velocity computation in potential function
                            'desiredSpeed': -0.01, # Desired speed in velocity potential function
                            'targetAltitude': 1.0, # Target altitude on takeoff
			    'collision_radius': 1.0, # Minimum distance to determine if you agent is in the repulsion set
                            'quadMass': 1.67,
#			    'quadMass': 1.77,
                            'grav': 9.81,
                            'isGPS': True,
                            'isTakeoff': False,
			    'isTakingOff': = False,
                            'initPos': False,
                            'isLanding': False,
                            'isHovering': False,
                            'isFormation': False,
                            'ignoreSelfPackets': True,
                            'hoverVel': True,
			    'GCS_timeout': 5, # seconds
			    'Form_timeout': 4 # seconds
                            }
        
    
                           
    
    return defaultParams
