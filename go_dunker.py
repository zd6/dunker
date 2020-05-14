import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm


# Get distances measurements from each joint center to base frame (useful for forward kinematics)
def get_joint():
	X = []
	Y = []
	Z = []
	result,vector=vrep.simxGetObjectPosition(clientID, joint_one_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_two_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_three_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_four_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_five_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_six_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	X = np.round(X, decimals = 3)
	Y = np.round(Y, decimals = 3)
	Z = np.round(Z, decimals = 3)
	return X,Y,Z

# Function that used to move joints
def SetJointPosition(theta):
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	time.sleep(0.05)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	time.sleep(0.05)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	time.sleep(0.05)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	time.sleep(0.05)
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	time.sleep(0.05)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	time.sleep(0.5)

# Function that reads suction pad sensor
def GetSuctionPad():	
	result, suck = vrep.simxGetIntegerSignal(
		clientID, 'suc_IO', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get suction variable')
	#print('Getsuction',suck)
	return suck

def GetballGPS_x():
	result, x = vrep.simxGetFloatSignal(
		clientID, 'GPS_x', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get gps x')
	return x


def GetballGPS_y():
	result, y = vrep.simxGetFloatSignal(
		clientID, 'GPS_y', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get gps y')
	return y


def GetballGPS_z():
	result, z = vrep.simxGetFloatSignal(
		clientID, 'GPS_z', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get gps z')
	return z

# Return skew matrix of a omega
def omega(w):
	return np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])

def RotMat(orientation):
	#print(orientation)
	Rx = np.array([[1,0,0],[0,np.cos(orientation[0]),np.sin(orientation[0])],[0,-np.sin(orientation[0]),np.cos(orientation[0])]])
	Ry = np.array([[np.cos(orientation[1]),0,-np.sin(orientation[1])],[0,1,0],[np.sin(orientation[1]),0,np.cos(orientation[1])]])
	Rz = np.array([[np.cos(orientation[2]),np.sin(orientation[2]),0],[-np.sin(orientation[2]),np.cos(orientation[2]),0],[0,0,1]])
	R = np.matmul(Rx,np.matmul(Ry,Rz))@np.array([[1,0,0], [0, 1, 0], [0, 0, 1]])
	return R

# Function that calculate suction pad position and orientation
def CalcSuctionPosition():	
	theta = GetJointAngle()
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	p = np.array([[0.38934922218322754, 0.2357194572687149, 0.2155454158782959]]).T
	orientation = [1.538409948348999, 0.0014859443763270974, -0.06876461207866669]
	R = RotMat(orientation)
	#print(R, p)
	M = np.vstack((np.hstack((R, p)), [0, 0, 0, 1]))
	theta = GetJointAngle()
	ps = [[-0.14829915761947632, 0.007646083831787109, 0.2555693984031677], [-0.15156295895576477, 0.11850930750370026, 0.2569207549095154], [0.09151709079742432, 0.11841396987438202, 0.24002958834171295], [0.3042745590209961, 0.11834046244621277, 0.22535544633865356], [0.38829654455184937, 0.11898066103458405, 0.21962565183639526], [0.38941383361816406, 0.11837434768676758, 0.21948370337486267]] 
	o1 = omega([0, 0, 1])
	S_1 = np.vstack((np.hstack((o1, np.array([np.cross([0, 0, -1], ps[0])]).T)), np.zeros(4)))
	o2 = omega([0, 1, 0])
	S_2 = np.vstack((np.hstack((o2, np.array([np.cross([0, -1, 0], ps[1])]).T)), np.zeros(4)))
	o3 = omega([0, 1, 0])
	S_3 = np.vstack((np.hstack((o3, np.array([np.cross([0, -1, 0], ps[2])]).T)), np.zeros(4)))
	o4 = omega([0, 1, 0])
	S_4 = np.vstack((np.hstack((o4, np.array([np.cross([0, -1, 0], ps[3])]).T)), np.zeros(4)))
	o5 = omega([-1, 0, 0])
	S_5 = np.vstack((np.hstack((o5, np.array([np.cross([1, 0, 0], ps[4])]).T)), np.zeros(4)))
	o6 = omega([0, 1, 0])
	S_6 = np.vstack((np.hstack((o6, np.array([np.cross([0, -1, 0], ps[5])]).T)), np.zeros(4)))
	S = [S_1,S_2,S_3,S_4,S_5,S_6]
	T = M
	# print('M: ', M)
	for i in range(6):
			T = expm(S[5-i]*theta[5-i])@T
	# print(T)
	return T

def Compare(T1, T2):
	s = 0
	for i in range(4):
		for j in range(4):
			s+= (T1[i][j]-T2[i][j])*(T1[i][j]-T2[i][j])
	print(s)
	return s

def Shoot():
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, np.pi/8, vrep.simx_opmode_oneshot)
	
	time.sleep(0.2)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, np.pi/8, vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, -np.pi/8, vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, -np.pi*2/3, vrep.simx_opmode_oneshot)
	time.sleep(0.14)
	vrep.simxSetIntegerSignal(
		clientID, 'suc_ON', 0, vrep.simx_opmode_oneshot)
	
	#vrep.simxSetJointTargetPosition(clientID, joint_two_handle, -np.pi/2, vrep.simx_opmode_oneshot)
	
	
	time.sleep(1)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, 0, vrep.simx_opmode_blocking)


# Function that reads suction pad position and orientation
def GetSuctionPosition():	
	result, position = vrep.simxGetObjectPosition(clientID, suction, car, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get suction position')
	result, orientation = vrep.simxGetObjectOrientation(clientID, suction, car, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get suction orientation')
	#print(position)
	#print(orientation)
	
	R = RotMat(orientation)
	p = np.array([position])
	M = np.vstack((np.hstack((R, p.transpose())), [0, 0, 0, 1]))
	print(M)
	return M

# Function that reads basket position
def GetBasketPosition():
	result, position = vrep.simxGetObjectPosition(clientID, basket, -1, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get basket position')
	return position

# Function that reads suction pad position relative to world
def GetSuctionAbsPosition():
	result, position = vrep.simxGetObjectPosition(clientID, suction, -1, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get suction position')
	return position

# Turn off the suction pad
def SetSuctionPadOff():
	vrep.simxSetIntegerSignal(
		clientID, 'suc_ON', 0, vrep.simx_opmode_blocking)
	time.sleep(0.5)

# Turn on the suction pad
def SetSuctionPadOn():
	vrep.simxSetIntegerSignal(
		clientID, 'suc_ON', 1, vrep.simx_opmode_blocking)
	time.sleep(0.5)
	
# Function that used to read joint angles
def GetJointAngle():
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 1 joint variable')
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 2 joint variable')
	result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 3 joint variable')
	result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 4 joint variable')
	result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 5 joint variable')
	result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 6 joint variable')
	theta = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	#print(theta)
	return theta

def move_base(l):
	vrep.simxSetJointTargetVelocity(
		clientID, wheel1, l[0]*3.34934966643019, vrep.simx_opmode_blocking)
	vrep.simxSetJointTargetVelocity(
		clientID, wheel2, l[1]*3.34934966643019, vrep.simx_opmode_blocking)
	vrep.simxSetJointTargetVelocity(
		clientID, wheel3, l[2]*3.34934966643019, vrep.simx_opmode_blocking)
	vrep.simxSetJointTargetVelocity(
		clientID, wheel4, l[3]*3.34934966643019, vrep.simx_opmode_blocking)

def dir_base(dir):
	if dir == 1:
		move_base([1, 1, -1, -1]) 
	elif dir == 2:
		move_base([-1, -1, 1, 1])
	elif dir == 3:
		move_base([1, -1, -1, 1])
	elif dir == 4:
		move_base([-1, 1, 1, -1])
	else:
		move_base([0, 0, 0, 0])



# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

'''
# Print object name list
result,joint_name,intData,floatData,stringData = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking)
print(stringData)
'''

# Get "handle" to the base of robot
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')
    
# Get "handle" to the all joints of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')
# Get "handle" to the end-effector of robot
result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')
result, suction = vrep.simxGetObjectHandle(clientID, 'suctionPad#0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for suction pad')
result, suction2 = vrep.simxGetObjectHandle(clientID, 'suctionPad', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for suction pad2')
result, car = vrep.simxGetObjectHandle(clientID, 'OmniPlatform', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for car')
result, wheel1 = vrep.simxGetObjectHandle(
	clientID, 'OmniWheel_regularRotation', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for whelel')
result, wheel2 = vrep.simxGetObjectHandle(
	clientID, 'OmniWheel_regularRotation#0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for whelel')
result, wheel3 = vrep.simxGetObjectHandle(
	clientID, 'OmniWheel_regularRotation#1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for whelel')
result, wheel4 = vrep.simxGetObjectHandle(
	clientID, 'OmniWheel_regularRotation#2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for whelel')
result, basket = vrep.simxGetObjectHandle(
	clientID, 'Basket', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for basket')

# ==================================================================================================== #

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #

SetJointPosition(np.array([0,0,0,0,0,0]))
result,p1 = vrep.simxGetObjectPosition(clientID, joint_one_handle, car, vrep.simx_opmode_blocking)
result,p2 = vrep.simxGetObjectPosition(clientID, joint_two_handle, car,vrep.simx_opmode_blocking)
result,p3 = vrep.simxGetObjectPosition(clientID, joint_three_handle, car,vrep.simx_opmode_blocking)
result,p4 = vrep.simxGetObjectPosition(clientID, joint_four_handle, car,vrep.simx_opmode_blocking)
result,p5 = vrep.simxGetObjectPosition(clientID, joint_five_handle, car,vrep.simx_opmode_blocking)
result,p6 = vrep.simxGetObjectPosition(clientID, joint_six_handle, car,vrep.simx_opmode_blocking)
result,r1 = vrep.simxGetObjectOrientation(clientID, joint_one_handle, car, vrep.simx_opmode_blocking)
result,r2 = vrep.simxGetObjectOrientation(clientID, joint_two_handle, car,vrep.simx_opmode_blocking)
result,r3 = vrep.simxGetObjectOrientation(clientID, joint_three_handle, car,vrep.simx_opmode_blocking)
result,r4 = vrep.simxGetObjectOrientation(clientID, joint_four_handle, car,vrep.simx_opmode_blocking)
result,r5 = vrep.simxGetObjectOrientation(clientID, joint_five_handle, car,vrep.simx_opmode_blocking)
result,r6 = vrep.simxGetObjectOrientation(clientID, joint_six_handle, car,vrep.simx_opmode_blocking)
#print([p1,p2,p3,p4,p5,p6])
#print([r1,r2,r3,r4,r5,r6])

#print(vrep.simxGetObjectPosition(clientID, suction, car, vrep.simx_opmode_blocking))
#print(vrep.simxGetObjectOrientation(clientID, suction, car, vrep.simx_opmode_blocking))



time.sleep(1)
print('start moving')
# Goal_joint_angles = np.array([[0,0,-0.5*np.pi,0.5*np.pi,-0.5*np.pi,-0.5*np.pi], \
# 							[0.5*np.pi,0,-0.5*np.pi,0.5*np.pi,0.5*np.pi,-0.5*np.pi],\
# 							[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,-0.5*np.pi],\
# 							[0,0,0,0,0,-0.5*np.pi], \
# 							[0.5*np.pi,0,-0.5*np.pi,0.5*np.pi,0.5*np.pi,-0.5*np.pi],\
# 							[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,-0.5*np.pi]])
#Goal_joint_angles = np.array([[-0.5*np.pi,0,0,0,0,0], \
#							[0,-0.5*np.pi,0,0,0,0], \
#							[0,0,-0.5*np.pi,0,0,0], \
#							[0,0,0,0.5*np.pi,0,0], \
#							[0,0,0,0,0.5*np.pi,0], \
#							[0,0,0,0,0,-0.5*np.pi]])



#GetJointAngle()
## get absolute position of the ball
ball_pos = [GetballGPS_x(), GetballGPS_y(), GetballGPS_z()]
print(ball_pos)
## move the mobile dir_base(direction) direction: 1= +x, 2= -x, 3= +y, 4= -y 
# dir_base(3)
# time.sleep(3)
# dir_base(1)


SetJointPosition(np.array([0,0,0,0.5*np.pi,0,0]))
while GetSuctionPad() == 0:
	time.sleep(0.5)	
time.sleep(0.5)
T1 = GetSuctionPosition()
T2 = CalcSuctionPosition()

# Wait two seconds
time.sleep(1)

SetJointPosition(np.array([0,0,0,0,-0.5*np.pi,0]))
time.sleep(1)
GetSuctionPosition()
CalcSuctionPosition()
ball_pos = [GetballGPS_x(), GetballGPS_y(), GetballGPS_z()]


result,j2pos=vrep.simxGetObjectPosition(clientID, joint_two_handle,-1,vrep.simx_opmode_blocking)
z = ball_pos[2]-j2pos[2]-0.083

t2 = np.arccos((z*z+0.244*0.244-0.213*0.213)/(2*z*0.244))
t3 = np.arccos((-z**2+0.244**2+0.213**2)/(2*0.213*0.244))
t4 = np.arccos((z**2-0.244**2+0.213**2)/(2*z*0.213))
print([t2,t3,t4])
SetJointPosition(np.array([0,-t2,np.pi-t3,-t4,-0.5*np.pi,0]))
suc_pos = GetSuctionAbsPosition()
while np.abs(ball_pos[1]-suc_pos[1]-0.130)>0.005 or np.abs(ball_pos[0]-suc_pos[0])>0.005:
	ball_pos = [GetballGPS_x(), GetballGPS_y(), GetballGPS_z()]
	suc_pos = GetSuctionAbsPosition()

	

	#print(ball_pos[0]-suc_pos[0])
	if(ball_pos[0]-suc_pos[0] > 0.01):
		dir_base(2)
		continue
	if(ball_pos[0]-suc_pos[0] <-0.01):
		dir_base(1)
		continue
	if(ball_pos[1]-suc_pos[1] > 0.131):
		dir_base(3)
		continue
	if(ball_pos[1]-suc_pos[1] <0.130):
		dir_base(4)
		continue
	dir_base(0)
	
	time.sleep(0.1)
time.sleep(1)
vrep.simxSetIntegerSignal(
		clientID, 'suc_ON2', 0, vrep.simx_opmode_blocking)
time.sleep(0.5)
SetJointPosition(np.array([np.pi,-t2,np.pi-t3,-t4,-0.5*np.pi,0]))
ball_pos = GetBasketPosition()
while np.abs(ball_pos[1]-suc_pos[1]+0.61)>0.005 or np.abs(ball_pos[0]-suc_pos[0])>0.005:
	
	suc_pos = GetSuctionAbsPosition()

	

	#print(ball_pos[1]-suc_pos[1])
	if(ball_pos[0]-suc_pos[0] > 0.01):
		dir_base(2)
		continue
	if(ball_pos[0]-suc_pos[0] <-0.01):
		dir_base(1)
		continue
	if(ball_pos[1]-suc_pos[1] > -0.611):
		dir_base(3)
		continue
	if(ball_pos[1]-suc_pos[1] <-0.610):
		dir_base(4)
		continue
	dir_base(0)
	
	time.sleep(0.1)
dir_base(0)
SetJointPosition(np.array([np.pi,t2,np.pi-t3,-t4,-0.5*np.pi,0]))
vrep.simxSetIntegerSignal(
		clientID, 'suc_ON', 0, vrep.simx_opmode_blocking)
time.sleep(0.5)
time.sleep(10)

# **************************************************************************************************** #

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
