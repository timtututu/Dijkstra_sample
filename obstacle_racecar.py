#controller Implementing motor control:
import pybullet as p
import pybullet_data
import time
import os, inspect
import numpy as np
import random 
from Astar import AStar
import parameters as para

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")

os.sys.path.insert(0, parentdir)

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)

useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
#p.loadURDF("plane.urdf")
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))


or_locx=para.START_POS
or_locy=para.END_POS
#loc&obstacle arrray
'''x=[];y=[];obstacle=[]'''

'''for i in range(5):
  loc_x=random.uniform(-5,5)
  loc_y=random.uniform(-5,5)
  x.append(loc_x)
  y.append(loc_y)

  O1StartPos= [x[i],y[i],0]
  O1Orientation = p.getQuaternionFromEuler([0,0,0])
  obstacle.append(p.loadURDF("/home/timtu/桌面/summer program/urdf/cylinder.urdf",O1StartPos, O1Orientation,useFixedBase=1))'''

# for double obstacles test
or_car_pos=[or_locx,or_locy,0]
O1StartPos= [2,1,0]
O2StartPos= [2,2,0]
O3StartPos= [2,3,0]
O1Orientation = p.getQuaternionFromEuler([0,0,0])
O2Orientation = p.getQuaternionFromEuler([0,0,0])
O3Orientation = p.getQuaternionFromEuler([0,0,0])
obstacle=p.loadURDF("/home/timtu/桌面/summer program/urdf/cylinder.urdf",O1StartPos, O1Orientation,useFixedBase=1)
obstacle2=p.loadURDF("/home/timtu/桌面/summer program/urdf/cylinder.urdf",O2StartPos, O2Orientation,useFixedBase=1)
obstacle3=p.loadURDF("/home/timtu/桌面/summer program/urdf/cylinder.urdf",O3StartPos, O2Orientation,useFixedBase=1)


car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))
#for i in range(p.getNumJoints(car)):
  #print(p.getJointInfo(car, i))

# ##test pos
# postemp=p.getBasePositionAndOrientation(car)
# print('\n')
# print(postemp)
# print('\n')
# ##

inactive_wheels = [3, 5, 7]
wheels = [2]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)
goal = [10,10]


while (True):
  maxForce = p.readUserDebugParameter(maxForceSlider)
  #targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
  #steeringAngle = p.readUserDebugParameter(steeringSlider)

  carpos, caror = p.getBasePositionAndOrientation(car) #get position and orientation
  angulo = p.getEulerFromQuaternion(caror)
  yaw = angulo[2]

  #calculating the distance,angle and velocities 
  distance = np.sqrt((goal[0]-carpos[0])**2 + (goal[1]- carpos[1])**2) #distance = sqrt((x2-x1)^2 + (y2-y1^2))
  t1 =(goal[1]- carpos[1])
  t2 = (goal[0]-carpos[0])
  theta = np.arctan((goal[1]- carpos[1]) / (goal[0]-carpos[0]))        #angle = arctan ((y2-y1)/(x2-x1))
  
  max_sa = .5 #maximum sterring angle 
  #stablishing that the maximum input to the angle is between (-.5 and .5)
  vel = 3.5*distance
  gamma = theta - yaw #the actual angle we need to input
  #lim_angle = max(min(theta, max_sa), -max_sa)


  for wheel in wheels:
    p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=vel,
                            force=maxForce)

  for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=gamma)

  steering
  if (useRealTimeSim == 0):
    p.stepSimulation()
  time.sleep(0.01)



