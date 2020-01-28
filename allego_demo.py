import pybullet as p
import pybullet_data
from time import sleep
import numpy as np

# Joint description
# J0 pinky rotate
# J1 pinky base
# J2 pinky knuckle
# J3 pinky tip. All tendons are inter-reliant
# J4 nothing
# J5 middle rotate
# J6 middle base
# J7 middle knuckle
# J8 middle tip
# J9 nothing
# J10 index rotate
# J11 index base
# J12 index knuckle
# J13 index tip
# J14 nothing
# J15 thumb base in
# J16 thumb rotate
# J17 thumb rotate base towards index
# J18 thumb knuckle
# J19 nothing


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


cubeStartPos = [-0.1, 0, 0.6]
cubeStartOrientation = p.getQuaternionFromEuler([0, 1, 0])
botId = p.loadURDF("/home/patrick/contact/allegro_hand_ros/allegro_hand_description/allegro_hand_description_left.urdf", cubeStartPos, cubeStartOrientation, globalScaling=8)

p.changeDynamics(botId, -1, mass=0.1)

for i in range(0, p.getNumJoints(botId)):
  p.changeDynamics(botId, i, mass=0.1)
  print(i, p.getJointInfo(botId, i))

p.changeDynamics(botId, -1, mass=0.1)

slider_joint_id = p.addUserDebugParameter("Joint moving", 0, p.getNumJoints(botId), 0)
slider_velo_id = p.addUserDebugParameter("Joint velocity", -1, 1, 0)
p.setRealTimeSimulation(0)

sim_count = 0
while p.isConnected():
  slider_joint_val = round(p.readUserDebugParameter(slider_joint_id))
  slider_velo_val = p.readUserDebugParameter(slider_velo_id)
  p.setJointMotorControl2(bodyUniqueId=botId, jointIndex=slider_joint_val, controlMode=p.VELOCITY_CONTROL, targetVelocity = slider_velo_val, force = 100)

  p.stepSimulation()
  sleep(1./240.)
  sim_count += 1