import pybullet as p
import pybullet_data
from time import sleep
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

softId = p.loadSoftBody("/home/patrick/contact/bullet3/data/tube.vtk", [0, 0, 0], mass=1000, useNeoHookean = 0, NeoHookeanMu = 60, NeoHookeanLambda = 200, 
                      NeoHookeanDamping = 0.01, useSelfCollision = 0, frictionCoeff = 0.5, 
                      springElasticStiffness=50, springDampingStiffness=5, springBendingStiffness=5, 
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.05)


cubeStartPos = [-0.1, 0, 0.6]
cubeStartOrientation = p.getQuaternionFromEuler([0, 1, 0])
#botId = p.loadURDF("biped/biped2d_pybullet.urdf", cubeStartPos, cubeStartOrientation)
#botId = p.loadURDF("humanoid.urdf", cubeStartPos, cubeStartOrientation)
#botId = p.loadURDF("/home/patrick/soft/GraspIt2URDF/urdf/HumanHand20DOF.urdf", cubeStartPos, cubeStartOrientation, globalScaling=5)
botId = p.loadURDF("/home/patrick/contact/allegro_hand_ros/allegro_hand_description/allegro_hand_description_left.urdf", cubeStartPos, cubeStartOrientation, globalScaling=8)

p.changeDynamics(botId, -1, mass=0.1)

velo_joint = [0, 1, 1, 1, 0, # Pinky 
              0, 1, 1, 1, 0, # Middle
              0, 1, 1, 1, 0, # Index
              1, 0, 0, 1, 0] # Thumb

for i in range(0, p.getNumJoints(botId)):
  p.changeDynamics(botId, i, mass=0.1)
  p.changeVisualShape(botId, i, rgbaColor=[0, 0, 1, 0.5])
  p.setJointMotorControl2(bodyUniqueId=botId, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetVelocity = velo_joint[i], force = 100)
  print(i, p.getJointInfo(botId, i))


p.changeDynamics(botId, -1, mass=0.1)
p.changeVisualShape(botId, -1, rgbaColor=[0, 0, 1, 0.5])
#p.changeDynamics(softId, -1, mass=1000)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

debug_lines = []
for i in range(200):
    line_id = p.addUserDebugLine([0,0,0], [0,0,0])
    debug_lines.append(line_id)

sim_count = 0
while p.isConnected():
  x, y, z, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)
  contact_pt = np.stack((contX, contY, contZ)).T
  contact_force = np.stack((contForceX, contForceY, contForceZ)).T

  for i in range(len(debug_lines)):
    if i < len(contX):
      debug_lines[i] = p.addUserDebugLine(contact_pt[i, :], contact_pt[i, :] + contact_force[i, :], lineWidth=3, replaceItemUniqueId=debug_lines[i])
    else:

      debug_lines[i] = p.addUserDebugLine([0,0,0], [0,0,0], replaceItemUniqueId=debug_lines[i])


  botPos, botOrn = p.getBasePositionAndOrientation(botId)
  if sim_count < 200:
    p.applyExternalForce(botId, -1, [5, 0, -50], botPos, flags=p.WORLD_FRAME)
  p.stepSimulation()
  sleep(1./240.)
  sim_count += 1