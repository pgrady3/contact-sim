import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
import scipy.io
import time

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

softId = p.loadSoftBody("/home/patrick/contact/bullet3/data/tube.vtk", [0, 0, 0], mass=1, useNeoHookean = 0, NeoHookeanMu = 60, NeoHookeanLambda = 200, 
                      NeoHookeanDamping = 0.01, useSelfCollision = 0, frictionCoeff = 0.5, 
                      springElasticStiffness=0.5, springDampingStiffness=0.5, springBendingStiffness=0.5, 
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.01)

cubeStartPos = [-0.7, 1.6, 1]
cubeStartOrientation = p.getQuaternionFromEuler([1, 0, 1.5])
#botId = p.loadURDF("biped/biped2d_pybullet.urdf", cubeStartPos, cubeStartOrientation)
#botId = p.loadURDF("humanoid.urdf", cubeStartPos, cubeStartOrientation)
#botId = p.loadURDF("/home/patrick/soft/GraspIt2URDF/urdf/HumanHand20DOF.urdf", cubeStartPos, cubeStartOrientation, globalScaling=5)
botId = p.loadURDF("/home/patrick/contact/contact-sim/urdf/hand.urdf", cubeStartPos, cubeStartOrientation, globalScaling=18)

velo_joint = [0, 1, 1, 1, 0, # Pinky 
              0, 1, 1, 1, 0, # Middle
              0, 1, 1, 1, 0, # Index
              3, 0, 0, 1, 0] # Thumb

velo_joint = np.ones(15)

for i in range(0, p.getNumJoints(botId)):
  p.changeDynamics(botId, i, mass=0.1)
  p.changeVisualShape(botId, i, rgbaColor=[0, 0, 1, 0.5])


p.changeDynamics(botId, -1, mass=0)
p.changeVisualShape(botId, -1, rgbaColor=[0, 0, 1, 0.5])
#p.changeDynamics(softId, -1, mass=1000)
#p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME)

#p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.025)
p.setRealTimeSimulation(0)
p.setGravity(0, 0, 10)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# debug_lines = []
# for i in range(200):
#     line_id = p.addUserDebugLine([0,0,0], [0,0,0])
#     debug_lines.append(line_id)

def save_model(filename):
    print('Saving')

    softPos, softRot = p.getBasePositionAndOrientation(softId)
    x0, y0, z0, x1, y1, z1, x2, y2, z2, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)
    contact_pt = np.stack((contX, contY, contZ)).T
    contact_force = np.stack((contForceX, contForceY, contForceZ)).T
    save_dict = {'pos':softPos, 'rot':softRot, 'x0':x0,'y0':y0,'z0':z0,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2, 'contX':contX, 'contY':contY, 'contZ':contZ, 'contForceX':contForceX, 'contForceY':contForceY, 'contForceZ':contForceZ}
    scipy.io.savemat(filename, save_dict)

sim_count = 0
last_time = time.time()

while p.isConnected():
  #p.getSoftBodyData(softId)


  # for i in range(len(debug_lines)):
  #   if i < len(contX):
  #     pt = contact_pt[i, :]
  #     #pt[1] += 2
  #     debug_lines[i] = p.addUserDebugLine(pt, pt + contact_force[i, :]*0.01, lineWidth=5, replaceItemUniqueId=debug_lines[i])
  #   else:

  #     debug_lines[i] = p.addUserDebugLine([0,0,0], [0,0,0], replaceItemUniqueId=debug_lines[i])

  botPos, botOrn = p.getBasePositionAndOrientation(botId)

  if sim_count < 200:
    p.applyExternalForce(botId, -1, [5, 0, -50], botPos, flags=p.WORLD_FRAME)

  if sim_count == 200:
    for i in range(0, p.getNumJoints(botId)):
      p.setJointMotorControl2(bodyUniqueId=botId, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetVelocity = velo_joint[i]*0.5, force = 10)

  save_key = ord('z')
  keys = p.getKeyboardEvents()
  if save_key in keys and keys[save_key]&p.KEY_WAS_TRIGGERED:
    save_model('deformed.mat')

  if sim_count == 2:
    save_model('orig.mat')


  p.stepSimulation()
  #sleep(1./240.)
  sim_count += 1
  print('Sim step takes', time.time() - last_time)
  last_time = time.time()