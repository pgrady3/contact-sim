import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
import scipy.io
import time
import argparse
import ast

def parse_args():
  parser = argparse.ArgumentParser(description='Contact simulation')
  parser.add_argument('--file', required=True, type=str, help='File name of object to grasp')
  parser.add_argument('--nogui', action='store_true', help='Disable GUI')
  parser.add_argument('--pos', default="[0,0,0]", type=str, help='Starting position of hand')
  parser.add_argument('--euler', default="[0,0,0]", type=str, help='Starting euler rotation of hand')
  parser.add_argument('--iter', default=1000, type=int, help='Number of iterations before termination')
  parser.add_argument('--force', default=10, type=float, help='Hand closing force')
  parser.add_argument('--vec', action='store_true', help='Draw force vectors')
  args = parser.parse_args()

  args.pos = ast.literal_eval(args.pos) # Parse into list
  args.euler = ast.literal_eval(args.euler)

  return args

def save_model(filename, softId):
  print('Saving')

  softPos, softRot = p.getBasePositionAndOrientation(softId)
  x0, y0, z0, x1, y1, z1, x2, y2, z2, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)
  save_dict = {'pos':softPos, 'rot':softRot, 'x0':x0,'y0':y0,'z0':z0,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2, 'contX':contX, 'contY':contY, 'contZ':contZ, 'contForceX':contForceX, 'contForceY':contForceY, 'contForceZ':contForceZ}
  scipy.io.savemat(filename, save_dict)

velo_joint = [3, 0.5, 0.3, #index
              3, 0.5, 0.3, #middle
              3, 0.5, 0.3, #ring
              3, 0.5, 0.3, #pinky
              3, 0.5, 0.3] #thumb

args = parse_args()

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
#p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.025)
p.setRealTimeSimulation(0)
p.setGravity(0, 0, 10)

softId = p.loadSoftBody(args.file, [0, 0, 0], mass=1,
                      useNeoHookean = 0, NeoHookeanMu = 60, NeoHookeanLambda = 200, NeoHookeanDamping = 0.01,
                      useSelfCollision = 0,
                      frictionCoeff = 0.5, 
                      springElasticStiffness=50, springDampingStiffness=5, springBendingStiffness=5, 
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.005)
p.changeVisualShape(softId, 1, rgbaColor=[0, 0, 1, 1.0])

handId = p.loadURDF("/home/patrick/contact/contact-sim/urdf/hand.urdf", args.pos, p.getQuaternionFromEuler(args.euler), globalScaling=16)
p.changeDynamics(handId, -1, mass=0)
for i in range(0, p.getNumJoints(handId)):
  p.changeDynamics(handId, i, mass=0.1)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

if args.vec:
  debug_lines = []
  for i in range(200):
      line_id = p.addUserDebugLine([0,0,0], [0,0,0])
      debug_lines.append(line_id)

sim_ticks = 0
while p.isConnected() and sim_ticks < args.iter:

  if args.vec:
    x0, y0, z0, x1, y1, z1, x2, y2, z2, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)
    contact_pt = np.stack((contX, contY, contZ)).T
    contact_force = np.stack((contForceX, contForceY, contForceZ)).T
    for i in range(len(debug_lines)):
      if i < len(contX):
        pt = contact_pt[i, :]
        debug_lines[i] = p.addUserDebugLine(pt, pt + contact_force[i, :]*0.01, lineWidth=5, replaceItemUniqueId=debug_lines[i])
      else:
        debug_lines[i] = p.addUserDebugLine([0,0,0], [0,0,0], replaceItemUniqueId=debug_lines[i])


  botPos, botOrn = p.getBasePositionAndOrientation(handId)

  if sim_ticks == 200:
    for i in range(0, p.getNumJoints(handId)):
      p.setJointMotorControl2(bodyUniqueId=handId, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetVelocity = velo_joint[i], force = args.force)

  save_key = ord('z')
  keys = p.getKeyboardEvents()
  if save_key in keys and keys[save_key]&p.KEY_WAS_TRIGGERED:
    save_model('deformed.mat', softId)

  if sim_ticks == 2:
    save_model('orig.mat', softId)

  p.stepSimulation()
  sim_ticks += 1