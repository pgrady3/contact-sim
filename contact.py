import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
import scipy.io
import time
import argparse
import ast
import matplotlib.pyplot as plt
from PIL import Image

def parse_args():
  parser = argparse.ArgumentParser(description='Contact simulation')
  parser.add_argument('--file', required=True, type=str, help='File name of object to grasp')
  parser.add_argument('--nogui', action='store_true', help='Disable GUI')
  parser.add_argument('--pos', default="[0,0,0]", type=str, help='Starting position of hand')
  parser.add_argument('--euler', default="[0,0,0]", type=str, help='Starting euler rotation of hand')
  parser.add_argument('--iter', default=1000, type=int, help='Number of iterations before termination')
  parser.add_argument('--force', default=10, type=float, help='Hand closing force')
  parser.add_argument('--vec', action='store_true', help='Draw force vectors')
  parser.add_argument('--outfile', default='test', type=str, help='Output filename')
  parser.add_argument('--allegro', action='store_true', help='Use Allegro hand')
  parser.add_argument('--elastic', default=50, type=float, help='Spring Elastic Stiffness')
  parser.add_argument('--damping', default=5, type=float, help='Spring Damping Stiffness')
  parser.add_argument('--bending', default=5, type=float, help='Spring Bending Stiffness')
  
  args = parser.parse_args()

  args.pos = ast.literal_eval(args.pos) # Parse into list
  args.euler = ast.literal_eval(args.euler)

  return args

def save_model(filename, softId):
  print('Saving', filename)

  softPos, softRot = p.getBasePositionAndOrientation(softId)
  x0, y0, z0, x1, y1, z1, x2, y2, z2, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)
  save_dict = {'pos':softPos, 'rot':softRot, 'x0':x0,'y0':y0,'z0':z0,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2, 'contX':contX, 'contY':contY, 'contZ':contZ, 'contForceX':contForceX, 'contForceY':contForceY, 'contForceZ':contForceZ}
  scipy.io.savemat(filename, save_dict)

def gen_img(view_matrix, im_width, im_height):
  far = 9.1
  near = 0.1
  projectionMatrix = p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=near, farVal=far)

  width, height, rgbImg, depthImg, segImg = p.getCameraImage(width=im_width, height=im_height, viewMatrix=view_matrix, projectionMatrix=projectionMatrix, 
                                                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
  rgb_img = np.reshape(rgbImg, (height, width, 4))
  rgb_img = rgb_img * (1. / 255.)
  depth_buffer = np.reshape(depthImg, [width, height])
  depth_img = far * near / (far - (far - near) * depth_buffer)
  seg_img = np.reshape(segImg, [width, height]) * 1. / 255.

  return rgb_img

def save_img(filename, im_width=500, im_height=500):
  view_matrix_1 = p.computeViewMatrix(cameraEyePosition=[-3, 3, 3], cameraTargetPosition=[0, 0, 0], cameraUpVector=[0, 0, 1])
  rgb_1 = gen_img(view_matrix_1, im_width, im_height)
  view_matrix_2 = p.computeViewMatrix(cameraEyePosition=[0, -4, 1], cameraTargetPosition=[0, 0, 0], cameraUpVector=[0, 0, 1])
  rgb_2 = gen_img(view_matrix_2, im_width, im_height)
  view_matrix_3 = p.computeViewMatrix(cameraEyePosition=[-3, 3, -1], cameraTargetPosition=[0, 0, 0], cameraUpVector=[0, 0, 1])
  rgb_3 = gen_img(view_matrix_3, im_width, im_height)

  rgb_all = np.concatenate((rgb_1, rgb_2, rgb_3), axis=1)
  # plt.imshow(rgb_all)
  # plt.axis('off')
  # plt.show()

  rgb_save = rgb_all[:,:,:3] * 255.
  pil_im = Image.fromarray(rgb_save.astype(np.uint8))
  pil_im.save(args.outfile + '_vis.png')


args = parse_args()

if args.allegro:
  initial_velo = [0, 1, 1, 1, 0, # Pinky 
              0, 1, 1, 1, 0, # Middle
              0, 1, 1, 1, 0, # Index
              3, 0, 0, 1, 0] # Thumb

  velo_joint = [0, 1, 1, 1, 0, # Pinky 
              0, 1, 1, 1, 0, # Middle
              0, 1, 1, 1, 0, # Index
              3, 0, 0, 1, 0] # Thumb
else:
  initial_velo = [0, 0, 0, #index
                0, 0, 0, #middle
                0, 0, 0, #ring
                0, 0, 0, #pinky
                -1, -0.5, -0.3] #thumb

  velo_joint = [3, 0.5, 0.3, #index
                3, 0.5, 0.3, #middle
                3, 0.5, 0.3, #ring
                3, 0.5, 0.3, #pinky
                3, 0.5, 0.3] #thumb


physicsClient = p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.025)
p.setRealTimeSimulation(0)
p.setGravity(0, 0, 10)

softId = p.loadSoftBody(args.file, [0, 0, 0], mass=1,
                      useNeoHookean = 0, NeoHookeanMu = 60, NeoHookeanLambda = 200, NeoHookeanDamping = 0.01,
                      useSelfCollision = 0,
                      frictionCoeff = 0.5, 
                      springElasticStiffness=50, springDampingStiffness=5, springBendingStiffness=5, 
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.005)
p.changeVisualShape(softId, 1, rgbaColor=[0, 0, 1, 1.0])

if args.allegro:
  new_euler = [args.euler[0] - 1.57, 1.57 - args.euler[1], args.euler[2] + 3.1415]
  new_pos = np.array(args.pos) + [-0.5,0,0]
  handStartOrientation = p.getQuaternionFromEuler(new_euler)
  handId = p.loadURDF("/home/patrick/contact/allegro_hand_ros/allegro_hand_description/allegro_hand_description_left.urdf", new_pos, handStartOrientation, globalScaling=10)
else:
  handId = p.loadURDF("/home/patrick/contact/contact-sim/urdf/hand.urdf", args.pos, p.getQuaternionFromEuler(args.euler), globalScaling=16)

p.changeDynamics(handId, -1, mass=0)
for i in range(0, p.getNumJoints(handId)):
  p.changeDynamics(handId, i, mass=0.1)
  p.setJointMotorControl2(bodyUniqueId=handId, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetVelocity = initial_velo[i], force = args.force)

if not args.nogui:
  p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

if args.vec:
  debug_lines = []
  for i in range(200):
      line_id = p.addUserDebugLine([0,0,0], [0,0,0])
      debug_lines.append(line_id)

sim_ticks = 0
start_time = time.time()
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
    save_model(args.outfile + '_deformed.mat', softId)

  if sim_ticks == 2:
    save_model(args.outfile + '_orig.mat', softId)

  p.stepSimulation()
  sim_ticks += 1

save_model(args.outfile + '_deformed.mat', softId)
print('Runtime: ', time.time() - start_time, ' sec')
save_img(args.outfile)