import numpy as np
from webuser.smpl_handpca_wrapper_HAND_only import load_model
import platform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import mode
import trimesh
import mesh_repair
import os
from tf import transformations as trans
print('cur working dir: ', os.getcwd())

def T2xyzrpy(T):
  tx, ty, tz = T[:3, 3]
  R = np.eye(4)
  R[:3, :3] = T[:3, :3]
  rx, ry, rz = trans.euler_from_matrix(R)
  #return np.array([tx, ty, tz, rx, ry, rz])
  return 'xyz="{:.4f} {:.4f} {:.4f}" rpy="{:.4f} {:.4f} {:.4f}"'.format(tx, ty, tz, rx, ry, rz)

# Load MANO model (here we load the right hand model)
try:
  m = load_model('../mano/models/MANO_RIGHT.pkl', ncomps=6, flat_hand_mean=False)
except:
  m = load_model('../models/MANO_RIGHT.pkl', ncomps=6, flat_hand_mean=False)

# Assign random pose and shape parameters
m.betas[:] = np.random.rand(m.betas.size) * .03
#m.pose[:] = np.random.rand(m.pose.size) * .2
m.pose[:3] = [0., 0., 0.]
m.pose[3:] = [-0.42671473, -0.85829819, -0.50662164, +1.97374622, -0.84298473, -1.29958491]
m.pose[0] = np.pi

vertices = m.r # Get vertex 3D pos
faces=np.array(m.f) # Get face to vertex mapping
vertex_to_joint = np.argmax(m.weights, axis=1) # Foreach vertex, find the joint with the strongest LBS value
face_node_to_joint = vertex_to_joint[faces.flatten()].reshape(faces.shape) # Foreach face node, find corresponding joint
face_to_joint, _ = mode(face_node_to_joint, axis=1) # Foreach face, find strongest joint

joints3D = np.array(m.J_transformed).reshape((-1, 3))

np.set_printoptions(precision=4, suppress=True)

parent_joints = m.kintree_table[0, :]


tform_relative = np.zeros((16, 4, 4))
tform_relative[0, :, :] = m.A_global[0] # Fill with original tform

joints_cum_tform = np.zeros((16, 4, 4))
joints_cum_tform[0, :, :] = m.A_global[0] # Make homogeneous

for i in range(1, 16):
  old_tform = m.A_global[parent_joints[i]]
  new_tform = np.array(m.A_global[i])
  inv_old = np.linalg.inv(old_tform)
  rel_tform = np.dot(inv_old, new_tform)
  tform_relative[i, :, :] = rel_tform


for i in range(1, 16):
  new_tform = tform_relative[i, :, :]
  old_tform = joints_cum_tform[parent_joints[i], :, :]
  joints_cum_tform[i, :, :] = np.dot(old_tform, new_tform)

jointsPos = joints_cum_tform[:, 0:3, 3] # Get translation element from transform

# trimesh.util.attach_to_log() # Gets verbose trimesh output
# 0 is palm, 1-3 index, 4-6 middle, 7-9 ring, 10-12 pinky, 13-15 thumb

for idx in set(vertex_to_joint):
#for idx in [3, 6, 9, 12, 15]: # Fingertips only
  print('Joint, fwd, invglo', idx)
  print(T2xyzrpy(tform_relative[idx, :, :]))
  #print('inv XYZ rpy', T2xyzrpy(np.linalg.inv(tform_relative[idx, :, :])))
  #print('a   XYZ rpy', T2xyzrpy(np.array(m.A[:, :, idx])))
  #print('glo XYZ rpy', T2xyzrpy(np.array(m.A_global[idx])))
  print(T2xyzrpy(np.linalg.inv(m.A_global[idx])))
  #print('cum XYZ rpy', T2xyzrpy(joints_cum_tform[idx, :, :]))

  face_mask = face_to_joint == idx
  faces_in_joint = np.where(face_mask)[0]
  joint_faces = faces[faces_in_joint, :]

  mesh = trimesh.Trimesh(vertices=vertices, faces=joint_faces)
  wtight = mesh_repair.fill_holes(mesh)
  #print("Trying to fix. Is now watertight:", wtight)
  #mesh.show()
  #mesh.export('mesh/joint_' + str(idx) + '.stl')

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  #ax.scatter(joints3D[:, 0], joints3D[:, 1], joints3D[:, 2], color='b') # Plot joints

  ax.scatter(jointsPos[:, 0], jointsPos[:, 1], jointsPos[:, 2], color='r') # Plot joints with my method
  ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=joint_faces, linewidth=0.2, antialiased=True)
  ax.set_aspect('equal', 'box')

  #plt.show()

