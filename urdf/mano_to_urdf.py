import numpy as np
from webuser.smpl_handpca_wrapper_HAND_only import load_model
import platform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import mode
import trimesh
import mesh_repair
import os
from scipy.spatial.transform import Rotation as Rot

print('cur working dir: ', os.getcwd())

def T2xyzrpy(T):
  tx, ty, tz = T[:3, 3]
  r = Rot.from_dcm(T[:3, :3])
  rx, ry, rz = r.as_euler('xyz')
  return 'xyz="{:.4f} {:.4f} {:.4f}" rpy="{:.4f} {:.4f} {:.4f}"'.format(tx, ty, tz, rx, ry, rz)

# Load MANO model (here we load the right hand model)
m = load_model('../../mano/models/MANO_RIGHT.pkl', ncomps=6, flat_hand_mean=False)

# Assign random pose and shape parameters
m.betas[:] = np.random.rand(m.betas.size) * .03
#m.pose[:] = np.random.rand(m.pose.size) * .2
m.pose[:3] = [0., 0., 0.]
#m.pose[3:] = [-0.42671473, -0.85829819, -0.50662164, +1.97374622, -0.84298473, -1.29958491]
#m.pose[0] = np.pi

vertices = m.r # Get vertex 3D pos
faces=np.array(m.f) # Get face to vertex mapping
vertex_to_joint = np.argmax(m.weights, axis=1) # Foreach vertex, find the joint with the strongest LBS value
face_node_to_joint = vertex_to_joint[faces.flatten()].reshape(faces.shape) # Foreach face node, find corresponding joint
face_to_joint, _ = mode(face_node_to_joint, axis=1) # Foreach face, find strongest joint

joints3D = np.array(m.J_transformed).reshape((-1, 3))
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

lookup_dict = {}
np.set_printoptions(precision=4, suppress=True)

# 0 is palm, 1-3 index, 4-6 middle, 7-9 ring, 10-12 pinky, 13-15 thumb
for idx in set(vertex_to_joint):
  print('Joint, fwd, invglo', idx)
  #print(T2xyzrpy(tform_relative[idx, :, :]))
  #print('inv XYZ rpy', T2xyzrpy(np.linalg.inv(tform_relative[idx, :, :])))
  #print('a   XYZ rpy', T2xyzrpy(np.array(m.A[:, :, idx])))
  #print('glo XYZ rpy', T2xyzrpy(np.array(m.A_global[idx])))
  #print(T2xyzrpy(np.linalg.inv(m.A_global[idx])))
  #print('cum XYZ rpy', T2xyzrpy(joints_cum_tform[idx, :, :]))
  prefix = '$J{}_'.format(idx)
  lookup_dict[prefix+'INVGLO'] = T2xyzrpy(np.linalg.inv(m.A_global[idx]))
  lookup_dict[prefix+'FWDREL'] = T2xyzrpy(tform_relative[idx, :, :])

  face_mask = face_to_joint == idx
  faces_in_joint = np.where(face_mask)[0]
  joint_faces = faces[faces_in_joint, :]

  mesh = trimesh.Trimesh(vertices=vertices, faces=joint_faces)
  wtight = mesh_repair.fill_holes(mesh)
  #print("Trying to fix. Is now watertight:", wtight)
  #mesh.show()
  joint_file = os.path.abspath('mesh/joint_' + str(idx) + '.stl')
  mesh.export(joint_file)
  #lookup_dict[prefix+'FILE'] = 'file://' + joint_file
  lookup_dict[prefix+'FILE'] = joint_file

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(jointsPos[:, 0], jointsPos[:, 1], jointsPos[:, 2], color='r') # Plot joints with my method
  ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=joint_faces, linewidth=0.2, antialiased=True)
  ax.set_aspect('equal', 'box')

  #plt.show()

out_file = []

with open('hand_template.urdf') as f:
  lines = f.readlines()
  for line in lines:
    new_line = line
    for key, val in lookup_dict.items():
      if key in new_line:
        find_idx = new_line.find(key)
        new_line = new_line[:find_idx] + val + new_line[find_idx + len(key):]
    out_file.append(new_line)

with open('hand.urdf', 'w') as f:
  for line in out_file:
    f.write(line)

print('Done!')