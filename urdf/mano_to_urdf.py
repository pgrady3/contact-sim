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

def normalize(v):
  return np.array(v) / np.linalg.norm(v)


# Load MANO model (here we load the right hand model)
m = load_model('../../mano/models/MANO_RIGHT.pkl', ncomps=6, flat_hand_mean=True)
np.random.seed() # load_model sets a fixed seed, so reset it

m.betas[:] = np.random.rand(m.betas.size) * .0 # Assign random shape parameters 
m.pose[:] = np.random.normal(size=m.pose.size, scale=1.0) * 0
m.pose[:3] = [0., 0., 0.] # Set palm rotation to zero

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

joint_vecs = np.zeros((15, 3))
for i in range(0, 15):
  start_idx = i * 3
  end_idx = start_idx + 3
  joint_vecs[i, :] = normalize(m.hands_components[0, start_idx:end_idx])

joint_vecs[13, :] = -joint_vecs[13, :] #this joint is backwards
joint_vecs *= -1  

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=faces, linewidth=0.2, antialiased=True)
ax.set_aspect('equal', 'box')
ax.quiver(joints3D[1:, 0], joints3D[1:, 1], joints3D[1:, 2], joint_vecs[:, 0], joint_vecs[:, 1], joint_vecs[:, 2], length=0.03, color='g')

#plt.show()


lookup_dict = {}
np.set_printoptions(precision=4, suppress=True)

# 0 is palm, 1-3 index, 4-6 middle, 7-9 ring, 10-12 pinky, 13-15 thumb
for idx in set(vertex_to_joint):
  #print('Joint, fwd, invglo', idx)
  #print(T2xyzrpy(tform_relative[idx, :, :]))
  #print('inv XYZ rpy', T2xyzrpy(np.linalg.inv(tform_relative[idx, :, :])))
  #print('a   XYZ rpy', T2xyzrpy(np.array(m.A[:, :, idx])))
  #print('glo XYZ rpy', T2xyzrpy(np.array(m.A_global[idx])))
  #print(T2xyzrpy(np.linalg.inv(m.A_global[idx])))
  #print('cum XYZ rpy', T2xyzrpy(joints_cum_tform[idx, :, :]))
  prefix = '$J{}_'.format(idx)
  #lookup_dict[prefix+'INVGLO'] = T2xyzrpy(np.linalg.inv(m.A_global[idx]))
  lookup_dict[prefix+'INVGLO'] = T2xyzrpy(np.eye(4))
  lookup_dict[prefix+'FWDREL'] = T2xyzrpy(tform_relative[idx, :, :])

  if idx >= 1:
    joint_vec_str = 'xyz="{:.4f} {:.4f} {:.4f}"'.format(joint_vecs[idx-1, 0], joint_vecs[idx-1, 1], joint_vecs[idx-1, 2])
    lookup_dict[prefix+'AX'] = joint_vec_str
    print(joint_vec_str)

  face_mask = face_to_joint == idx
  faces_in_joint = np.where(face_mask)[0]
  joint_faces = faces[faces_in_joint, :]

  mesh = trimesh.Trimesh(vertices=vertices, faces=joint_faces)
  mesh_tform = trimesh.transformations.identity_matrix()
  mesh_tform[:,:] = np.linalg.inv(m.A_global[idx])
  mesh.apply_transform(mesh_tform)

  wtight = mesh_repair.fill_holes(mesh)
  joint_file = os.path.abspath('mesh/joint_' + str(idx) + '.stl')
  mesh.export(joint_file)
  lookup_dict[prefix+'FILE_ROS'] = 'file://' + joint_file
  lookup_dict[prefix+'FILE'] = joint_file

out_file = []
out_file_ros = []

with open('hand_template.urdf') as f:
  lines = f.readlines()
  for line in lines:
    new_line = line
    new_line_ros = line
    for key, val in lookup_dict.items():
      if key in new_line:
        find_idx = line.find(key)
        new_line = line[:find_idx] + val + line[find_idx + len(key):]
        
        ros_key = key + "_ROS"
        if ros_key in lookup_dict:
            new_line_ros = line[:find_idx] + lookup_dict[ros_key] + line[find_idx + len(key):]
        else:
            new_line_ros = new_line



    out_file.append(new_line)
    out_file_ros.append(new_line_ros)

with open('hand.urdf', 'w') as f:
  for line in out_file:
    f.write(line)

with open('hand_ros.urdf', 'w') as f:
  for line in out_file_ros:
    f.write(line)

print('Done!')