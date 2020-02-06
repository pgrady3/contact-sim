import numpy as np
from webuser.smpl_handpca_wrapper_HAND_only import load_model
import platform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import mode
import trimesh

# Load MANO model (here we load the right hand model)
try:
  m = load_model('mano/models/MANO_RIGHT.pkl', ncomps=6, flat_hand_mean=False)
except:
  m = load_model('../mano/models/MANO_RIGHT.pkl', ncomps=6, flat_hand_mean=False)


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


for idx in set(vertex_to_joint):
  print("Plotting joint", idx)

  face_mask = face_to_joint == idx
  faces_in_joint = np.where(face_mask)[0]


  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(joints3D[:, 0], joints3D[:, 1], joints3D[:, 2], color='b') # Plot joints
  ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=faces[faces_in_joint, :], linewidth=0.2, antialiased=True)
  ax.set_aspect('equal', 'box')
  plt.show()

