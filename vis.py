from time import sleep
import numpy as np
import scipy.io
import trimesh
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import icp

def load_mat(filename):
    mat_content = scipy.io.loadmat(filename, squeeze_me=True)
    x0 = mat_content['x0']
    y0 = mat_content['y0']
    z0 = mat_content['z0']
    x1 = mat_content['x1']
    y1 = mat_content['y1']
    z1 = mat_content['z1']
    x2 = mat_content['x2']
    y2 = mat_content['y2']
    z2 = mat_content['z2']

    x = np.stack((x0, x1, x2), axis=1).flatten()
    y = np.stack((y0, y1, y2), axis=1).flatten()
    z = np.stack((z0, z1, z2), axis=1).flatten()
    contX = mat_content['contX']
    contY = mat_content['contY']
    contZ = mat_content['contZ']
    contForceX = mat_content['contForceX']
    contForceY = mat_content['contForceY']
    contForceZ = mat_content['contForceZ']
    pos = mat_content['pos']
    rot = mat_content['rot']
    print(pos, rot)

    pt = np.stack((x, y, z)).T
    contact_pt = np.stack((contX, contY, contZ)).T
    contact_force_3d = np.stack((contForceX, contForceY, contForceZ)).T
    contact_force = np.linalg.norm(contact_force_3d, axis=1)

    pt -= pos
    contact_pt -= pos

    return pt, contact_pt, contact_force

pt, contact_pt, contact_force = load_mat('deformed.mat')
orig_pt, _, _ = load_mat('orig.mat')

T, distances, iterations = icp.icp(orig_pt, pt, max_iterations=20, tolerance=0.0001)
orig_pt_homo = np.ones((orig_pt.shape[0], 4))
orig_pt_homo[:, :3] = orig_pt
orig_pt_icp = np.dot(T, orig_pt_homo.T).T
orig_pt_icp = orig_pt_icp[:, :3]

faces = np.arange(pt.shape[0]).reshape(-1, 3)
mesh = trimesh.Trimesh(vertices=orig_pt, faces=faces, process=False)


dist_pt = np.linalg.norm(pt - orig_pt_icp, axis=1)
dist_pt = dist_pt - dist_pt.mean()
dist_pt = dist_pt.clip(min=0)
max_dist = dist_pt.max()


for idx, v in enumerate(mesh.vertices):
    base_color = [0, 0, 100, 255]
    
    norm_force = np.power(dist_pt[idx] / max_dist, 0.4)
    base_color[1] = int(norm_force * 255)
        
    mesh.visual.vertex_colors[idx] = base_color

mesh.show()

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot_trisurf(pt[:, 0], pt[:, 1], pt[:, 2], triangles=faces, linewidth=0.2, antialiased=True)
ax.plot_trisurf(orig_pt_icp[:, 0], orig_pt_icp[:, 1], orig_pt_icp[:, 2], triangles=faces, linewidth=0.2, antialiased=True)
#ax.plot_trisurf(orig_pt[:, 0], orig_pt[:, 1], orig_pt[:, 2], triangles=faces, linewidth=0.2, antialiased=True)

#plt.show()