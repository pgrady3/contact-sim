from time import sleep
import numpy as np
import scipy.io
import trimesh


mat_content = scipy.io.loadmat('map.mat', squeeze_me=True)
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

pt = np.stack((x, y, z)).T
contact_pt = np.stack((contX, contY, contZ)).T
contact_force_3d = np.stack((contForceX, contForceY, contForceZ)).T
contact_force = np.linalg.norm(contact_force_3d, axis=1)

max_force = contact_force.max()

faces = np.arange(x.shape[0]).reshape(-1, 3)
mesh = trimesh.Trimesh(vertices=pt, faces=faces, process=False)

for idx, v in enumerate(mesh.vertices):
    base_color = [255, 0, 100, 255]
    
    dist_to_contact = np.linalg.norm(v - contact_pt, axis=1)
    min_idx = np.argmin(dist_to_contact)
    if dist_to_contact[min_idx] < 0.0001:
        norm_force = np.power(contact_force[min_idx] / max_force, 0.2)
        base_color[1] = int(norm_force * 255)
        
    mesh.visual.vertex_colors[idx] = base_color

print(contact_pt)

mesh.show()