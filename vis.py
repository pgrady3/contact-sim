from time import sleep
import numpy as np
import scipy.io
import trimesh
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import icp
import argparse

def parse_args():
  parser = argparse.ArgumentParser(description='Contact visualization')
  parser.add_argument('--folder', default='output/', type=str, help='Input folder')
  parser.add_argument('--infile', default='test1', type=str, help='Input filename')
  args = parser.parse_args()

  return args

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
    hand_state = mat_content['handState']

    pt = np.stack((x, y, z)).T
    contact_pt = np.stack((contX, contY, contZ)).T
    contact_force_3d = np.stack((contForceX, contForceY, contForceZ)).T
    contact_force = np.linalg.norm(contact_force_3d, axis=1)

    pt -= pos
    contact_pt -= pos

    return pt, contact_pt, contact_force, hand_state

def render_mesh(mesh, fileprefix):

    mesh_list = []
    for i in range(3):
        m = mesh.copy()
        R = trimesh.transformations.rotation_matrix(i * 1.0, [1,0,1])
        m.apply_transform(R)
        m.apply_translation([i*2.5, 0, 0])
        mesh_list.append(m)

        m = mesh.copy()
        R = trimesh.transformations.rotation_matrix((i+1) * 1.0, [1,1,0])
        m.apply_transform(R)
        m.apply_translation([i*2.5, 2.5, 0])
        mesh_list.append(m)

    scene = trimesh.Scene(mesh_list)

    for i in range(1):
        #camera_old, _geometry = scene.graph[scene.camera.name]
        #camera_new = np.dot(camera_old, rotate)
        #scene.graph[scene.camera.name] = camera_new
        #rotate = trimesh.transformations.rotation_matrix(angle=np.radians(60.0 * i), direction=[0, 1, 0], point=scene.centroid)
        
        #fov = np.array([20, 50])
        #transform = trimesh.scene.cameras.look_at(mesh.vertices, fov, rotation=rotate)
        #print(transform)
        #scene.graph[scene.camera.name] = transform

        try:
            # increment the file name
            file_name = fileprefix + '_' + str(i) + '.png'
            png = scene.save_image(resolution=[1920, 1080], visible=True)
            with open(file_name, 'wb') as f:
                f.write(png)
                f.close()

        except BaseException as E:
            print("unable to save image", str(E))

args = parse_args()

deform_pt, contact_pt, contact_force, hand_state = load_mat(args.folder + args.infile + '_deformed.mat')
orig_pt, _, _, _ = load_mat(args.folder + args.infile + '_orig.mat')

T, distances, iterations = icp.icp(orig_pt, deform_pt, max_iterations=20, tolerance=0.0001)
orig_pt_homo = np.ones((orig_pt.shape[0], 4))
orig_pt_homo[:, :3] = orig_pt
orig_pt_icp = np.dot(T, orig_pt_homo.T).T
orig_pt_icp = orig_pt_icp[:, :3]

shifted_pt = np.array(deform_pt)
#shifted_pt[:, 0] += 1
faces = np.arange(deform_pt.shape[0]).reshape(-1, 3)
mesh_deform = trimesh.Trimesh(vertices=shifted_pt, faces=faces, process=False)
mesh_force = trimesh.Trimesh(vertices=orig_pt, faces=faces, process=False)

dist_pt = np.linalg.norm(deform_pt - orig_pt_icp, axis=1)
dist_pt = dist_pt - dist_pt.mean()
dist_pt = dist_pt.clip(min=0)
dist_pt = dist_pt + dist_pt.min()

normalized_deform = np.power(dist_pt / dist_pt.max(), 0.3)
mesh_deform.visual.vertex_colors = trimesh.visual.interpolate(normalized_deform, color_map='viridis') 

force_pt = np.zeros(dist_pt.shape)
for i in range(contact_pt.shape[0]):
    dist_to_contact = np.linalg.norm(contact_pt[i, :] - deform_pt, axis=1)
    #min_idx = np.argmin(dist_to_contact)
    #print(dist_to_contact[min_idx])
    close_pts = dist_to_contact < 0.0001
    
    force_pt[close_pts] = contact_force[i]

normalized_force = np.power(force_pt / force_pt.max(), 0.2)
mesh_force.visual.vertex_colors = trimesh.visual.interpolate(normalized_force, color_map='viridis') 

vis_list = []
#vis_list.append(mesh_force)
vis_list.append(mesh_deform)
trimesh.Scene(vis_list).show()

path = args.folder + "force_" + args.infile
render_mesh(mesh_force, path)
path = args.folder + "deform_" + args.infile
render_mesh(mesh_deform, path)

# hand_meshes = []

# for i in range(hand_state.shape[0]):
#     filename = str('urdf/mesh/joint_{}.stl'.format(i))
#     mesh = trimesh.load(filename)
#     hand_meshes.append(mesh)
#     quat = np.zeros(4)
#     quat[1:] = hand_state[i, 3:6]
#     quat[0] = hand_state[i, 6]

#     Rq = trimesh.transformations.quaternion_matrix(quat)
#     T = trimesh.transformations.translation_matrix(hand_state[i, :3] / 16)
#     Rq[:3, 3] = hand_state[i, :3] / 32
#     #print(hand_state[i, :3])
#     mesh.apply_transform(Rq)
#     #mesh.apply_transform(T)
    
    

# trimesh.Scene(hand_meshes).show()

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot_trisurf(deform_pt[:, 0], deform_pt[:, 1], deform_pt[:, 2], triangles=faces, linewidth=0.2, antialiased=True)
# ax.plot_trisurf(orig_pt_icp[:, 0], orig_pt_icp[:, 1], orig_pt_icp[:, 2], triangles=faces, linewidth=0.2, antialiased=True)
# ax.plot_trisurf(orig_pt[:, 0], orig_pt[:, 1], orig_pt[:, 2], triangles=faces, linewidth=0.2, antialiased=True)
# plt.show()