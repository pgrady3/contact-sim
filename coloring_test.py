import trimesh
import numpy as np

meshes = [trimesh.creation.uv_sphere() for i in range(10)]

for i, m in enumerate(meshes):
    m.vertices *= (np.random.random(3) + 1 ) * 2
    m.apply_translation([0,0, i*6])
    radii = np.linalg.norm(m.vertices - m.center_mass, axis=1)
    print('Vertices shape', m.vertices.shape)
    m.visual.vertex_colors = trimesh.visual.interpolate(radii, color_map='viridis') 

trimesh.Scene(meshes).show()