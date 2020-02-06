import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
import scipy.io

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

mat_content = scipy.io.loadmat('map.mat', squeeze_me=True)
x = mat_content['x']
y = mat_content['y']
z = mat_content['z']
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

p.setRealTimeSimulation(0)

sphere_red = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 1])
sphere_orange = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0.5, 0, 1])
sphere_yellow = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 1, 0, 1])


debug_lines = []
for i in range(len(x) - 1):
    #print(pt[i, :], pt[i+1, :])
    line_id = p.addUserDebugLine(pt[i, :], pt[i+1, :])
    debug_lines.append(line_id)

for i in range(len(contX)):
    #print(pt[i, :], pt[i+1, :])
    #line_id = p.addUserDebugLine(contact_pt[i, :], contact_pt[i, :] + contact_force[i, :] * 0.01, lineColorRGB=[1, 0, 0], lineWidth=10)
    #debug_lines.append(line_id)
    if contact_force[i] > 0.5 * max_force:
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_red, basePosition=contact_pt[i, :])
    elif contact_force[i] > 0.2 * max_force:
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_orange, basePosition=contact_pt[i, :])
    else:
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_yellow, basePosition=contact_pt[i, :])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
sim_count = 0
while p.isConnected():
    #debug_lines[i] = p.addUserDebugLine(pt, pt + contact_force[i, :]*0.01, lineWidth=5, replaceItemUniqueId=debug_lines[i])

    p.stepSimulation()
    sleep(1./240.)
    sim_count += 1
