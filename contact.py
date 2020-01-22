import pybullet as p
import pybullet_data
from time import sleep

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

#p.setGravity(0, 0, -10)

#planeId = p.loadURDF("plane.urdf", [0,0,-2])
#boxId = p.loadURDF("cube.urdf", [0,3,2])

#boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

softId = p.loadSoftBody("/home/patrick/contact/bullet3/data/tube.vtk", [0, 0, 0], mass=1000, useNeoHookean = 1, NeoHookeanMu = 60, NeoHookeanLambda = 200, 
                      NeoHookeanDamping = 0.01, useSelfCollision = 0, frictionCoeff = 0.5, 
                      springElasticStiffness=100, springDampingStiffness=10, springBendingStiffness=10, 
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.05)


cubeStartPos = [-0.1, 0, 0.6]
cubeStartOrientation = p.getQuaternionFromEuler([0, 1, 0])
#botId = p.loadURDF("biped/biped2d_pybullet.urdf", cubeStartPos, cubeStartOrientation)
#botId = p.loadURDF("humanoid.urdf", cubeStartPos, cubeStartOrientation)
#botId = p.loadURDF("/home/patrick/soft/GraspIt2URDF/urdf/HumanHand20DOF.urdf", cubeStartPos, cubeStartOrientation, globalScaling=5)
botId = p.loadURDF("/home/patrick/contact/allegro_hand_ros/allegro_hand_description/allegro_hand_description_left.urdf", cubeStartPos, cubeStartOrientation, globalScaling=8)

p.changeDynamics(botId, -1, mass=0.1)

for i in range(p.getNumJoints(botId)):
  p.changeDynamics(botId, i, mass=0.1)
  p.setJointMotorControl2(bodyUniqueId=botId, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetVelocity = 1, force = 10000)
  print(i, p.getJointInfo(botId, i))

#p.changeDynamics(softId, -1, mass=1000)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

while p.isConnected():

  botPos, botOrn = p.getBasePositionAndOrientation(botId)
  softPos, softOrn = p.getBasePositionAndOrientation(softId)
  #p.setGravity(0,0,-10)
  p.applyExternalForce(botId, -1, [0, 0, -50], botPos, flags=p.WORLD_FRAME)
  p.applyExternalForce(softId, -1, [10000, 1000, 0], softPos, flags=p.WORLD_FRAME)
  p.stepSimulation()
  sleep(1./240.)
  #print('bot ', p.getDynamicsInfo(botId, -1))
  #print('soft', p.getDynamicsInfo(softId, -1))
