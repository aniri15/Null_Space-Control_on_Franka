import pybullet as p
import pybullet_data


q_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5], rgbaColor=[0, 1, 0, 1])
q_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])


q = p.createMultiBody(
    baseMass=1,
    baseInertialFramePosition=[0, 0, 0],
    baseCollisionShapeIndex=q_collision,
    baseVisualShapeIndex=q_visual,
    basePosition=[0, 0, 1],
    useMaximalCoordinates=True
)
visual_shape_id = p.createVisualShape(shapeType, radius=1, halfExtents=None, fileName=None, 
    rgbaColor=None, specularColor=None, visualFramePosition=None, visualFrameOrientation=None, 
    meshScale=None, flags=0, physicsClientId=0)

collision_shape_id = p.createCollisionShape(
    shapeType,
    radius=1,
    halfExtents=None,
    height=None,
    fileName=None,
    meshScale=None,
    collisionFramePosition=None,
    collisionFrameOrientation=None,
    flags=0,
    physicsClientId=0
)
body_id = p.createMultiBody(
    baseMass=0,
    baseInertialFramePosition=None,
    baseInertialFrameOrientation=None,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=-1,
    basePosition=None,
    baseOrientation=None,
    baseLinVel=None,
    baseAngVel=None,
    baseAppliedForce=None,
    baseTorque=None,
    baseInertialFramePositionLocalInertial=None,
    baseInertialFrameOrientationLocalInertial=None,
    linkMasses=None,
    linkCollisionShapeIndices=None,
    linkVisualShapeIndices=None,
    linkPositions=None,
    linkOrientations=None,
    linkInertialFramePositions=None,
    linkInertialFrameOrientations=None,
    linkParentIndices=None,
    linkJointTypes=None,
    linkJointAxes=None,
    useMaximalCoordinates=False,
    flags=0,
    physicsClientId=0
)


rollId = p.addUserDebugParameter("roll", -1.5, 1.5, 0)
pitchId = p.addUserDebugParameter("pitch", -1.5, 1.5, 0)
yawId = p.addUserDebugParameter("yaw", -1.5, 1.5, 0)
fwdxId = p.addUserDebugParameter("fwd_x", -1, 1, 0)
fwdyId = p.addUserDebugParameter("fwd_y", -1, 1, 0)
fwdzId = p.addUserDebugParameter("fwd_z", -1, 1, 0)

while True:
    roll = p.readUserDebugParameter(rollId)
    pitch = p.readUserDebugParameter(pitchId)
    yaw = p.readUserDebugParameter(yawId)
    x = p.readUserDebugParameter(fwdxId)
    y = p.readUserDebugParameter(fwdyId)
    z = p.readUserDebugParameter(fwdzId)

    orn = p.getQuaternionFromEuler([roll, pitch, yaw])
    p.resetBasePositionAndOrientation(q, [x, y, z], orn)
    # p.stepSimulation()  # not really necessary for this demo, no physics used
