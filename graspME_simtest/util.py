import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation


def add_frame(center, orientation, length=.3):
    cylinders = []
    colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    Rx = Rotation.from_euler('y', 90, degrees=True).as_quat()
    Ry = Rotation.from_euler('x', 90, degrees=True).as_quat()
    Rz = np.eye(3)
    px = [length / 2, 0, 0]
    py = [0, length / 2, 0]
    pz = [0, 0, length / 2]
    orientations = [Rx, Ry, Rz]
    positions = [px, py, pz]
    for i in range(3):
        cylinders.append(
            p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.002,
                length=length,
                rgbaColor=colors[i],
                visualFramePosition=positions[i],
                visualFrameOrientation=orientations[i])
        )

    # pose the body
    body = p.createMultiBody(
        baseMass=0,
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 1],
        baseVisualShapeIndex=-1,
        linkMasses=[0] * 3,
        linkPositions=[[0, 0, 0]] * 3,
        linkOrientations=[[0, 0, 0, 1]] * 3,
        linkInertialFramePositions=[[0, 0, 0]] * 3,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * 3,
        linkVisualShapeIndices=cylinders,
        linkCollisionShapeIndices=[-1] * 3,
        linkParentIndices=[0, 1, 2],
        linkJointTypes=[p.JOINT_FIXED] * 3,
        linkJointAxis=[[0, 0, 1]] * 3
    )
    p.resetBasePositionAndOrientation(body, center, orientation)

    return body


def sim_grasp(simulation, grasp_pose):
    pos = grasp_pose[:3]
    orient = grasp_pose[3:]
    p.resetBasePositionAndOrientation(simulation.pandaId, pos, orient)

    return p.getLinkState(simulation.pandaId, 2)[0]
