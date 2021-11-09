import os.path
import h5py
import numpy as np
import pybullet as p
import time

from graspME_simtest.sim import Simulation
from graspME_simtest.util import add_frame, sim_grasp


def sim_grasps_hand(grasp_files):
    print("\nSimulate grasps\n")

    mode = p.GUI

    with Simulation(mode) as s:

        # Show initial pose with robot hand frame
        p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraPitch=-30, cameraYaw=45,
                                     cameraTargetPosition=(0, 0, -0.1),
                                     physicsClientId=s.physics_client_id)
        pos = [0, 0, 0]
        orient = [1, 0, 0, 0]
        p.resetBasePositionAndOrientation(s.pandaId, pos, orient)
        body_frame = add_frame(p.getLinkState(s.pandaId, 0)[4], p.getLinkState(s.pandaId, 0)[5], length=0.2)
        for _ in range(100000):
            p.stepSimulation()
        p.removeBody(body_frame)
        p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraPitch=-60, cameraYaw=0,
                                     cameraTargetPosition=(0, 0, 0),
                                     physicsClientId=s.physics_client_id)

        # Load grasps
        with h5py.File(grasp_files, 'r') as f_read:
            obj_name = f_read['object_class'][()]
            poses = f_read['poses']
            num_grasps = len(poses)

            obj_path = os.path.join("data", f_read['object'][()])
            obj_scale = 100 * f_read['object_scale'][()]
            obj_com = f_read['object_com'][()]
            obj_vis = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_path,
                                          meshScale=[obj_scale, obj_scale, obj_scale])
            obj_id = p.createMultiBody(baseVisualShapeIndex=obj_vis, basePosition=obj_com)
            print("\n\tObject '{}': {} grasps\n".format(obj_name, num_grasps))

            for i in range(num_grasps):
                # show random grasps (alternative show one grasp after the other)
                idx = np.random.random_integers(0, num_grasps)
                sim_grasp(s, poses[idx])
                p.stepSimulation()
                time.sleep(.3)

    print("\nDone simulating all grasps")


def main():
    grasp_files = "data/002_master_chef_can_grasps/grasps.h5"

    sim_grasps_hand(grasp_files)

    print("\n--- Finished ---")


if __name__ == '__main__':
    main()
