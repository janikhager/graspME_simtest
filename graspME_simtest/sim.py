import pybullet as p


class Simulation:
    """
    Class which set ups the simulation environment with the robot hand.
    Should be used within a 'with'-statement.
    """

    def __init__(self, mode=p.GUI):
        """
        init simulation with default parameters
        """
        self.mode = mode

    def __enter__(self):
        self.physics_client_id = p.connect(self.mode)
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        self.pandaId = p.loadURDF("panda_hand/panda_hand.urdf", basePosition=(0, 0, 0))
        p.setJointMotorControlArray(self.pandaId, [1, 2], p.POSITION_CONTROL, [1, 1])

        p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraPitch=-60, cameraYaw=0, cameraTargetPosition=(0, 0, 0),
                                     physicsClientId=self.physics_client_id)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.is_connected():
            p.disconnect(physicsClientId=self.physics_client_id)

    def is_connected(self):
        """
        checks if the simulation instance is still connected
        """
        return p.isConnected(self.physics_client_id)
