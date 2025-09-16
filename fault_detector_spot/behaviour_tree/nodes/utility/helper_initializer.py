import py_trees
from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper


class HelperInitializer(py_trees.behaviour.Behaviour):
    """
    One-time behaviour to initialize singleton helpers and attach blackboard clients.
    Returns SUCCESS immediately, so it can be added at the top of the tree.
    """
    def __init__(self, name: str, node, use_simulation: bool = True):
        super().__init__(name)
        self.node = node
        self.slam_helper = None
        self.nav2_helper = None
        self.use_simulation = use_simulation

    def setup(self, timeout):
        # attach a blackboard client
        self.bb_client = self.attach_blackboard_client()
        # register any keys your helpers need

        if self.use_simulation:
            self.slam_helper = SlamToolboxHelper(
                node=self.node,
                blackboard=self.bb_client,
                launch_file="slam_sim_merged_launch.py",
                nav2_launch_file="nav2_sim_launch.py",
                nav2_params_file="nav2_sim_params.yaml"
            )
        else: # real robot
            self.slam_helper = SlamToolboxHelper(
                node=self.node,
                blackboard=self.bb_client,
                launch_file="slam_merged_launch.py",
                nav2_launch_file="nav2_spot_launch.py",
                nav2_params_file="nav2_spot_params.yaml"
            )

        self.nav2_helper = self.slam_helper.nav2_helper

        return True

    def initialise(self):
        # nothing to do on initialise
        pass

    def update(self):
        # immediately return SUCCESS
        return py_trees.common.Status.SUCCESS