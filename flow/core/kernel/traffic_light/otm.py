"""Script containing the OTM traffic light kernel class."""

from flow.core.kernel.traffic_light import KernelTrafficLight


class OTMTrafficLight(KernelTrafficLight):
    """OTM traffic light kernel.

    Implements all methods discussed in the base traffic light kernel class.
    """

    def __init__(self, master_kernel):
        """Instantiate the OTM traffic light kernel.

        Parameters
        ----------
        master_kernel : flow.core.kernel.Kernel
            the higher level kernel (used to call methods from other
            sub-kernels)
        """
        print("implement me")
        KernelTrafficLight.__init__(self, master_kernel)

        # self.__tls = dict()  # contains current time step traffic light data
        # self.__tls_properties = dict()  # traffic light xml properties
        #
        # # names of nodes with traffic lights
        # self.__ids = []
        #
        # # number of traffic light nodes
        # self.num_traffic_lights = 0

    # OVERRIDE!!
    def update(self, reset):
        """Update the states and phases of the traffic lights.

        This ensures that the traffic light variables match current traffic
        light data.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        # tls_obs = {}
        # for tl_id in self.__ids:
        #     tls_obs[tl_id] = \
        #         self.kernel_api.trafficlight.getSubscriptionResults(tl_id)
        # self.__tls = tls_obs.copy()

    # OVERRIDE!!
    def get_ids(self):
        """Return the names of all nodes with traffic lights."""
        # return self.__ids

    # OVERRIDE!!
    def set_state(self, node_id, state, link_index="all"):
        """Set the state of the traffic lights on a specific node.

        Parameters
        ----------
        node_id : str
            name of the node with the controlled traffic lights
        state : str
            desired state(s) for the traffic light
        link_index : int, optional
            index of the link whose traffic light state is meant to be changed.
            If no value is provided, the lights on all links are updated.
        """
        # if link_index == "all":
        #     # if lights on all lanes are changed
        #     self.kernel_api.trafficlight.setRedYellowGreenState(
        #         tlsID=node_id, state=state)
        # else:
        #     # if lights on a single lane is changed
        #     self.kernel_api.trafficlight.setLinkState(
        #         tlsID=node_id, tlsLinkIndex=link_index, state=state)

    # OVERRIDE!!
    def get_state(self, node_id):
        """Return the state of the traffic light(s) at the specified node.

        Parameters
        ----------
        node_id: str
            name of the node

        Returns
        -------
        state : str
            Index = lane index
            Element = state of the traffic light at that node/lane
        """
        # return self.__tls[node_id][tc.TL_RED_YELLOW_GREEN_STATE]
