"""Script containing the OTM network kernel class."""

from flow.core.kernel.network import BaseKernelNetwork
from flow.core.util import makexml, printxml, ensure_dir
import time
import os
import subprocess
import xml.etree.ElementTree as ElementTree
from lxml import etree
from copy import deepcopy

E = etree.Element

# Number of retries on accessing the .net.xml file before giving up
RETRIES_ON_ERROR = 10
# number of seconds to wait before trying to access the .net.xml file again
WAIT_ON_ERROR = 1


def _flow(name, vtype, route, **kwargs):
    return E('flow', id=name, route=route, type=vtype, **kwargs)


def _inputs(net=None, rou=None, add=None, gui=None):
    inp = E("input")
    inp.append(E("net-file", value=net))
    inp.append(E("route-files", value=rou))
    inp.append(E("additional-files", value=add))
    inp.append(E("gui-settings-file", value=gui))
    return inp


class OTMKernelNetwork(BaseKernelNetwork):
    """Base network kernel for otm-based simulations.

    This class initializes a new network. Networks are used to specify
    features of a network, including the positions of nodes, properties of the
    edges and junctions connecting these nodes, properties of vehicles and
    traffic lights, and other features as well.
    """

    def __init__(self, master_kernel, sim_params):
        """Instantiate a otm network kernel.

        Parameters
        ----------
        master_kernel : flow.core.kernel.Kernel
            the higher level kernel (used to call methods from other
            sub-kernels)
        sim_params : flow.core.params.SimParams
            simulation-specific parameters
        """
        super(OTMKernelNetwork, self).__init__(master_kernel, sim_params)

        # # directories for the network-specific files that will be generated by
        # # the `generate_network` method
        # self.net_path = os.path.dirname(os.path.abspath(__file__)) \
        #     + '/debug/net/'
        # self.cfg_path = os.path.dirname(os.path.abspath(__file__)) \
        #     + '/debug/cfg/'
        #
        # ensure_dir('%s' % self.net_path)
        # ensure_dir('%s' % self.cfg_path)
        #
        # # variables to be defined during network generation
        # self.network = None
        #
        # # self.nodfn = None
        # # self.edgfn = None
        # # self.typfn = None
        # # self.cfgfn = None
        # # self.netfn = None
        # # self.confn = None
        # # self.roufn = None
        # # self.addfn = None
        # # self.sumfn = None
        # # self.guifn = None
        #
        # self._edges = None
        # self._connections = None
        # self._edge_list = None
        # self._junction_list = None
        # self.__max_speed = None
        # self.__length = None  # total length
        # self.__non_internal_length = None  # total length of non-internal edges
        # self.rts = None
        # self.cfg = None

    # OVERRIDE!!
    def generate_network(self, network):
        """Generate the necessary prerequisites for the simulating a network.

        Parameters
        ----------
        network : flow.networks.Network
            an object containing relevant network-specific features such as the
            locations and properties of nodes and edges in the network
        """
        print("Implement Me")
        # # store the network object in the network variable
        # self.network = network
        # self.orig_name = network.orig_name
        # self.name = network.name

        # # names of the soon-to-be-generated xml and otm config files
        # self.nodfn = '%s.nod.xml' % self.network.name
        # self.edgfn = '%s.edg.xml' % self.network.name
        # self.typfn = '%s.typ.xml' % self.network.name
        # self.cfgfn = '%s.netccfg' % self.network.name
        # self.netfn = '%s.net.xml' % self.network.name
        # self.confn = '%s.con.xml' % self.network.name
        # self.roufn = '%s.rou.xml' % self.network.name
        # self.addfn = '%s.add.xml' % self.network.name
        # self.sumfn = '%s.sumo.cfg' % self.network.name
        # self.guifn = '%s.gui.cfg' % self.network.name
        #
        # # can only provide one of osm path or template path to the network
        # assert self.network.net_params.template is None \
        #     or self.network.net_params.osm_path is None
        #
        # # create the network configuration files
        # if self.network.net_params.template is not None:
        #     self._edges, self._connections = self.generate_net_from_template(
        #         self.network.net_params)
        # elif self.network.net_params.osm_path is not None:
        #     self._edges, self._connections = self.generate_net_from_osm(
        #         self.network.net_params)
        # else:
        #     # combine all connections into a list
        #     if network.connections is not None:
        #         if isinstance(network.connections, list):
        #             connections = network.connections
        #         else:
        #             connections = []
        #             for key in network.connections.keys():
        #                 connections.extend(network.connections[key])
        #     else:
        #         connections = None
        #
        #     self._edges, self._connections = self.generate_net(
        #         self.network.net_params,
        #         self.network.traffic_lights,
        #         self.network.nodes,
        #         self.network.edges,
        #         self.network.types,
        #         connections
        #     )
        #
        # # list of edges and internal links (junctions)
        # self._edge_list = [
        #     edge_id for edge_id in self._edges.keys() if edge_id[0] != ':'
        # ]
        # self._junction_list = list(
        #     set(self._edges.keys()) - set(self._edge_list))
        #
        # # maximum achievable speed on any edge in the network
        # self.__max_speed = max(
        #     self.speed_limit(edge) for edge in self.get_edge_list())
        #
        # # length of the network, or the portion of the network in
        # # which cars are meant to be distributed
        # self.__non_internal_length = sum(
        #     self.edge_length(edge_id) for edge_id in self.get_edge_list()
        # )
        #
        # # parameters to be specified under each unique subclass's
        # # __init__ function
        # self.edgestarts = self.network.edge_starts
        #
        # # if no edge_starts are specified, generate default values to be used
        # # by the "get_x" method
        # if self.edgestarts is None:
        #     length = 0
        #     self.edgestarts = []
        #     for edge_id in sorted(self._edge_list):
        #         # the current edge starts where the last edge ended
        #         self.edgestarts.append((edge_id, length))
        #         # increment the total length of the network with the length of
        #         # the current edge
        #         length += self._edges[edge_id]['length']
        #
        # # these optional parameters need only be used if "no-internal-links"
        # # is set to "false" while calling sumo's netconvert function
        # self.internal_edgestarts = self.network.internal_edge_starts
        # self.internal_edgestarts_dict = dict(self.internal_edgestarts)
        #
        # # total_edgestarts and total_edgestarts_dict contain all of the above
        # # edges, with the former being ordered by position
        # self.total_edgestarts = self.edgestarts + self.internal_edgestarts
        # self.total_edgestarts.sort(key=lambda tup: tup[1])
        #
        # self.total_edgestarts_dict = dict(self.total_edgestarts)
        #
        # self.__length = sum(
        #     self._edges[edge_id]['length'] for edge_id in self._edges
        # )
        #
        # if self.network.routes is None:
        #     print("No routes specified, defaulting to single edge routes.")
        #     self.network.routes = {edge: [edge] for edge in self._edge_list}
        #
        # # specify routes vehicles can take  # TODO: move into a method
        # self.rts = self.network.routes
        #
        # # create the sumo configuration files
        # cfg_name = self.generate_cfg(self.network.net_params,
        #                              self.network.traffic_lights,
        #                              self.network.routes)
        #
        # # specify the location of the sumo configuration file
        # self.cfg = self.cfg_path + cfg_name

    # OVERRIDE!!
    def update(self, reset):
        """Update the network with current state information.

        Since networks are generally static, this will most likely not include
        any actions being performed. This is primarily here for consistency
        with other sub-kernels.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        raise NotImplementedError

    # OVERRIDE!!
    def close(self):
        """Close the network."""
        raise NotImplementedError

    ###########################################################################
    #                        State acquisition methods                        #
    ###########################################################################

    # OVERRIDE!!
    def edge_length(self, edge_id):
        """Return the length of a given edge/junction.

        Return -1001 if edge not found.
        """
        print("Implement Me")
        # try:
        #     return self._edges[edge_id]['length']
        # except KeyError:
        #     print('Error in edge length with key', edge_id)
        #     return -1001

    # OVERRIDE!!
    def length(self):
        """Return the total length of all junctions and edges."""
        print("Implement Me")
        # return self.__length

    # OVERRIDE!!
    def speed_limit(self, edge_id):
        """Return the speed limit of a given edge/junction.

        Return -1001 if edge not found.
        """
        # try:
        #     return self._edges[edge_id]['speed']
        # except KeyError:
        #     print('Error in speed limit with key', edge_id)
        #     return -1001

    # OVERRIDE!!
    def max_speed(self):
        """Return the maximum achievable speed on any edge in the network."""
        # return self.__max_speed

    # OVERRIDE!!
    def num_lanes(self, edge_id):
        """Return the number of lanes of a given edge/junction.

        Return -1001 if edge not found.
        """
        # try:
        #     return self._edges[edge_id]['lanes']
        # except KeyError:
        #     print('Error in num lanes with key', edge_id)
        #     return -1001

    # OVERRIDE!!
    def get_edge_list(self):
        """Return the names of all edges in the network."""
        # return self._edge_list

    # OVERRIDE!!
    def get_junction_list(self):
        """Return the names of all junctions in the network."""
        # return self._junction_list

    # OVERRIDE!!
    def get_edge(self, x):  # TODO: maybe remove
        """Compute an edge and relative position from an absolute position.

        Parameters
        ----------
        x : float
            absolute position in network

        Returns
        -------
        tup
            1st element: edge name (such as bottom, right, etc.)
            2nd element: relative position on edge
        """
        # for (edge, start_pos) in reversed(self.total_edgestarts):
        #     if x >= start_pos:
        #         return edge, x - start_pos

    # OVERRIDE!!
    def get_x(self, edge, position):  # TODO: maybe remove
        """Return the absolute position on the track.

        Parameters
        ----------
        edge : str
            name of the edge
        position : float
            relative position on the edge

        Returns
        -------
        float
            position with respect to some global reference
        """
        # # if there was a collision which caused the vehicle to disappear,
        # # return an x value of -1001
        # if len(edge) == 0:
        #     return -1001
        #
        # if edge[0] == ':':
        #     try:
        #         return self.internal_edgestarts_dict[edge] + position
        #     except KeyError:
        #         # in case several internal links are being generalized for
        #         # by a single element (for backwards compatibility)
        #         edge_name = edge.rsplit('_', 1)[0]
        #         return self.total_edgestarts_dict.get(edge_name, -1001)
        # else:
        #     return self.total_edgestarts_dict[edge] + position

    # OVERRIDE!!
    def next_edge(self, edge, lane):
        """Return the next edge/lane pair from the given edge/lane.

        These edges may also be internal links (junctions). Returns an empty
        list if there are no edge/lane pairs in front.
        """
        # try:
        #     return self._connections['next'][edge][lane]
        # except KeyError:
        #     return []

    # OVERRIDE!!
    def prev_edge(self, edge, lane):
        """Return the edge/lane pair right before this edge/lane.

        These edges may also be internal links (junctions). Returns an empty
        list if there are no edge/lane pairs behind.
        """
        # try:
        #     return self._connections['prev'][edge][lane]
        # except KeyError:
        #     return []
