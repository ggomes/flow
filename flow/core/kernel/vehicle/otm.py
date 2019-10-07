"""Script containing the OTM vehicle kernel class."""
import traceback

from flow.core.kernel.vehicle import KernelVehicle
import numpy as np
import collections
import warnings
from flow.controllers.car_following_models import SimCarFollowingController
from flow.controllers.rlcontroller import RLController
from flow.controllers.lane_change_controllers import SimLaneChangeController
from bisect import bisect_left
import itertools
from copy import deepcopy

# colors for vehicles
WHITE = (255, 255, 255)
CYAN = (0, 255, 255)
RED = (255, 0, 0)


class OTMVehicle(KernelVehicle):
    """Flow kernel for the OTM API.

    Extends flow.core.kernel.vehicle.base.KernelVehicle
    """

    def __init__(self,
                 master_kernel,
                 sim_params):
        """See parent class."""
        KernelVehicle.__init__(self, master_kernel, sim_params)

        self.__ids = []  # ids of all vehicles
        self.__human_ids = []  # ids of human-driven vehicles
        self.__controlled_ids = []  # ids of flow-controlled vehicles
        self.__controlled_lc_ids = []  # ids of flow lc-controlled vehicles
        self.__rl_ids = []  # ids of rl-controlled vehicles
        self.__observed_ids = []  # ids of the observed vehicles

        # vehicles: Key = Vehicle ID, Value = Dictionary describing the vehicle
        # Ordered dictionary used to keep neural net inputs in order
        self.__vehicles = collections.OrderedDict()

        # create a sumo_observations variable that will carry all information
        # on the state of the vehicles for a given time step
        self.__sumo_obs = {}

        # total number of vehicles in the network
        self.num_vehicles = 0
        # number of rl vehicles in the network
        self.num_rl_vehicles = 0

        # contains the parameters associated with each type of vehicle
        self.type_parameters = {}

        # contain the minGap attribute of each type of vehicle
        self.minGap = {}

        # list of vehicle ids located in each edge in the network
        self._ids_by_edge = dict()

        # number of vehicles that entered the network for every time-step
        self._num_departed = []
        self._departed_ids = []

        # number of vehicles to exit the network for every time-step
        self._num_arrived = []
        self._arrived_ids = []

        # whether or not to automatically color vehicles
        try:
            self._color_vehicles = sim_params.color_vehicles
        except AttributeError:
            self._color_vehicles = False

    ###########################################################################
    #               Methods for interacting with the simulator                #
    ###########################################################################

    # OVERRIDE!!!
    def update(self, reset):
        """Update the vehicle kernel with data from the current time step.

        This method is used to optimize the computational efficiency of
        acquiring vehicle state information from the kernel.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step

        The following actions are performed:

        * The state of all vehicles is modified to match their state at the
          current time step. This includes states specified by sumo, and states
          explicitly defined by flow, e.g. "num_arrived".
        * If vehicles exit the network, they are removed from the vehicles
          class, and newly departed vehicles are introduced to the class.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        # vehicle_obs = {}
        # for veh_id in self.__ids:
        #     vehicle_obs[veh_id] = \
        #         self.kernel_api.vehicle.getSubscriptionResults(veh_id)
        # sim_obs = self.kernel_api.simulation.getSubscriptionResults()
        #
        # # remove exiting vehicles from the vehicles class
        # for veh_id in sim_obs[tc.VAR_ARRIVED_VEHICLES_IDS]:
        #     if veh_id in sim_obs[tc.VAR_TELEPORT_STARTING_VEHICLES_IDS]:
        #         # this is meant to resolve the KeyError bug when there are
        #         # collisions
        #         vehicle_obs[veh_id] = self.__sumo_obs[veh_id]
        #     self.remove(veh_id)
        #     # remove exiting vehicles from the vehicle subscription if they
        #     # haven't been removed already
        #     if vehicle_obs[veh_id] is None:
        #         vehicle_obs.pop(veh_id, None)
        #
        # # add entering vehicles into the vehicles class
        # for veh_id in sim_obs[tc.VAR_DEPARTED_VEHICLES_IDS]:
        #     if veh_id in self.get_ids() and vehicle_obs[veh_id] is not None:
        #         # this occurs when a vehicle is actively being removed and
        #         # placed again in the network to ensure a constant number of
        #         # total vehicles (e.g. TrafficLightGridEnv). In this case, the vehicle
        #         # is already in the class; its state data just needs to be
        #         # updated
        #         pass
        #     else:
        #         veh_type = self.kernel_api.vehicle.getTypeID(veh_id)
        #         obs = self._add_departed(veh_id, veh_type)
        #         # add the subscription information of the new vehicle
        #         vehicle_obs[veh_id] = obs
        #
        # if reset:
        #     self.time_counter = 0
        #
        #     # reset all necessary values
        #     self.prev_last_lc = dict()
        #     for veh_id in self.__rl_ids:
        #         self.__vehicles[veh_id]["last_lc"] = -float("inf")
        #         self.prev_last_lc[veh_id] = -float("inf")
        #     self._num_departed.clear()
        #     self._num_arrived.clear()
        #     self._departed_ids.clear()
        #     self._arrived_ids.clear()
        #
        #     # add vehicles from a network template, if applicable
        #     if hasattr(self.master_kernel.network.network,
        #                "template_vehicles"):
        #         for veh_id in self.master_kernel.network.network. \
        #                 template_vehicles:
        #             vals = deepcopy(self.master_kernel.network.network.
        #                             template_vehicles[veh_id])
        #             # a step is executed during initialization, so add this sim
        #             # step to the departure time of vehicles
        #             vals['depart'] = str(
        #                 float(vals['depart']) + 2 * self.sim_step)
        #             self.kernel_api.vehicle.addFull(
        #                 veh_id, 'route{}_0'.format(veh_id), **vals)
        # else:
        #     self.time_counter += 1
        #     # update the "last_lc" variable
        #     for veh_id in self.__rl_ids:
        #         prev_lane = self.get_lane(veh_id)
        #         if vehicle_obs[veh_id][tc.VAR_LANE_INDEX] != prev_lane:
        #             self.__vehicles[veh_id]["last_lc"] = self.time_counter
        #
        #     # updated the list of departed and arrived vehicles
        #     self._num_departed.append(
        #         len(sim_obs[tc.VAR_DEPARTED_VEHICLES_IDS]))
        #     self._num_arrived.append(len(sim_obs[tc.VAR_ARRIVED_VEHICLES_IDS]))
        #     self._departed_ids.append(sim_obs[tc.VAR_DEPARTED_VEHICLES_IDS])
        #     self._arrived_ids.append(sim_obs[tc.VAR_ARRIVED_VEHICLES_IDS])
        #
        # # update the "headway", "leader", and "follower" variables
        # for veh_id in self.__ids:
        #     try:
        #         _position = vehicle_obs.get(veh_id, {}).get(
        #             tc.VAR_POSITION, -1001)
        #         _angle = vehicle_obs.get(veh_id, {}).get(tc.VAR_ANGLE, -1001)
        #         _time_step = sim_obs[tc.VAR_TIME_STEP]
        #         _time_delta = sim_obs[tc.VAR_DELTA_T]
        #         self.__vehicles[veh_id]["orientation"] = \
        #             list(_position) + [_angle]
        #         self.__vehicles[veh_id]["timestep"] = _time_step
        #         self.__vehicles[veh_id]["timedelta"] = _time_delta
        #     except TypeError:
        #         print(traceback.format_exc())
        #     headway = vehicle_obs.get(veh_id, {}).get(tc.VAR_LEADER, None)
        #     # check for a collided vehicle or a vehicle with no leader
        #     if headway is None:
        #         self.__vehicles[veh_id]["leader"] = None
        #         self.__vehicles[veh_id]["follower"] = None
        #         self.__vehicles[veh_id]["headway"] = 1e+3
        #         self.__vehicles[veh_id]["follower_headway"] = 1e+3
        #     else:
        #         min_gap = self.minGap[self.get_type(veh_id)]
        #         self.__vehicles[veh_id]["headway"] = headway[1] + min_gap
        #         self.__vehicles[veh_id]["leader"] = headway[0]
        #         if headway[0] in self.__vehicles:
        #             leader = self.__vehicles[headway[0]]
        #             # if veh_id is closer from leader than another follower
        #             # (in case followers are in different converging edges)
        #             if ("follower_headway" not in leader or
        #                     headway[1] + min_gap < leader["follower_headway"]):
        #                 leader["follower"] = veh_id
        #                 leader["follower_headway"] = headway[1] + min_gap
        #
        # # update the sumo observations variable
        # self.__sumo_obs = vehicle_obs.copy()
        #
        # # update the lane leaders data for each vehicle
        # self._multi_lane_headways()
        #
        # # make sure the rl vehicle list is still sorted
        # self.__rl_ids.sort()

    # OVERIDE!!!
    def add(self, veh_id, type_id, edge, pos, lane, speed):
        """Add a vehicle to the network.

        Parameters
        ----------
        veh_id : str
            unique identifier of the vehicle to be added
        type_id : str
            vehicle type of the added vehicle
        edge : str
            starting edge of the added vehicle
        pos : float
            starting position of the added vehicle
        lane : int
            starting lane of the added vehicle
        speed : float
            starting speed of the added vehicle
        """
        if veh_id in self.master_kernel.network.rts:
            # If the vehicle has its own route, use that route. This is used in
            # the case of network templates.
            route_id = 'route{}_0'.format(veh_id)
        else:
            num_routes = len(self.master_kernel.network.rts[edge])
            frac = [val[1] for val in self.master_kernel.network.rts[edge]]
            route_id = 'route{}_{}'.format(edge, np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0])

        self.kernel_api.vehicle.addFull(
            veh_id,
            route_id,
            typeID=str(type_id),
            departLane=str(lane),
            departPos=str(pos),
            departSpeed=str(speed))

    # OVERRIDE!!!
    def remove(self, veh_id):
        """Remove a vehicle.

        This method removes all traces of the vehicle from the vehicles kernel
        and all valid ID lists, and decrements the total number of vehicles in
        this class.

        In addition, if the vehicle is still in the network, this method calls
        the necessary simulator-specific commands to remove it.

        Parameters
        ----------
        veh_id : str
            unique identifier of the vehicle to be removed
        """
        # # remove from sumo
        # if veh_id in self.kernel_api.vehicle.getIDList():
        #     self.kernel_api.vehicle.unsubscribe(veh_id)
        #     self.kernel_api.vehicle.remove(veh_id)
        #
        # if veh_id in self.__ids:
        #     self.__ids.remove(veh_id)
        #
        # # remove from the vehicles kernel
        # if veh_id in self.__vehicles:
        #     del self.__vehicles[veh_id]
        #
        # if veh_id in self.__sumo_obs:
        #     del self.__sumo_obs[veh_id]
        #
        # # remove it from all other id lists (if it is there)
        # if veh_id in self.__human_ids:
        #     self.__human_ids.remove(veh_id)
        #     if veh_id in self.__controlled_ids:
        #         self.__controlled_ids.remove(veh_id)
        #     if veh_id in self.__controlled_lc_ids:
        #         self.__controlled_lc_ids.remove(veh_id)
        # elif veh_id in self.__rl_ids:
        #     self.__rl_ids.remove(veh_id)
        #     # make sure that the rl ids remain sorted
        #     self.__rl_ids.sort()
        #
        # # modify the number of vehicles and RL vehicles
        # self.num_vehicles = len(self.get_ids())
        # self.num_rl_vehicles = len(self.get_rl_ids())

    # OVERRIDE!!!
    def apply_acceleration(self, veh_id, acc):
        """Apply the acceleration requested by a vehicle in the simulator.

        Parameters
        ----------
        veh_id : str or list of str
            list of vehicle identifiers
        acc : float or array_like
            requested accelerations from the vehicles
        """
        # # to hand the case of a single vehicle
        # if type(veh_ids) == str:
        #     veh_ids = [veh_ids]
        #     acc = [acc]
        #
        # for i, vid in enumerate(veh_ids):
        #     if acc[i] is not None and vid in self.get_ids():
        #         this_vel = self.get_speed(vid)
        #         next_vel = max([this_vel + acc[i] * self.sim_step, 0])
        #         self.kernel_api.vehicle.slowDown(vid, next_vel, 1e-3)

    # OVERRIDE!!!
    def apply_lane_change(self, veh_id, direction):
        """Apply an instantaneous lane-change to a set of vehicles.

        This method also prevents vehicles from moving to lanes that do not
        exist, and set the "last_lc" variable for RL vehicles that lane changed
        to match the current time step, in order to assist in maintaining a
        lane change duration for these vehicles.

        Parameters
        ----------
        veh_id : str or list of str
            list of vehicle identifiers
        direction : {-1, 0, 1} or list of {-1, 0, 1}
            -1: lane change to the right
             0: no lane change
             1: lane change to the left
-
        Raises
        ------
        ValueError
            If any of the direction values are not -1, 0, or 1.
        """
        # # to hand the case of a single vehicle
        # if type(veh_ids) == str:
        #     veh_ids = [veh_ids]
        #     direction = [direction]
        #
        # # if any of the directions are not -1, 0, or 1, raise a ValueError
        # if any(d not in [-1, 0, 1] for d in direction):
        #     raise ValueError(
        #         "Direction values for lane changes may only be: -1, 0, or 1.")
        #
        # for i, veh_id in enumerate(veh_ids):
        #     # check for no lane change
        #     if direction[i] == 0:
        #         continue
        #
        #     # compute the target lane, and clip it so vehicle don't try to lane
        #     # change out of range
        #     this_lane = self.get_lane(veh_id)
        #     this_edge = self.get_edge(veh_id)
        #     target_lane = min(
        #         max(this_lane + direction[i], 0),
        #         self.master_kernel.network.num_lanes(this_edge) - 1)
        #
        #     # perform the requested lane action action in TraCI
        #     if target_lane != this_lane:
        #         self.kernel_api.vehicle.changeLane(
        #             veh_id, int(target_lane), 100000)
        #
        #         if veh_id in self.get_rl_ids():
        #             self.prev_last_lc[veh_id] = \
        #                 self.__vehicles[veh_id]["last_lc"]

    # OVERRIDE!!!
    def choose_routes(self, veh_id, route_choices):
        """Update the route choice of vehicles in the network.

        Parameters
        ----------
        veh_id : str or list of str
            list of vehicle identifiers
        route_choices : array_like
            list of edges the vehicle wishes to traverse, starting with the
            edge the vehicle is currently on. If a value of None is provided,
            the vehicle does not update its route
        """
        # # to hand the case of a single vehicle
        # if type(veh_ids) == str:
        #     veh_ids = [veh_ids]
        #     route_choices = [route_choices]
        #
        # for i, veh_id in enumerate(veh_ids):
        #     if route_choices[i] is not None:
        #         self.kernel_api.vehicle.setRoute(
        #             vehID=veh_id, edgeList=route_choices[i])

    # OVERRIDE!!!
    def set_max_speed(self, veh_id, max_speed):
        """Update the maximum allowable speed by a vehicles in the network.

        Parameters
        ----------
        veh_id : list
            vehicle identifier
        max_speed : float
            desired max speed by the vehicle
        """
        # self.kernel_api.vehicle.setMaxSpeed(veh_id, max_speed)

    ###########################################################################
    # Methods to visually distinguish vehicles by {RL, observed, unobserved}  #
    ###########################################################################

    # OVERRIDE!!!
    def update_vehicle_colors(self):
        """Modify the color of vehicles if rendering is active.
        The colors of all vehicles are updated as follows:
        - red: autonomous (rl) vehicles
        - white: unobserved human-driven vehicles
        - cyan: observed human-driven vehicles
        """
        # for veh_id in self.get_rl_ids():
        #     try:
        #         # color rl vehicles red
        #         self.set_color(veh_id=veh_id, color=RED)
        #     except (FatalTraCIError, TraCIException) as e:
        #         print('Error when updating rl vehicle colors:', e)
        #
        # # color vehicles white if not observed and cyan if observed
        # for veh_id in self.get_human_ids():
        #     try:
        #         color = CYAN if veh_id in self.get_observed_ids() else WHITE
        #         self.set_color(veh_id=veh_id, color=color)
        #     except (FatalTraCIError, TraCIException) as e:
        #         print('Error when updating human vehicle colors:', e)
        #
        # # clear the list of observed vehicles
        # for veh_id in self.get_observed_ids():
        #     self.remove_observed(veh_id)

    # OVERRIDE!!!
    def set_observed(self, veh_id):
        """Add a vehicle to the list of observed vehicles."""
        # if veh_id not in self.__observed_ids:
        #     self.__observed_ids.append(veh_id)

    # OVERRIDE!!!
    def remove_observed(self, veh_id):
        """Remove a vehicle from the list of observed vehicles."""
        # if veh_id in self.__observed_ids:
        #     self.__observed_ids.remove(veh_id)

    # OVERRIDE!!!
    def get_observed_ids(self):
        """Return the list of observed vehicles."""
        # return self.__observed_ids

    # OVERRIDE!!!
    def get_color(self, veh_id):
        """Return and RGB tuple of the color of the specified vehicle."""
        # r, g, b, t = self.kernel_api.vehicle.getColor(veh_id)
        # return r, g, b

    # OVERRIDE!!!
    def set_color(self, veh_id, color):
        """Set the color of the specified vehicle with the RGB tuple.
        The last term for sumo (transparency) is set to 255.
        """
        # if self._color_vehicles:
        #     r, g, b = color
        #     self.kernel_api.vehicle.setColor(
        #         vehID=veh_id, color=(r, g, b, 255))

    ###########################################################################
    #                        State acquisition methods                        #
    ###########################################################################

    # OVERRIDE!!!
    def get_orientation(self, veh_id):
        """Return the orientation of the vehicle of veh_id."""
        # return self.__vehicles[veh_id]["orientation"]

    # OVERRIDE!!!
    def get_timestep(self, veh_id):
        """Return the time step of the vehicle of veh_id."""
        # return self.__vehicles[veh_id]["timestep"]

    # OVERRIDE!!!
    def get_timedelta(self, veh_id):
        """Return the simulation time delta of the vehicle of veh_id."""
        # return self.__vehicles[veh_id]["timedelta"]

    # OVERRIDE!!!
    def get_type(self, veh_id):
        """Return the type of the vehicle of veh_id."""
        # return self.__vehicles[veh_id]["type"]

    # OVERRIDE!!!
    def get_ids(self):
        """Return the names of all vehicles currently in the network."""
        # return self.__ids

    # OVERRIDE!!!
    def get_human_ids(self):
        """Return the names of all non-rl vehicles currently in the network."""
        # return self.__human_ids

    # OVERRIDE!!!
    def get_controlled_ids(self):
        """Return the names of all flow acceleration-controlled vehicles.

        This only include vehicles that are currently in the network.
        """
        # return self.__controlled_ids

    # OVERRIDE!!!
    def get_controlled_lc_ids(self):
        """Return the names of all flow lane change-controlled vehicles.

        This only include vehicles that are currently in the network.
        """
        # return self.__controlled_lc_ids

    # OVERRIDE!!!
    def get_rl_ids(self):
        """Return the names of all rl-controlled vehicles in the network."""
        # return self.__rl_ids

    # OVERRIDE!!!
    def get_ids_by_edge(self, edges):
        """Return the names of all vehicles in the specified edge.

        If no vehicles are currently in the edge, then returns an empty list.
        """
        # if isinstance(edges, (list, np.ndarray)):
        #     return sum([self.get_ids_by_edge(edge) for edge in edges], [])
        # return self._ids_by_edge.get(edges, []) or []

    # OVERRIDE!!!
    def get_inflow_rate(self, time_span):
        """Return the inflow rate (in veh/hr) of vehicles from the network.

        This value is computed over the specified **time_span** seconds.
        """
        # if len(self._num_departed) == 0:
        #     return 0
        # num_inflow = self._num_departed[-int(time_span / self.sim_step):]
        # return 3600 * sum(num_inflow) / (len(num_inflow) * self.sim_step)

    # OVERRIDE!!!
    def get_outflow_rate(self, time_span):
        """Return the outflow rate (in veh/hr) of vehicles from the network.

        This value is computed over the specified **time_span** seconds.
        """
        # if len(self._num_arrived) == 0:
        #     return 0
        # num_outflow = self._num_arrived[-int(time_span / self.sim_step):]
        # return 3600 * sum(num_outflow) / (len(num_outflow) * self.sim_step)

    # OVERRIDE!!
    def get_num_arrived(self):
        """Return the number of vehicles that arrived in the last time step."""
        # if len(self._num_arrived) > 0:
        #     return self._num_arrived[-1]
        # else:
        #     return 0

    # OVERIDE!!!
    def get_arrived_ids(self):
        """Return the ids of vehicles that arrived in the last time step."""
        # if len(self._arrived_ids) > 0:
        #     return self._arrived_ids[-1]
        # else:
        #     return 0

    # OVERRIDE!!!
    def get_departed_ids(self):
        """Return the ids of vehicles that departed in the last time step."""
        # if len(self._departed_ids) > 0:
        #     return self._departed_ids[-1]
        # else:
        #     return 0

    # OVERRIDE!!!
    def get_speed(self, veh_id, error=-1001):
        """Return the speed of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_speed(vehID, error) for vehID in veh_id]
        # return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_SPEED, error)

    # OVERRIDE!!!
    def get_default_speed(self, veh_id, error=-1001):
        """Return the expected speed if no control were applied.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_default_speed(vehID, error) for vehID in veh_id]
        # return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_SPEED_WITHOUT_TRACI, error)

    # OVERRIDE!!!
    def get_position(self, veh_id, error=-1001):
        """Return the position of the vehicle relative to its current edge.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_position(vehID, error) for vehID in veh_id]
        # return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_LANEPOSITION, error)

    # OVERRIDE!!!
    def get_edge(self, veh_id, error=""):
        """Return the edge the specified vehicle is currently on.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        str
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_edge(vehID, error) for vehID in veh_id]
        # return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_ROAD_ID, error)

    # OVERRIDE!!!
    def get_lane(self, veh_id, error=-1001):
        """Return the lane index of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        int
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_lane(vehID, error) for vehID in veh_id]
        # return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_LANE_INDEX, error)

    # OVERRIDE!!!
    def get_route(self, veh_id, error=list()):
        """Return the route of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list of str
        """
        # if error is None:
        #     error = list()
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_route(vehID, error) for vehID in veh_id]
        # return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_EDGES, error)

    # OVERRIDE!!!
    def get_length(self, veh_id, error=-1001):
        """Return the length of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_length(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("length", error)

    # OVERRIDE!!!
    def get_leader(self, veh_id, error=""):
        """Return the leader of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        str
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_leader(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("leader", error)

    # OVERRIDE!!!
    def get_follower(self, veh_id, error=""):
        """Return the follower of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        str
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_follower(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("follower", error)

    # OVERRIDE!!!
    def get_headway(self, veh_id, error=-1001):
        """Return the headway of the specified vehicle(s).

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_headway(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("headway", error)

    # OVERRIDE!!!
    def get_last_lc(self, veh_id, error=-1001):
        """Return the last time step a vehicle changed lanes.

        Note: This value is only stored for RL vehicles. All other vehicles
        calling this will cause a warning to be printed and their "last_lc"
        term will be the error value.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        int
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_headway(vehID, error) for vehID in veh_id]
        #
        # if veh_id not in self.__rl_ids:
        #     warnings.warn('Vehicle {} is not RL vehicle, "last_lc" term set to'
        #                   ' {}.'.format(veh_id, error))
        #     return error
        # else:
        #     return self.__vehicles.get(veh_id, {}).get("headway", error)

    # OVERRIDE!!!
    def get_acc_controller(self, veh_id, error=None):
        """Return the acceleration controller of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        object
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_acc_controller(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("acc_controller", error)

    # OVERRIDE!!!
    def get_lane_changing_controller(self, veh_id, error=None):
        """Return the lane changing controller of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        object
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [
        #         self.get_lane_changing_controller(vehID, error)
        #         for vehID in veh_id
        #     ]
        # return self.__vehicles.get(veh_id, {}).get("lane_changer", error)

    # OVERRIDE!!!
    def get_routing_controller(self, veh_id, error=None):
        """Return the routing controller of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        object
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [
        #         self.get_routing_controller(vehID, error) for vehID in veh_id
        #     ]
        # return self.__vehicles.get(veh_id, {}).get("router", error)

    # OVERRIDE!!!
    def get_lane_headways(self, veh_id, error=list()):
        """Return the lane headways of the specified vehicles.

        This includes the headways between the specified vehicle and the
        vehicle immediately ahead of it in all lanes.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list of float
        """
        # if error is None:
        #     error = list()
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_lane_headways(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("lane_headways", error)

    # OVERRIDE!!!
    def get_lane_leaders_speed(self, veh_id, error=list()):
        """Return the speed of the leaders of the specified vehicles.

        This includes the speed between the specified vehicle and the
        vehicle immediately ahead of it in all lanes.

        Missing lead vehicles have a speed of zero.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list of float
        """
        # lane_leaders = self.get_lane_leaders(veh_id)
        # return [0 if lane_leader == '' else self.get_speed(lane_leader)
        #         for lane_leader in lane_leaders]

    # OVERRIDE!!!
    def get_lane_followers_speed(self, veh_id, error=list()):
        """Return the speed of the followers of the specified vehicles.

        This includes the speed between the specified vehicle and the
        vehicle immediately behind it in all lanes.

        Missing following vehicles have a speed of zero.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list of float
        """
        # lane_followers = self.get_lane_followers(veh_id)
        # return [0 if lane_follower == '' else self.get_speed(lane_follower)
        #         for lane_follower in lane_followers]

    # OVERRIDE!!!
    def get_lane_leaders(self, veh_id, error=list()):
        """Return the leaders for the specified vehicle in all lanes.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        lis of str
        """
        # if error is None:
        #     error = list()
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_lane_leaders(vehID, error) for vehID in veh_id]
        # return self.__vehicles[veh_id]["lane_leaders"]

    # OVERRIDE!!!
    def get_lane_tailways(self, veh_id, error=list()):
        """Return the lane tailways of the specified vehicle.

        This includes the headways between the specified vehicle and the
        vehicle immediately behind it in all lanes.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list of float
        """
        # if error is None:
        #     error = list()
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_lane_tailways(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("lane_tailways", error)

    # OVERRIDE!!!
    def get_lane_followers(self, veh_id, error=list()):
        """Return the followers for the specified vehicle in all lanes.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : list, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list of str
        """
        # if error is None:
        #     error = list()
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_lane_followers(vehID, error) for vehID in veh_id]
        # return self.__vehicles.get(veh_id, {}).get("lane_followers", error)

    # OVERRIDE!!!
    def get_x_by_id(self, veh_id):
        """Provide a 1-D representation of the position of a vehicle.

        Note: These values are only meaningful if the specify_edge_starts
        method in the network is set appropriately; otherwise, a value of 0 is
        returned for all vehicles.

        Parameters
        ----------
        veh_id : str
            vehicle identifier

        Returns
        -------
        float
        """
        # if self.get_edge(veh_id) == '':
        #     # occurs when a vehicle crashes is teleported for some other reason
        #     return 0.
        # return self.master_kernel.network.get_x(
        #     self.get_edge(veh_id), self.get_position(veh_id))

    # OVERRIDE!!!
    def get_max_speed(self, veh_id, error):
        """Return the max speed of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list of str
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        # if isinstance(veh_id, (list, np.ndarray)):
        #     return [self.get_max_speed(vehID, error) for vehID in veh_id]
        # return self.kernel_api.vehicle.getMaxSpeed(veh_id)


# NOTE: THIS IS NOT IN THE BASE CLASS. WHERE IS IT CALLED?
# def initialize(self, vehicles):
#     """Initialize vehicle state information.
#
#     This is responsible for collecting vehicle type information from the
#     VehicleParams object and placing them within the Vehicles kernel.
#
#     Parameters
#     ----------
#     vehicles : flow.core.params.VehicleParams
#         initial vehicle parameter information, including the types of
#         individual vehicles and their initial speeds
#     """
#     self.type_parameters = vehicles.type_parameters
#     self.minGap = vehicles.minGap
#     self.num_vehicles = 0
#     self.num_rl_vehicles = 0
#
#     self.__vehicles.clear()
#     for typ in vehicles.initial:
#         for i in range(typ['num_vehicles']):
#             veh_id = '{}_{}'.format(typ['veh_id'], i)
#             self.__vehicles[veh_id] = dict()
#             self.__vehicles[veh_id]['type'] = typ['veh_id']
#             self.__vehicles[veh_id]['initial_speed'] = typ['initial_speed']
#             self.num_vehicles += 1
#             if typ['acceleration_controller'][0] == RLController:
#                 self.num_rl_vehicles += 1


# def _add_departed(self, veh_id, veh_type):
#         """Add a vehicle that entered the network from an inflow or reset.
#
#         Parameters
#         ----------
#         veh_id: str
#             name of the vehicle
#         veh_type: str
#             type of vehicle, as specified to sumo
#
#         Returns
#         -------
#         dict
#             subscription results from the new vehicle
#         """
#         if veh_type not in self.type_parameters:
#             raise KeyError("Entering vehicle is not a valid type.")
#
#         if veh_id not in self.__ids:
#             self.__ids.append(veh_id)
#         if veh_id not in self.__vehicles:
#             self.num_vehicles += 1
#             self.__vehicles[veh_id] = dict()
#
#         # specify the type
#         self.__vehicles[veh_id]["type"] = veh_type
#
#         car_following_params = \
#             self.type_parameters[veh_type]["car_following_params"]
#
#         # specify the acceleration controller class
#         accel_controller = \
#             self.type_parameters[veh_type]["acceleration_controller"]
#         self.__vehicles[veh_id]["acc_controller"] = \
#             accel_controller[0](veh_id,
#                                 car_following_params=car_following_params,
#                                 **accel_controller[1])
#
#         # specify the lane-changing controller class
#         lc_controller = \
#             self.type_parameters[veh_type]["lane_change_controller"]
#         self.__vehicles[veh_id]["lane_changer"] = \
#             lc_controller[0](veh_id=veh_id, **lc_controller[1])
#
#         # specify the routing controller class
#         rt_controller = self.type_parameters[veh_type]["routing_controller"]
#         if rt_controller is not None:
#             self.__vehicles[veh_id]["router"] = \
#                 rt_controller[0](veh_id=veh_id, router_params=rt_controller[1])
#         else:
#             self.__vehicles[veh_id]["router"] = None
#
#         # add the vehicle's id to the list of vehicle ids
#         if accel_controller[0] == RLController:
#             if veh_id not in self.__rl_ids:
#                 self.__rl_ids.append(veh_id)
#                 self.num_rl_vehicles += 1
#         else:
#             if veh_id not in self.__human_ids:
#                 self.__human_ids.append(veh_id)
#                 if accel_controller[0] != SimCarFollowingController:
#                     self.__controlled_ids.append(veh_id)
#                 if lc_controller[0] != SimLaneChangeController:
#                     self.__controlled_lc_ids.append(veh_id)
#
#         # subscribe the new vehicle
#         self.kernel_api.vehicle.subscribe(veh_id, [
#             tc.VAR_LANE_INDEX, tc.VAR_LANEPOSITION, tc.VAR_ROAD_ID,
#             tc.VAR_SPEED, tc.VAR_EDGES, tc.VAR_POSITION, tc.VAR_ANGLE,
#             tc.VAR_SPEED_WITHOUT_TRACI
#         ])
#         self.kernel_api.vehicle.subscribeLeader(veh_id, 2000)
#
#         # some constant vehicle parameters to the vehicles class
#         self.__vehicles[veh_id]["length"] = self.kernel_api.vehicle.getLength(
#             veh_id)
#
#         # set the "last_lc" parameter of the vehicle
#         self.__vehicles[veh_id]["last_lc"] = -float("inf")
#
#         # specify the initial speed
#         self.__vehicles[veh_id]["initial_speed"] = \
#             self.type_parameters[veh_type]["initial_speed"]
#
#         # set the speed mode for the vehicle
#         speed_mode = self.type_parameters[veh_type][
#             "car_following_params"].speed_mode
#         self.kernel_api.vehicle.setSpeedMode(veh_id, speed_mode)
#
#         # set the lane changing mode for the vehicle
#         lc_mode = self.type_parameters[veh_type][
#             "lane_change_params"].lane_change_mode
#         self.kernel_api.vehicle.setLaneChangeMode(veh_id, lc_mode)
#
#         # get initial state info
#         self.__sumo_obs[veh_id] = dict()
#         self.__sumo_obs[veh_id][tc.VAR_ROAD_ID] = \
#             self.kernel_api.vehicle.getRoadID(veh_id)
#         self.__sumo_obs[veh_id][tc.VAR_LANEPOSITION] = \
#             self.kernel_api.vehicle.getLanePosition(veh_id)
#         self.__sumo_obs[veh_id][tc.VAR_LANE_INDEX] = \
#             self.kernel_api.vehicle.getLaneIndex(veh_id)
#         self.__sumo_obs[veh_id][tc.VAR_SPEED] = \
#             self.kernel_api.vehicle.getSpeed(veh_id)
#
#         # make sure that the order of rl_ids is kept sorted
#         self.__rl_ids.sort()
#
#         # get the subscription results from the new vehicle
#         new_obs = self.kernel_api.vehicle.getSubscriptionResults(veh_id)
#
#         return new_obs

    # NOT IN BASE CLASS. WHO CALLS THIS??
    # def get_initial_speed(self, veh_id):
    #     """Return the initial speed of the vehicle of veh_id."""
    #     return self.__vehicles[veh_id]["initial_speed"]


    # NOT IN BASE CLASS. WHO CALLS THIS??
    # def set_lane_headways(self, veh_id, lane_headways):
    #     """Set the lane headways of the specified vehicle."""
    #     self.__vehicles[veh_id]["lane_headways"] = lane_headways

    # NOT IN BASE CLASS. WHO CALLS THIS??
    # def set_lane_leaders(self, veh_id, lane_leaders):
    #     """Set the lane leaders of the specified vehicle."""
    #     self.__vehicles[veh_id]["lane_leaders"] = lane_leaders

    # NOT IN BASE CLASS. WHO CALLS THIS??
    # def set_lane_tailways(self, veh_id, lane_tailways):
    #     """Set the lane tailways of the specified vehicle."""
    #     self.__vehicles[veh_id]["lane_tailways"] = lane_tailways

    # NOT IN BASE CLASS. WHO CALLS THIS??
    # def set_lane_followers(self, veh_id, lane_followers):
    #     """Set the lane followers of the specified vehicle."""
    #     self.__vehicles[veh_id]["lane_followers"] = lane_followers

    # NOT IN BASE CLASS. WHO CALLS THIS??
    # def _multi_lane_headways(self):
    #     """Compute multi-lane data for all vehicles.
    #
    #     This includes the lane leaders/followers/headways/tailways/
    #     leader velocity/follower velocity for all
    #     vehicles in the network.
    #     """
    #     edge_list = self.master_kernel.network.get_edge_list()
    #     junction_list = self.master_kernel.network.get_junction_list()
    #     tot_list = edge_list + junction_list
    #     num_edges = (len(self.master_kernel.network.get_edge_list()) + len(
    #         self.master_kernel.network.get_junction_list()))
    #
    #     # maximum number of lanes in the network
    #     max_lanes = max([self.master_kernel.network.num_lanes(edge_id)
    #                      for edge_id in tot_list])
    #
    #     # Key = edge id
    #     # Element = list, with the ith element containing tuples with the name
    #     #           and position of all vehicles in lane i
    #     edge_dict = dict.fromkeys(tot_list)
    #
    #     # add the vehicles to the edge_dict element
    #     for veh_id in self.get_ids():
    #         edge = self.get_edge(veh_id)
    #         lane = self.get_lane(veh_id)
    #         pos = self.get_position(veh_id)
    #         if edge:
    #             if edge_dict[edge] is None:
    #                 edge_dict[edge] = [[] for _ in range(max_lanes)]
    #             edge_dict[edge][lane].append((veh_id, pos))
    #
    #     # sort all lanes in each edge by position
    #     for edge in tot_list:
    #         if edge_dict[edge] is None:
    #             del edge_dict[edge]
    #         else:
    #             for lane in range(max_lanes):
    #                 edge_dict[edge][lane].sort(key=lambda x: x[1])
    #
    #     for veh_id in self.get_rl_ids():
    #         # collect the lane leaders, followers, headways, and tailways for
    #         # each vehicle
    #         edge = self.get_edge(veh_id)
    #         if edge:
    #             headways, tailways, leaders, followers = \
    #                 self._multi_lane_headways_util(veh_id, edge_dict,
    #                                                num_edges)
    #
    #             # add the above values to the vehicles class
    #             self.set_lane_headways(veh_id, headways)
    #             self.set_lane_tailways(veh_id, tailways)
    #             self.set_lane_leaders(veh_id, leaders)
    #             self.set_lane_followers(veh_id, followers)
    #
    #     self._ids_by_edge = dict().fromkeys(edge_list)
    #
    #     for edge_id in edge_dict:
    #         edges = list(itertools.chain.from_iterable(edge_dict[edge_id]))
    #         # check for edges with no vehicles
    #         if len(edges) > 0:
    #             edges, _ = zip(*edges)
    #             self._ids_by_edge[edge_id] = list(edges)
    #         else:
    #             self._ids_by_edge[edge_id] = []

    # def _multi_lane_headways_util(self, veh_id, edge_dict, num_edges):
    #     """Compute multi-lane data for the specified vehicle.
    #
    #     Parameters
    #     ----------
    #     veh_id : str
    #         name of the vehicle
    #     edge_dict : dict < list<tuple> >
    #         Key = Edge name
    #             Index = lane index
    #             Element = list sorted by position of (vehicle id, position)
    #
    #     Returns
    #     -------
    #     headway : list<float>
    #         Index = lane index
    #         Element = headway at this lane
    #     tailway : list<float>
    #         Index = lane index
    #         Element = tailway at this lane
    #     lead_speed : list<str>
    #         Index = lane index
    #         Element = speed of leader at this lane
    #     follow_speed : list<str>
    #         Index = lane index
    #         Element = speed of follower at this lane
    #     leader : list<str>
    #         Index = lane index
    #         Element = leader at this lane
    #     follower : list<str>
    #         Index = lane index
    #         Element = follower at this lane
    #     """
    #     this_pos = self.get_position(veh_id)
    #     this_edge = self.get_edge(veh_id)
    #     this_lane = self.get_lane(veh_id)
    #     num_lanes = self.master_kernel.network.num_lanes(this_edge)
    #
    #     # set default values for all output values
    #     headway = [1000] * num_lanes
    #     tailway = [1000] * num_lanes
    #     leader = [""] * num_lanes
    #     follower = [""] * num_lanes
    #
    #     for lane in range(num_lanes):
    #         # check the vehicle's current  edge for lane leaders and followers
    #         if len(edge_dict[this_edge][lane]) > 0:
    #             ids, positions = zip(*edge_dict[this_edge][lane])
    #             ids = list(ids)
    #             positions = list(positions)
    #             index = bisect_left(positions, this_pos)
    #
    #             # if you are at the end or the front of the edge, the lane
    #             # leader is in the edges in front of you
    #             if (lane == this_lane and index < len(positions) - 1) \
    #                     or (lane != this_lane and index < len(positions)):
    #                 # check if the index does not correspond to the current
    #                 # vehicle
    #                 if ids[index] == veh_id:
    #                     leader[lane] = ids[index + 1]
    #                     headway[lane] = (positions[index + 1] - this_pos -
    #                                      self.get_length(leader[lane]))
    #                 else:
    #                     leader[lane] = ids[index]
    #                     headway[lane] = (positions[index] - this_pos
    #                                      - self.get_length(leader[lane]))
    #
    #             # you are in the back of the queue, the lane follower is in the
    #             # edges behind you
    #             if index > 0:
    #                 follower[lane] = ids[index - 1]
    #                 tailway[lane] = (this_pos - positions[index - 1]
    #                                  - self.get_length(veh_id))
    #
    #         # if lane leader not found, check next edges
    #         if leader[lane] == "":
    #             headway[lane], leader[lane] = self._next_edge_leaders(
    #                 veh_id, edge_dict, lane, num_edges)
    #
    #         # if lane follower not found, check previous edges
    #         if follower[lane] == "":
    #             tailway[lane], follower[lane] = self._prev_edge_followers(
    #                 veh_id, edge_dict, lane, num_edges)
    #
    #     return headway, tailway, leader, follower

    # def _next_edge_leaders(self, veh_id, edge_dict, lane, num_edges):
    #     """Search for leaders in the next edge.
    #
    #     Looks to the edges/junctions in front of the vehicle's current edge
    #     for potential leaders. This is currently done by only looking one
    #     edge/junction forwards.
    #
    #     Returns
    #     -------
    #     headway : float
    #         lane headway for the specified lane
    #     leader : str
    #         lane leader for the specified lane
    #     """
    #     pos = self.get_position(veh_id)
    #     edge = self.get_edge(veh_id)
    #
    #     headway = 1000  # env.network.length
    #     leader = ""
    #     add_length = 0  # length increment in headway
    #
    #     for _ in range(num_edges):
    #         # break if there are no edge/lane pairs behind the current one
    #         if len(self.master_kernel.network.next_edge(edge, lane)) == 0:
    #             break
    #
    #         add_length += self.master_kernel.network.edge_length(edge)
    #         edge, lane = self.master_kernel.network.next_edge(edge, lane)[0]
    #
    #         try:
    #             if len(edge_dict[edge][lane]) > 0:
    #                 leader = edge_dict[edge][lane][0][0]
    #                 headway = edge_dict[edge][lane][0][1] - pos + add_length \
    #                     - self.get_length(leader)
    #         except KeyError:
    #             # current edge has no vehicles, so move on
    #             # print(traceback.format_exc())
    #             continue
    #
    #         # stop if a lane follower is found
    #         if leader != "":
    #             break
    #
    #     return headway, leader

    # def _prev_edge_followers(self, veh_id, edge_dict, lane, num_edges):
    #     """Search for followers in the previous edge.
    #
    #     Looks to the edges/junctions behind the vehicle's current edge for
    #     potential followers. This is currently done by only looking one
    #     edge/junction backwards.
    #
    #     Returns
    #     -------
    #     tailway : float
    #         lane tailway for the specified lane
    #     follower : str
    #         lane follower for the specified lane
    #     """
    #     pos = self.get_position(veh_id)
    #     edge = self.get_edge(veh_id)
    #
    #     tailway = 1000  # env.network.length
    #     follower = ""
    #     add_length = 0  # length increment in headway
    #
    #     for _ in range(num_edges):
    #         # break if there are no edge/lane pairs behind the current one
    #         if len(self.master_kernel.network.prev_edge(edge, lane)) == 0:
    #             break
    #
    #         edge, lane = self.master_kernel.network.prev_edge(edge, lane)[0]
    #         add_length += self.master_kernel.network.edge_length(edge)
    #
    #         try:
    #             if len(edge_dict[edge][lane]) > 0:
    #                 tailway = pos - edge_dict[edge][lane][-1][1] + add_length \
    #                           - self.get_length(veh_id)
    #                 follower = edge_dict[edge][lane][-1][0]
    #         except KeyError:
    #             # current edge has no vehicles, so move on
    #             # print(traceback.format_exc())
    #             continue
    #
    #         # stop if a lane follower is found
    #         if follower != "":
    #             break
    #
    #     return tailway, follower

