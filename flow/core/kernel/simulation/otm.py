"""Script containing the OTM simulation kernel class."""

from flow.core.kernel.simulation import KernelSimulation
from flow.core.util import ensure_dir
import flow.config as config
import traceback
import os
import time
import logging
import subprocess
import signal


# Number of retries on restarting SUMO before giving up
RETRIES_ON_ERROR = 10


class OTMSimulation(KernelSimulation):
    """OTM simulation kernel.

    Extends flow.core.kernel.simulation.KernelSimulation
    """

    def __init__(self, master_kernel):
        """Instantiate the OTM simulator kernel.

        Parameters
        ----------
        master_kernel : flow.core.kernel.Kernel
            the higher level kernel (used to call methods from other
            sub-kernels)
        """
        KernelSimulation.__init__(self, master_kernel)

    def start_simulation(self, network, sim_params):
        """Start a simulation instance.

        network : any
            an object or variable that is meant to symbolize the network that
            is used during the simulation. For example, in the case of sumo
            simulations, this is (string) the path to the .sumo.cfg file.
        sim_params : flow.core.params.SimParams
            simulation-specific parameters
        """
        # error = None
        # for _ in range(RETRIES_ON_ERROR):
        #     try:
        #         # port number the sumo instance will be run on
        #         port = sim_params.port
        #
        #         sumo_binary = "sumo-gui" if sim_params.render is True \
        #             else "sumo"
        #
        #         # command used to start sumo
        #         sumo_call = [
        #             sumo_binary, "-c", network.cfg,
        #             "--remote-port", str(sim_params.port),
        #             "--num-clients", str(sim_params.num_clients),
        #             "--step-length", str(sim_params.sim_step)
        #         ]
        #
        #         # add step logs (if requested)
        #         if sim_params.no_step_log:
        #             sumo_call.append("--no-step-log")
        #
        #         # add the lateral resolution of the sublanes (if requested)
        #         if sim_params.lateral_resolution is not None:
        #             sumo_call.append("--lateral-resolution")
        #             sumo_call.append(str(sim_params.lateral_resolution))
        #
        #         # add the emission path to the sumo command (if requested)
        #         if sim_params.emission_path is not None:
        #             ensure_dir(sim_params.emission_path)
        #             emission_out = os.path.join(
        #                 sim_params.emission_path,
        #                 "{0}-emission.xml".format(network.name))
        #             sumo_call.append("--emission-output")
        #             sumo_call.append(emission_out)
        #         else:
        #             emission_out = None
        #
        #         if sim_params.overtake_right:
        #             sumo_call.append("--lanechange.overtake-right")
        #             sumo_call.append("true")
        #
        #         # specify a simulation seed (if requested)
        #         if sim_params.seed is not None:
        #             sumo_call.append("--seed")
        #             sumo_call.append(str(sim_params.seed))
        #
        #         if not sim_params.print_warnings:
        #             sumo_call.append("--no-warnings")
        #             sumo_call.append("true")
        #
        #         # set the time it takes for a gridlock teleport to occur
        #         sumo_call.append("--time-to-teleport")
        #         sumo_call.append(str(int(sim_params.teleport_time)))
        #
        #         # check collisions at intersections
        #         sumo_call.append("--collision.check-junctions")
        #         sumo_call.append("true")
        #
        #         logging.info(" Starting SUMO on port " + str(port))
        #         logging.debug(" Cfg file: " + str(network.cfg))
        #         if sim_params.num_clients > 1:
        #             logging.info(" Num clients are" +
        #                          str(sim_params.num_clients))
        #         logging.debug(" Emission file: " + str(emission_out))
        #         logging.debug(" Step length: " + str(sim_params.sim_step))
        #
        #         # Opening the I/O thread to SUMO
        #         self.sumo_proc = subprocess.Popen(
        #             sumo_call, preexec_fn=os.setsid)
        #
        #         # wait a small period of time for the subprocess to activate
        #         # before trying to connect with traci
        #         if os.environ.get("TEST_FLAG", 0):
        #             time.sleep(0.1)
        #         else:
        #             time.sleep(config.SUMO_SLEEP)
        #
        #         traci_connection = traci.connect(port, numRetries=100)
        #         traci_connection.setOrder(0)
        #         traci_connection.simulationStep()
        #
        #         return traci_connection
        #     except Exception as e:
        #         print("Error during start: {}".format(traceback.format_exc()))
        #         error = e
        #         self.teardown_sumo()
        # raise error

    # OVERRIDE!!
    def simulation_step(self):
        """Advance the simulation by one step.

        This is done in most cases by calling a relevant simulator API method.
        """
        # self.kernel_api.simulationStep()

    # OVERRIDE!!
    def update(self, reset):
        """Update the internal attributes of the simulation kernel.

        Any update operations are meant to support ease of simulation in
        current and future steps.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        pass

    # OVERRIDE!!
    def check_collision(self):
        """Determine if a collision occurred in the last time step.

        Returns
        -------
        bool
            True if collision occurred, False otherwise
        """
        # return self.kernel_api.simulation.getStartingTeleportNumber() != 0

    # OVERRIDE!!
    def close(self):
        """Close the current simulation instance."""
        # self.kernel_api.close()
