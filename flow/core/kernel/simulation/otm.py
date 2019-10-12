"""Script containing the OTM simulation kernel class."""
from py4j.java_gateway import JavaGateway, launch_gateway, GatewayParameters
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

common_gateway = JavaGateway(gateway_parameters=GatewayParameters(
        port=launch_gateway(
            classpath=os.path.join(
                os.path.dirname(__file__),
                '../../../otm-python-api-1.0-SNAPSHOT-jar-with-dependencies.jar'),
            die_on_exit=True, redirect_stdout=sys.stdout
        ), auto_field=True, auto_convert=True
    ))

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
        self.current_time = 0
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
        self.entry_point = common_gateway.jvm.EntryPointOTM()
        self.sim_step = sim_params.sim_step
        return self.entry_point.api

    def simulation_step(self):
        """Advance the simulation by one step.

        This is done in most cases by calling a relevant simulator API method.
        """
        self.kernel_api.run(self.current_time, self.current_time+self.sim_step)
        self.current_time += self.sim_step
