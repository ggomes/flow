"""Empty init file to ensure documentation for the vehicle class is created."""

from flow.core.kernel.vehicle.base import KernelVehicle
from flow.core.kernel.vehicle.traci import TraCIVehicle
from flow.core.kernel.vehicle.aimsun import AimsunKernelVehicle
from flow.core.kernel.vehicle.otm import OTMVehicle


__all__ = ['KernelVehicle', 'TraCIVehicle', 'AimsunKernelVehicle', 'OTMVehicle']
