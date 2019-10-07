"""Empty init file to ensure documentation for traffic lights is created."""

from flow.core.kernel.traffic_light.base import KernelTrafficLight
from flow.core.kernel.traffic_light.traci import TraCITrafficLight
from flow.core.kernel.traffic_light.aimsun import AimsunKernelTrafficLight
from flow.core.kernel.traffic_light.otm import OTMTrafficLight

__all__ = ["KernelTrafficLight", "TraCITrafficLight",
           "AimsunKernelTrafficLight", "OTMTrafficLight"]
