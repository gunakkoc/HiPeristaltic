# Generated by sila2.code_generator; sila2.__version__: 0.12.2
from __future__ import annotations

from typing import NamedTuple


class SetPumpCalibration_Responses(NamedTuple):

    Completed: bool
    """
    Whether the new calibration parameter is applied successfully.
    """


class StartPump_Responses(NamedTuple):

    PumpedVolume: float
    """
    Total pumped volume in microliters until the pump was finished or stopped.
    """


class StartPumpContinuous_Responses(NamedTuple):

    PumpedVolume: float
    """
    Total pumped volume in microliters until the pump was stopped.
    """


class StopPump_Responses(NamedTuple):

    RemainingVolume: float
    """
    The remaining volume in microliters to be pumped at the time of stopping.
    """


class ResumePump_Responses(NamedTuple):

    PumpedVolume: float
    """
    Total pumped volume in microliters until the pump was finished or stopped.
    """


class StartPumpCalibration_Responses(NamedTuple):

    Completed: bool
    """
    Whether the target number of revolution is achieved successfully.
    """


class StartPump_IntermediateResponses(NamedTuple):

    CurrentVolume: float
    """
    Current volume in microliters.
    """


class StartPumpContinuous_IntermediateResponses(NamedTuple):

    CurrentVolume: float
    """
    Current volume in microliters.
    """


class ResumePump_IntermediateResponses(NamedTuple):

    CurrentVolume: float
    """
    Current volume in microliters.
    """


class StartPumpCalibration_IntermediateResponses(NamedTuple):

    CurrentRevolution: float
    """
    Current number of revolution.
    """
