# Generated by sila2.code_generator; sila2.__version__: 0.12.2
from __future__ import annotations

from typing import Optional

from sila2.framework.errors.defined_execution_error import DefinedExecutionError

from .hiperistaltic_feature import HiPeristalticFeature


class PumpIndexOutOfRange(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "The selected pump channel index is out of range. Must be 1-4."
        super().__init__(HiPeristalticFeature.defined_execution_errors["PumpIndexOutOfRange"], message=message)


class FlowRateOutOfRange(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "The selected flow rate is either negative, too small, or above the maximum possible rate."
        super().__init__(HiPeristalticFeature.defined_execution_errors["FlowRateOutOfRange"], message=message)


class TargetVolumeOutOfRange(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "The selected target volume is either negative, too small, or above the maximum possible volume."
        super().__init__(HiPeristalticFeature.defined_execution_errors["TargetVolumeOutOfRange"], message=message)


class PumpInterrupted(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "Pump operation was interrupted by another command or by an external event."
        super().__init__(HiPeristalticFeature.defined_execution_errors["PumpInterrupted"], message=message)


class InvalidDirection(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "Flow direction of the pump is invalid."
        super().__init__(HiPeristalticFeature.defined_execution_errors["InvalidDirection"], message=message)


class TargetRevolutionsOutOfRange(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = (
                "The selected target number of revolutions is either negative, or above the maximum possible count."
            )
        super().__init__(HiPeristalticFeature.defined_execution_errors["TargetRevolutionsOutOfRange"], message=message)


class RPMOutOfRange(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "The selected RPM is either negative, too small, or above the maximum possible rate."
        super().__init__(HiPeristalticFeature.defined_execution_errors["RPMOutOfRange"], message=message)


class CalibrationParameterOutOfRange(DefinedExecutionError):
    def __init__(self, message: Optional[str] = None):
        if message is None:
            message = "The selected calibration parameter is either negative or above the maximum possible value."
        super().__init__(
            HiPeristalticFeature.defined_execution_errors["CalibrationParameterOutOfRange"], message=message
        )
