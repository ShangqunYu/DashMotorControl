"""MIT command scaling ranges.

The firmware packs/unpacks position, velocity, gains, and torque using ranges
that live in the motor's flash. In particular torque is scaled to
±(CFG_I_MAX · CFG_KT · CFG_GR). `MotorScaling` captures those ranges so the GUI
can command and decode with the values that are actually on the motor instead
of the compile-time constants in protocol.
"""

from dataclasses import dataclass

from config import SCALING_PARAM_INDICES


@dataclass(frozen=True)
class MotorScaling:
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    kp_max: float
    kd_max: float
    t_min: float
    t_max: float
    temp_min: float
    temp_max: float

    @classmethod
    def from_params(cls, params):
        """Build scaling from a {param_index: value} dict read off the motor.
        Raises KeyError if a required parameter is missing."""
        gear_ratio = params[1]        # CFG_GR
        torque_constant = params[2]   # CFG_KT  (N·m/A)
        current_limit = params[5]     # CFG_I_MAX
        t_max = current_limit * torque_constant * gear_ratio
        return cls(
            p_min=params[6], p_max=params[7],
            v_min=params[8], v_max=params[9],
            kp_max=params[10], kd_max=params[11],
            t_min=-t_max, t_max=t_max,
            temp_min=params[15], temp_max=params[16],  # CFG_TEMP_MIN / CFG_TEMP_MAX
        )

    @staticmethod
    def has_required(params):
        """True if `params` contains every index needed by from_params()."""
        return all(index in params for index in SCALING_PARAM_INDICES)
