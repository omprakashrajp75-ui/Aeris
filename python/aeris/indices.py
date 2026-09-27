"""Pure computation functions for aeris's physiological indices.

No I/O, no globals beyond static configuration, no database access. Every
function/class here takes its inputs as arguments and returns its outputs as
values, so it can be exercised directly from tests/test_indices.py without any
board, Bridge, or SQLite dependency.
"""

from collections import namedtuple

MODEL_COEFFS = {
    # TODO: replace with values read directly from Buller et al. 2013 /
    # the USARIEM reference implementation at usariem.health.mil before any
    # accuracy claim is made. These are placeholders.
    "b0": 1698.0, "b1": -118.0, "b2": 2.0,   # quadratic HR <-> Tc observation model
    "gamma": 0.0004,                          # process variance, degC^2 per minute
    "sigma2": 25.0,                           # observation variance, bpm^2
    "tc_init": 37.1, "var_init": 0.0,
}

# Flip to True only once every coefficient above has been replaced with a
# value read directly from Buller et al. 2013 / the USARIEM reference
# implementation. Until then, ECTemp.update() reports calibrated=False on
# every call so downstream consumers (dashboards, alerts) can't mistake the
# placeholder core temperature for a real one.
CALIBRATED = False

# Default interval assumed between ECTemp.update() calls when the caller
# doesn't say otherwise. The rest of this app currently polls at 1
# reading/second (see python/main.py).
DEFAULT_STEP_MINUTES = 1.0 / 60.0

ECTempEstimate = namedtuple("ECTempEstimate", ("core_c", "variance", "calibrated"))

# Which strain figure the alert path grades on. Recorded per-alert so a stored
# alert can always be traced back to the number that actually triggered it.
STRAIN_SOURCE_PSI = "psi"
STRAIN_SOURCE_PSI_HR_ONLY = "psi_hr_only"
# A band graded by the M33 itself, off cached raw-HR thresholds, while Python
# was unreachable (see sketch.ino's autonomous mode and main.py's
# drain_autonomous_log handling). Listed here alongside the two sources above
# only as documentation of the full set of values these columns can hold -
# nothing in this file produces it.
STRAIN_SOURCE_AUTONOMOUS_M33 = "autonomous_m33"

StrainIndices = namedtuple("StrainIndices", ("psi", "psi_hr_only"))
AlertStrain = namedtuple("AlertStrain", ("band", "source", "value"))


class ECTemp:
    """Extended Kalman filter estimating core body temperature (Tc) from a
    sequence of heart-rate observations, following the ECTemp algorithm
    structure described in Buller et al. 2013 (USARIEM).

    State: scalar core temperature Tc (degC), modeled as a random walk
    between observations. Observation model: HR is a quadratic function of
    Tc, HR = b0 + b1*Tc + b2*Tc^2, linearized at each step via its Jacobian
    for the EKF correction.
    """

    def __init__(self, coeffs=MODEL_COEFFS, step_minutes=DEFAULT_STEP_MINUTES):
        self._b0 = coeffs["b0"]
        self._b1 = coeffs["b1"]
        self._b2 = coeffs["b2"]
        self._gamma = coeffs["gamma"]
        self._sigma2 = coeffs["sigma2"]
        self._step_minutes = step_minutes
        self.tc = coeffs["tc_init"]
        self.var = coeffs["var_init"]

    def _predicted_hr(self, tc):
        return self._b0 + self._b1 * tc + self._b2 * tc * tc

    def _jacobian(self, tc):
        return self._b1 + 2.0 * self._b2 * tc

    def update(self, hr):
        """Fold in one heart-rate observation, `step_minutes` (as passed to
        the constructor) after the previous one. Returns an ECTempEstimate
        (core_c, variance, calibrated)."""
        # Predict
        tc_pred = self.tc
        var_pred = self.var + self._gamma * self._step_minutes

        # Update (linearized around the predicted state)
        jacobian = self._jacobian(tc_pred)
        innovation = hr - self._predicted_hr(tc_pred)
        innovation_var = jacobian * var_pred * jacobian + self._sigma2
        kalman_gain = var_pred * jacobian / innovation_var

        self.tc = tc_pred + kalman_gain * innovation
        self.var = (1.0 - kalman_gain * jacobian) * var_pred

        return ECTempEstimate(self.tc, self.var, CALIBRATED)


def psi(core_c, core_c0, hr, hr0):
    """Physiological Strain Index (Moran, Shitzer & Pandolf 1998), clamped
    to the defined 0-10 range."""
    value = 5.0 * (core_c - core_c0) / (39.5 - core_c0) + 5.0 * (hr - hr0) / (180.0 - hr0)
    if value < 0.0:
        return 0.0
    if value > 10.0:
        return 10.0
    return value


def psi_hr_only(hr, hr0):
    """The heart-rate term of PSI alone, scaled to 0-10 (clamped). Stays
    meaningful while the core-temperature model is uncalibrated (see
    CALIBRATED), since it doesn't depend on ECTemp at all."""
    value = 10.0 * (hr - hr0) / (180.0 - hr0)
    if value < 0.0:
        return 0.0
    if value > 10.0:
        return 10.0
    return value


def psi_band(psi_value):
    """Map a (0-10) PSI value to its qualitative strain band."""
    if psi_value <= 2:
        return "NONE"
    if psi_value <= 4:
        return "LOW"
    if psi_value <= 6:
        return "MODERATE"
    if psi_value <= 8:
        return "HIGH"
    return "VERY_HIGH"


def compute_strain_indices(core_c, session_tc0, hr, session_hr0):
    """Both strain figures for one observation, each gated on only the session
    references it actually needs: full PSI needs Tc_0 and HR_0, while the
    HR-only term needs HR_0 alone. Either may be None; returns StrainIndices."""
    psi_value = None
    if session_tc0 is not None and session_hr0 is not None:
        psi_value = psi(core_c, session_tc0, hr, session_hr0)

    psi_hr_only_value = None
    if session_hr0 is not None:
        psi_hr_only_value = psi_hr_only(hr, session_hr0)

    return StrainIndices(psi_value, psi_hr_only_value)


def select_alert_strain(psi_value, psi_hr_only_value):
    """Pick the strain figure the alert path grades on. While CALIBRATED is
    False the full PSI is derived from placeholder ECTemp coefficients, so
    alerts grade on the HR-only term instead, which needs no core temperature.
    Returns AlertStrain; .band is None when the chosen value is unavailable."""
    if CALIBRATED:
        value, source = psi_value, STRAIN_SOURCE_PSI
    else:
        value, source = psi_hr_only_value, STRAIN_SOURCE_PSI_HR_ONLY

    return AlertStrain(psi_band(value) if value is not None else None, source, value)


class PersonalBaseline:
    """EWMA-based resting heart-rate / skin-temperature baseline.

    Only updated while the wearer is stationary (motion_state == "STILL"),
    so the baseline reflects true resting physiology rather than active
    movement. Needs MIN_STILL_SAMPLES stillness observations before it is
    considered reliable.
    """

    MIN_STILL_SAMPLES = 300
    ALPHA = 0.02  # EWMA smoothing factor

    def __init__(self):
        self.hr_mean = None
        self.hr_var = None
        self.skin_mean = None
        self._still_samples = 0

    @property
    def is_ready(self):
        return self._still_samples >= self.MIN_STILL_SAMPLES

    @property
    def still_samples(self):
        """Stillness samples collected so far (read-only, for the dashboard)."""
        return self._still_samples

    @property
    def samples_until_ready(self):
        """How many more stillness samples are needed before is_ready."""
        return max(0, self.MIN_STILL_SAMPLES - self._still_samples)

    @property
    def hr_sd(self):
        if self.hr_var is None:
            return None
        return self.hr_var ** 0.5

    def update(self, hr, skin_c, motion_state):
        if motion_state != "STILL":
            return

        if self.hr_mean is None:
            self.hr_mean = hr
            self.hr_var = 0.0
            self.skin_mean = skin_c
        else:
            delta = hr - self.hr_mean
            self.hr_mean += self.ALPHA * delta
            self.hr_var = (1.0 - self.ALPHA) * (self.hr_var + self.ALPHA * delta * delta)
            self.skin_mean += self.ALPHA * (skin_c - self.skin_mean)

        self._still_samples += 1

    def deviation_sd(self, hr):
        """Current hr's deviation from the resting baseline, in standard
        deviations. None until the baseline has enough stillness samples."""
        if not self.is_ready:
            return None
        sd = self.hr_sd
        if not sd:
            return 0.0
        return (hr - self.hr_mean) / sd
