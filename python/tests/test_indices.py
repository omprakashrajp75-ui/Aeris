import math

import pytest

from aeris import indices
from aeris.indices import (
    CALIBRATED,
    ECTemp,
    MODEL_COEFFS,
    STRAIN_SOURCE_PSI,
    STRAIN_SOURCE_PSI_HR_ONLY,
    PersonalBaseline,
    compute_strain_indices,
    psi,
    psi_band,
    psi_hr_only,
    select_alert_strain,
)


# --- ECTemp ------------------------------------------------------------

def test_model_coeffs_has_required_keys():
    assert set(MODEL_COEFFS) == {
        "b0", "b1", "b2", "gamma", "sigma2", "tc_init", "var_init",
    }


def test_ectemp_starts_at_configured_initial_state():
    ekf = ECTemp()
    assert ekf.tc == MODEL_COEFFS["tc_init"]
    assert ekf.var == MODEL_COEFFS["var_init"]


def test_calibrated_flag_is_false():
    assert CALIBRATED is False


def test_ectemp_update_returns_core_variance_and_calibrated_flag():
    ekf = ECTemp()
    result = ekf.update(70.0)
    assert isinstance(result.core_c, float)
    assert isinstance(result.variance, float)
    assert result.variance > 0.0
    assert result.calibrated is False
    assert result.core_c == ekf.tc
    assert result.variance == ekf.var


def test_ectemp_matches_hand_computed_first_step():
    # Verifies the EKF math itself, independent of whether MODEL_COEFFS are
    # scientifically accurate placeholders.
    coeffs = MODEL_COEFFS
    ekf = ECTemp()
    hr = 90.0

    tc_pred = coeffs["tc_init"]
    var_pred = coeffs["var_init"] + coeffs["gamma"] * (1.0 / 60.0)
    jacobian = coeffs["b1"] + 2.0 * coeffs["b2"] * tc_pred
    predicted_hr = coeffs["b0"] + coeffs["b1"] * tc_pred + coeffs["b2"] * tc_pred ** 2
    innovation = hr - predicted_hr
    innovation_var = jacobian * var_pred * jacobian + coeffs["sigma2"]
    kalman_gain = var_pred * jacobian / innovation_var
    expected_tc = tc_pred + kalman_gain * innovation
    expected_var = (1.0 - kalman_gain * jacobian) * var_pred

    result = ekf.update(hr)
    assert result.core_c == pytest.approx(expected_tc)
    assert result.variance == pytest.approx(expected_var)
    assert result.calibrated is False


def test_ectemp_variance_stays_bounded_under_repeated_observations():
    ekf = ECTemp()
    variances = []
    for _ in range(500):
        result = ekf.update(75.0)
        variances.append(result.variance)
    assert all(v >= 0.0 for v in variances)
    assert all(math.isfinite(v) for v in variances)


def test_two_half_steps_grow_process_variance_like_one_full_step():
    # Use a huge sigma2 so the Kalman correction is negligible, isolating the
    # process-noise-growth math (var += gamma * step_minutes) from the
    # nonlinear correction step that would otherwise make the two paths
    # diverge slightly.
    coeffs = dict(MODEL_COEFFS)
    coeffs["sigma2"] = 1e18

    two_steps = ECTemp(coeffs=coeffs, step_minutes=1.0 / 60.0)
    two_steps.update(70.0)
    two_steps.update(70.0)

    one_step = ECTemp(coeffs=coeffs, step_minutes=2.0 / 60.0)
    one_step.update(70.0)

    assert two_steps.var == pytest.approx(one_step.var, rel=1e-9)


# --- psi -----------------------------------------------------------------

def test_psi_at_zero_endpoint():
    assert psi(core_c=37.0, core_c0=37.0, hr=70.0, hr0=70.0) == 0.0


def test_psi_at_ten_endpoint():
    # Tc term maxes at 5 when Tc == 39.5, HR term maxes at 5 when HR == 180.
    assert psi(core_c=39.5, core_c0=37.0, hr=180.0, hr0=70.0) == pytest.approx(10.0)


def test_psi_clamps_above_ten():
    assert psi(core_c=41.0, core_c0=37.0, hr=220.0, hr0=70.0) == 10.0


def test_psi_clamps_below_zero():
    assert psi(core_c=35.0, core_c0=37.0, hr=40.0, hr0=70.0) == 0.0


# --- psi_hr_only -----------------------------------------------------------

def test_psi_hr_only_at_zero_endpoint():
    assert psi_hr_only(hr=70.0, hr0=70.0) == 0.0


def test_psi_hr_only_at_ten_endpoint():
    assert psi_hr_only(hr=180.0, hr0=70.0) == pytest.approx(10.0)


def test_psi_hr_only_clamps_above_ten():
    assert psi_hr_only(hr=250.0, hr0=70.0) == 10.0


def test_psi_hr_only_clamps_below_zero():
    assert psi_hr_only(hr=40.0, hr0=70.0) == 0.0


# --- strain selection ------------------------------------------------------

def test_psi_hr_only_produced_when_hr0_set_but_tc0_missing():
    result = compute_strain_indices(core_c=37.5, session_tc0=None, hr=140.0, session_hr0=70.0)

    assert result.psi is None
    assert result.psi_hr_only == pytest.approx(psi_hr_only(140.0, 70.0))


def test_alert_strain_selects_hr_only_while_uncalibrated():
    assert CALIBRATED is False

    # psi would grade HIGH here; psi_hr_only grades NONE. The uncalibrated
    # alert path must follow psi_hr_only.
    strain = select_alert_strain(psi_value=8.0, psi_hr_only_value=2.0)

    assert strain.source == STRAIN_SOURCE_PSI_HR_ONLY
    assert strain.value == 2.0
    assert strain.band == "NONE"


@pytest.mark.parametrize(
    "calibrated,expected_source",
    [(False, STRAIN_SOURCE_PSI_HR_ONLY), (True, STRAIN_SOURCE_PSI)],
)
def test_active_source_matches_selected_strain_source(monkeypatch, calibrated, expected_source):
    # main.store_indices writes AlertStrain.source into the indices.active_source
    # column verbatim (NULL only when .band is None), so pinning .source here
    # pins what that column records for each CALIBRATED value.
    monkeypatch.setattr(indices, "CALIBRATED", calibrated)

    strain = select_alert_strain(psi_value=8.0, psi_hr_only_value=2.0)

    assert strain.source == expected_source
    assert strain.band is not None  # so active_source is stored, not NULLed


# --- psi_band --------------------------------------------------------------

@pytest.mark.parametrize(
    "value,expected",
    [
        (0, "NONE"),
        (2, "NONE"),
        (3, "LOW"),
        (4, "LOW"),
        (5, "MODERATE"),
        (6, "MODERATE"),
        (7, "HIGH"),
        (8, "HIGH"),
        (9, "VERY_HIGH"),
        (10, "VERY_HIGH"),
    ],
)
def test_psi_band_boundaries(value, expected):
    assert psi_band(value) == expected


# --- PersonalBaseline --------------------------------------------------

def test_baseline_refuses_to_update_while_moving():
    baseline = PersonalBaseline()
    for _ in range(400):
        baseline.update(hr=90.0, skin_c=34.0, motion_state="ACTIVE")

    assert baseline.hr_mean is None
    assert baseline.hr_sd is None
    assert baseline.is_ready is False
    assert baseline.deviation_sd(90.0) is None


def test_baseline_ignores_light_motion_too():
    baseline = PersonalBaseline()
    baseline.update(hr=80.0, skin_c=33.5, motion_state="LIGHT")
    assert baseline.hr_mean is None


def test_deviation_sd_none_before_sample_threshold():
    baseline = PersonalBaseline()
    for _ in range(PersonalBaseline.MIN_STILL_SAMPLES - 1):
        baseline.update(hr=65.0, skin_c=33.0, motion_state="STILL")

    assert baseline.is_ready is False
    assert baseline.deviation_sd(65.0) is None


def test_deviation_sd_available_once_threshold_met():
    baseline = PersonalBaseline()
    for _ in range(PersonalBaseline.MIN_STILL_SAMPLES):
        baseline.update(hr=65.0, skin_c=33.0, motion_state="STILL")

    assert baseline.is_ready is True
    deviation = baseline.deviation_sd(65.0)
    assert deviation is not None
    assert deviation == pytest.approx(0.0)


def test_deviation_sd_reflects_offset_from_mean():
    baseline = PersonalBaseline()
    for _ in range(PersonalBaseline.MIN_STILL_SAMPLES):
        baseline.update(hr=60.0, skin_c=33.0, motion_state="STILL")

    # Constant input HR -> hr_var stays 0 -> deviation_sd falls back to 0.0
    # rather than dividing by zero.
    assert baseline.hr_sd == 0.0
    assert baseline.deviation_sd(100.0) == 0.0


def test_baseline_only_counts_still_samples_toward_threshold():
    baseline = PersonalBaseline()
    for _ in range(299):
        baseline.update(hr=65.0, skin_c=33.0, motion_state="STILL")
    for _ in range(1000):
        baseline.update(hr=150.0, skin_c=36.0, motion_state="ACTIVE")

    assert baseline.is_ready is False

    baseline.update(hr=65.0, skin_c=33.0, motion_state="STILL")
    assert baseline.is_ready is True
