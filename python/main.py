import json
import logging
import os
import sqlite3
import time

from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI

import config
from aeris.indices import (
    CALIBRATED,
    ECTemp,
    PersonalBaseline,
    STRAIN_SOURCE_AUTONOMOUS_M33,
    compute_strain_indices,
    psi_band,
    select_alert_strain,
)
from web.server import push_live, register_web_ui

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("aeris")

APP_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DB_PATH = os.path.join(APP_ROOT, "data", "aeris.db")
RETENTION_SECONDS = 30 * 24 * 60 * 60

os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
db = sqlite3.connect(DB_PATH)
# WAL so the dashboard's reader connections (web/server.py, on the Brick's own
# thread) don't block against this writer.
db.execute("PRAGMA journal_mode=WAL")
db.execute("""
    CREATE TABLE IF NOT EXISTS readings (
        ts            INTEGER PRIMARY KEY,   -- unix epoch seconds
        hr            REAL, hr_valid INTEGER,
        spo2          REAL, spo2_valid INTEGER,
        skin_c        REAL,
        amb_c         REAL,
        rh            REAL,
        motion_g      REAL,
        motion_state  TEXT
    )
""")
db.execute("""
    CREATE TABLE IF NOT EXISTS indices (
        ts        INTEGER PRIMARY KEY,
        core_c    REAL,    -- ECTemp estimate
        core_var  REAL,    -- Kalman variance
        psi       REAL,    -- 0-10
        band      TEXT,    -- NONE|LOW|MODERATE|HIGH|VERY_HIGH
        baseline_hr REAL,
        hr_dev_sd REAL
    )
""")
db.execute("""
    CREATE TABLE IF NOT EXISTS alerts (
        ts          INTEGER PRIMARY KEY,   -- when the alert was raised
        band        TEXT,
        acked_ts    INTEGER,
        false_alarm INTEGER
    )
""")


def _add_column_if_missing(table, column, decl):
    """Guarded ALTER TABLE: safe to run against a pre-existing database that
    predates this column."""
    existing = {row[1] for row in db.execute("PRAGMA table_info({})".format(table)).fetchall()}
    if column not in existing:
        db.execute("ALTER TABLE {} ADD COLUMN {} {}".format(table, column, decl))


# core_calibrated mirrors aeris.indices.CALIBRATED (see ECTemp.update()); until
# that's True, core_c/core_var are illustrative, not a real temperature.
_add_column_if_missing("indices", "core_calibrated", "INTEGER")
# HR-only PSI term (aeris.indices.psi_hr_only): stays meaningful while the
# core-temperature model above is uncalibrated.
_add_column_if_missing("indices", "psi_hr_only", "REAL")
# Which strain figure actually triggered each alert: 'psi', 'psi_hr_only', or
# (Stage 7) 'autonomous_m33' for a band the M33 graded on its own while
# unreachable - see aeris.indices.STRAIN_SOURCE_* for the documented set. No
# ALTER TABLE is needed to add that third value: strain_source/active_source
# are plain TEXT with no CHECK constraint, so any string is already legal -
# this comment (and the STRAIN_SOURCE_* constants) is the actual documentation
# the guarded-schema helper would otherwise have needed to update.
_add_column_if_missing("alerts", "strain_source", "TEXT")
# What the alert path actually graded on this tick. `band` above always follows
# PSI, which disagrees with the alert whenever the core-temperature model is
# uncalibrated - so active_* is the single source of truth for anything
# downstream (dashboard, summary). For autonomous_m33 rows, active_strain is
# NULL (the M33 grades HR against raw bpm thresholds, not a 0-10 strain
# figure) - active_source and band are still populated.
_add_column_if_missing("indices", "active_strain", "REAL")
_add_column_if_missing("indices", "active_source", "TEXT")

db.commit()


def store_reading(ts, vitals):
    db.execute(
        "INSERT OR REPLACE INTO readings "
        "(ts, hr, hr_valid, spo2, spo2_valid, skin_c, amb_c, rh, motion_g, motion_state) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (
            ts,
            vitals.get("hr"),
            1 if vitals.get("hr_valid") else 0,
            vitals.get("spo2"),
            1 if vitals.get("spo2_valid") else 0,
            vitals.get("skin_c"),
            vitals.get("amb_c"),
            vitals.get("rh"),
            vitals.get("motion_g"),
            vitals.get("motion_state"),
        ),
    )
    db.commit()


def purge_old(ts):
    db.execute("DELETE FROM readings WHERE ts < ?", (ts - RETENTION_SECONDS,))
    db.commit()


def format_line(ts, vitals):
    hr = "{:.1f}".format(vitals["hr"]) if vitals.get("hr_valid") else "--"
    spo2 = "{:.1f}".format(vitals["spo2"]) if vitals.get("spo2_valid") else "--"
    skin = "{:.2f}".format(vitals["skin_c"]) if vitals.get("skin_c") is not None else "--"
    amb = "{:.2f}".format(vitals["amb_c"]) if vitals.get("amb_c") is not None else "--"
    rh = "{:.1f}".format(vitals["rh"]) if vitals.get("rh") is not None else "--"
    motion_g = "{:.3f}".format(vitals["motion_g"]) if vitals.get("motion_g") is not None else "--"
    motion_state = vitals.get("motion_state") or "--"
    return (
        "[{}] HR: {} bpm | SpO2: {} % | Skin: {} C | Amb: {} C RH {} % | Motion: {} g {}"
    ).format(ts, hr, spo2, skin, amb, rh, motion_g, motion_state)


# ---------------------------------------------------------------------------
# Stage 3/4: indices (ECTemp / PSI / personal baseline) + graded alerts
# ---------------------------------------------------------------------------
ectemp = ECTemp()
baseline = PersonalBaseline()

# The session's PSI reference point (Tc_0, HR_0): the core-temp/HR pair from
# the first time the wearer is observed STILL this session. Fixed for the
# rest of the session, distinct from the continuously-updating PersonalBaseline.
session_tc0 = None
session_hr0 = None

SEVERITY = {"NONE": 0, "LOW": 1, "MODERATE": 2, "HIGH": 3, "VERY_HIGH": 4}
LEVELS_BY_SEVERITY = ["NONE", "LOW", "MODERATE", "HIGH", "VERY_HIGH"]

MODERATE_STREAK_THRESHOLD_S = 60
HIGH_STREAK_THRESHOLD_S = 180
HR_DEVIATION_SD_THRESHOLD = 3.0
ALERT_SUPPRESS_SECONDS = 10 * 60

# Rule 3 (resting HR far from personal baseline) isn't derived from a PSI
# band, so it has none of its own. MODERATE is a deliberate placeholder
# middle-ground pending product guidance on how urgently to treat it.
HR_DEVIATION_ALERT_SEVERITY = SEVERITY["MODERATE"]

mod_streak_seconds = 0
high_streak_seconds = 0
last_alert_severity = -1
last_alert_ts = None
pending_alert_ts = None

# Printed to stdout at startup so the strain figure the alert path grades on is
# on the record (and in any demo recording), not just implied by the code.
ALERT_STRAIN_SOURCE = select_alert_strain(None, None).source
print(
    "[aeris] Alert path grading on {} (indices.CALIBRATED={}){}".format(
        ALERT_STRAIN_SOURCE,
        CALIBRATED,
        "" if CALIBRATED else " - ECTemp coefficients are placeholders, so PSI is excluded from alert grading",
    )
)


def store_indices(ts, core_c, core_var, core_calibrated, psi_value, psi_hr_only_value, band, baseline_hr, hr_dev_sd, alert_strain):
    # Taken straight off the same AlertStrain that evaluate_alerts() grades on,
    # so the recorded active_* can't drift from what actually fired.
    active_strain = alert_strain.value
    active_source = alert_strain.source if alert_strain.band is not None else None

    db.execute(
        "INSERT OR REPLACE INTO indices "
        "(ts, core_c, core_var, core_calibrated, psi, psi_hr_only, band, baseline_hr, hr_dev_sd, "
        "active_strain, active_source) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (ts, core_c, core_var, 1 if core_calibrated else 0, psi_value, psi_hr_only_value, band, baseline_hr, hr_dev_sd,
         active_strain, active_source),
    )
    db.commit()


def fire_alert(ts, band_name, strain_source):
    global pending_alert_ts

    try:
        Bridge.call("raise_alert", band_name)
    except Exception as e:
        logger.error("raise_alert bridge call failed: %s", e)

    db.execute(
        "INSERT OR REPLACE INTO alerts (ts, band, acked_ts, false_alarm, strain_source) "
        "VALUES (?, ?, NULL, NULL, ?)",
        (ts, band_name, strain_source),
    )
    db.commit()
    pending_alert_ts = ts
    logger.info("ALERT fired: band=%s source=%s ts=%s", band_name, strain_source, ts)


def evaluate_alerts(ts, band, strain_source, deviation, motion_state):
    global mod_streak_seconds, high_streak_seconds
    global last_alert_severity, last_alert_ts

    band_severity = SEVERITY.get(band, SEVERITY["NONE"])

    mod_streak_seconds = mod_streak_seconds + 1 if band_severity >= SEVERITY["MODERATE"] else 0
    high_streak_seconds = high_streak_seconds + 1 if band_severity >= SEVERITY["HIGH"] else 0

    candidate_severity = None

    if mod_streak_seconds >= MODERATE_STREAK_THRESHOLD_S:
        candidate_severity = band_severity

    if high_streak_seconds >= HIGH_STREAK_THRESHOLD_S:
        # Sustained HIGH+ for the full 180s escalates the alert one level
        # above the currently observed band (capped at VERY_HIGH).
        escalated = min(band_severity + 1, SEVERITY["VERY_HIGH"])
        candidate_severity = escalated if candidate_severity is None else max(candidate_severity, escalated)

    if motion_state == "STILL" and deviation is not None and abs(deviation) > HR_DEVIATION_SD_THRESHOLD:
        candidate_severity = (
            HR_DEVIATION_ALERT_SEVERITY
            if candidate_severity is None
            else max(candidate_severity, HR_DEVIATION_ALERT_SEVERITY)
        )

    if candidate_severity is None:
        return

    if (
        last_alert_ts is not None
        and candidate_severity <= last_alert_severity
        and (ts - last_alert_ts) < ALERT_SUPPRESS_SECONDS
    ):
        return  # suppressed: same-or-lower band within 10 minutes of the last alert

    fire_alert(ts, LEVELS_BY_SEVERITY[candidate_severity], strain_source)
    last_alert_severity = candidate_severity
    last_alert_ts = ts


def process_indices(ts, vitals):
    global session_tc0, session_hr0

    hr = vitals["hr"]
    skin_c = vitals.get("skin_c")
    motion_state = vitals.get("motion_state")

    core_c, core_var, core_calibrated = ectemp.update(hr)

    # Latched independently: the HR-only strain term needs HR_0 alone, so it
    # must not wait on a core-temperature reference it doesn't use.
    if motion_state == "STILL":
        if session_hr0 is None:
            session_hr0 = hr
        if session_tc0 is None:
            session_tc0 = core_c

    if skin_c is not None:
        baseline.update(hr, skin_c, motion_state)

    psi_value, psi_hr_only_value = compute_strain_indices(core_c, session_tc0, hr, session_hr0)
    band = psi_band(psi_value) if psi_value is not None else None

    # The band stored above is always the PSI-derived one; the alert path may
    # grade on a different figure while the core-temperature model is
    # uncalibrated (see the startup banner).
    alert_strain = select_alert_strain(psi_value, psi_hr_only_value)

    deviation = baseline.deviation_sd(hr) if baseline.is_ready else None

    store_indices(
        ts, core_c, core_var, core_calibrated,
        psi_value, psi_hr_only_value, band,
        baseline.hr_mean, deviation, alert_strain,
    )

    evaluate_alerts(
        ts,
        alert_strain.band if alert_strain.band is not None else "NONE",
        alert_strain.source,
        deviation,
        motion_state,
    )

    return alert_strain


def ack_alert():
    global pending_alert_ts

    if pending_alert_ts is None:
        logger.info("ack_alert received but no pending alert to acknowledge")
        return

    acked_ts = int(time.time())
    db.execute("UPDATE alerts SET acked_ts = ? WHERE ts = ?", (acked_ts, pending_alert_ts))
    db.commit()
    logger.info("Alert at %s acknowledged at %s", pending_alert_ts, acked_ts)
    pending_alert_ts = None


Bridge.provide("ack_alert", ack_alert)


# ---------------------------------------------------------------------------
# Stage 7: M33 autonomous fail-safe - HR-threshold cache + reconnect drain
# ---------------------------------------------------------------------------
# HR reference last sent via set_thresholds(), so it's only re-sent once it
# actually drifts rather than on every tick for a value that barely moves.
_last_sent_hr0 = None
HR0_RESEND_DELTA_BPM = 2.0  # placeholder - re-send once the reference moves this much

# Whether the previous tick's Bridge.call("read_vitals") failed. Used to
# detect the "read_vitals() succeeds again" edge, i.e. Python just reconnected.
_bridge_was_down = False


def _hr_thresholds_for(hr0):
    """Mirrors aeris.indices.psi_hr_only()'s linear HR<->PSI relationship
    (10*(hr-hr0)/(180-hr0)), inverted to the HR values at which that formula
    crosses psi_band()'s NONE/LOW/MODERATE/HIGH cutoffs (2/4/6/8). This is a
    separate, parallel computation for the M33's fail-safe cache only - it
    does not call into or modify indices.py's own psi_hr_only()/psi_band().

    Known limitation, stated once here rather than repeated at each call
    site: if indices.CALIBRATED ever becomes True, real-time grading also
    folds in core temperature (full psi), which the M33 cannot reproduce on
    its own since it has no core-temperature model - the autonomous fallback
    always grades on HR alone, regardless of CALIBRATED. That is an
    intentional fail-safe simplification, not an attempt to fully mirror PSI.
    """
    span = 180.0 - hr0
    return tuple(hr0 + boundary * span / 10.0 for boundary in (2.0, 4.0, 6.0, 8.0))


def maybe_push_thresholds():
    """Sends set_thresholds() once a resting HR reference exists, and again
    whenever it drifts by more than HR0_RESEND_DELTA_BPM. Prefers the
    continuously-refined PersonalBaseline mean once ready; falls back to the
    one-shot session_hr0 anchor before that, so the M33 has some cache as
    early as possible rather than waiting on 300 stillness samples for any
    cache at all. Call once per tick after process_indices()."""
    global _last_sent_hr0

    hr0 = baseline.hr_mean if baseline.is_ready else session_hr0
    if hr0 is None:
        return
    if _last_sent_hr0 is not None and abs(hr0 - _last_sent_hr0) < HR0_RESEND_DELTA_BPM:
        return

    hr_low, hr_moderate, hr_high, hr_very_high = _hr_thresholds_for(hr0)
    try:
        Bridge.call(
            "set_thresholds",
            round(hr_low), round(hr_moderate), round(hr_high), round(hr_very_high),
        )
        logger.info(
            "set_thresholds sent: hr0=%.1f -> LOW>=%d MODERATE>=%d HIGH>=%d VERY_HIGH>=%d",
            hr0, round(hr_low), round(hr_moderate), round(hr_high), round(hr_very_high),
        )
        _last_sent_hr0 = hr0
    except Exception as e:
        logger.error("set_thresholds bridge call failed: %s", e)


def _convert_autonomous_events(events, now_ms):
    """Converts the M33's boot-relative t_ms into wall-clock unix seconds.

    Anchor: at the instant drain_autonomous_log() returns, the M33 reports
    its own current millis() as now_ms, and this line runs at essentially the
    same instant on the Python side (Bridge.call() is synchronous). offset =
    python_now - now_ms/1000 is that one anchor point; every event converts
    via wall_ts = offset + t_ms/1000.

    Known error margin - reasoned, not measured on this hardware:
      - The Bridge.call() round-trip itself between the M33 sampling now_ms
        and this line reading time.time(): unmeasured for this board, likely
        low milliseconds to some tens of ms given it's synchronous.
      - millis() has 1ms resolution; the float math above adds nothing on top.
      - Clock drift between the M33's oscillator and the Linux clock,
        accumulated over the outage: unmeasured for this specific board.
        Typical crystal oscillators run within tens of ppm, which would stay
        well under a second even over a multi-hour outage, but that is an
        estimate from general hardware norms, not a verified spec here.
    Net expectation: sub-second accuracy for outages of ordinary length
    (minutes). Do not treat these timestamps as more precise than that.
    """
    python_now = time.time()
    offset = python_now - (now_ms / 1000.0)
    return [(int(offset + ev["t_ms"] / 1000.0), ev.get("hr"), ev["band"]) for ev in events]


def drain_autonomous_log(reason):
    """Calls the M33's drain_autonomous_log(), converts and stores whatever
    it recorded on its own. Called once at startup and once on every detected
    reconnect (see loop())."""
    try:
        raw = Bridge.call("drain_autonomous_log")
        payload = json.loads(raw)
    except Exception as e:
        logger.error("drain_autonomous_log failed (%s): %s", reason, e)
        return

    events = payload.get("events") or []
    overflow_count = payload.get("overflow_count", 0)
    now_ms = payload.get("now_ms")

    if overflow_count:
        logger.warning(
            "Autonomous log overflowed: %d event(s) were overwritten on the M33 "
            "before this drain and are unrecoverable.", overflow_count,
        )

    if not events:
        return

    if now_ms is None:
        logger.error(
            "drain_autonomous_log response missing now_ms - cannot anchor "
            "timestamps, discarding %d event(s) rather than guess.", len(events),
        )
        return

    logger.warning(
        "Draining %d autonomous_m33 event(s) recorded while Python was unreachable (%s).",
        len(events), reason,
    )

    for wall_ts, hr, band in _convert_autonomous_events(events, now_ms):
        # indices: one row per drained event, for the continuous timeline. No
        # psi/psi_hr_only/core_c exists for these - the M33 graded raw HR
        # against raw bpm thresholds, not a 0-10 strain figure - so those
        # columns stay NULL rather than fabricating a number.
        db.execute(
            "INSERT OR REPLACE INTO indices "
            "(ts, core_c, core_var, core_calibrated, psi, psi_hr_only, band, "
            "baseline_hr, hr_dev_sd, active_strain, active_source) "
            "VALUES (?, NULL, NULL, 0, NULL, NULL, ?, NULL, NULL, NULL, ?)",
            (wall_ts, band, STRAIN_SOURCE_AUTONOMOUS_M33),
        )

        # alerts: only for events that were actually elevated - mirrors
        # fire_alert()'s own semantics, where a row means "an alert condition
        # was raised", not "a tick happened".
        if band != "NONE":
            db.execute(
                "INSERT OR REPLACE INTO alerts (ts, band, acked_ts, false_alarm, strain_source) "
                "VALUES (?, ?, NULL, NULL, ?)",
                (wall_ts, band, STRAIN_SOURCE_AUTONOMOUS_M33),
            )
            logger.info("Recovered autonomous alert: band=%s ts=%s hr=%s", band, wall_ts, hr)

    db.commit()


# Recover anything the M33 logged on its own before this run even started -
# e.g. Python crashed and was restarted, or this is a fresh deploy onto a
# board that had already been running standalone for a while.
drain_autonomous_log("startup")


# ---------------------------------------------------------------------------
# Dashboard (WebUI Brick)
# ---------------------------------------------------------------------------
# Last payload pushed over the socket, kept so a browser connecting mid-session
# (or hitting GET /api/live) paints immediately instead of waiting a second.
_latest_live = {"status": "starting_up"}


def baseline_snapshot():
    return {
        "hr_mean": baseline.hr_mean,
        "hr_sd": baseline.hr_sd,
        "still_samples": baseline.still_samples,
        "samples_until_ready": baseline.samples_until_ready,
        "min_still_samples": PersonalBaseline.MIN_STILL_SAMPLES,
        "is_ready": baseline.is_ready,
    }


def publish_live(ts, vitals, alert_strain):
    """Build and push the NOW + baseline payload. Every numeric field travels
    with the validity flag it was measured under, so the UI can grey out
    anything not currently live rather than showing a stale number."""
    global _latest_live

    if vitals is None:
        # Bridge read failed this tick - report that explicitly instead of
        # letting the browser keep the previous second's numbers on screen.
        _latest_live = {"status": "bridge_error", "ts": ts}
    else:
        _latest_live = {
            "status": "ok",
            "ts": ts,
            "hr": vitals.get("hr"),
            "hr_valid": bool(vitals.get("hr_valid")),
            "spo2": vitals.get("spo2"),
            "spo2_valid": bool(vitals.get("spo2_valid")),
            "skin_c": vitals.get("skin_c"),
            "amb_c": vitals.get("amb_c"),
            "rh": vitals.get("rh"),
            "motion_g": vitals.get("motion_g"),
            "motion_state": vitals.get("motion_state"),
            # The strain figure the alert path actually graded on this tick.
            "strain": alert_strain.value if alert_strain else None,
            "band": alert_strain.band if alert_strain else None,
            "strain_source": alert_strain.source if alert_strain else None,
            "calibrated": CALIBRATED,
            "baseline": baseline_snapshot(),
        }

    push_live(ui, _latest_live)


try:
    # host/port kwargs are not exercised by any local WebUI example, so fall
    # back to the Brick's own defaults rather than failing to start if this
    # build doesn't accept them.
    ui = WebUI(host=config.DASHBOARD_HOST, port=config.WEB_PORT)
    logger.info("WebUI bound to %s:%s", config.DASHBOARD_HOST, config.WEB_PORT)
except TypeError:
    ui = WebUI()
    logger.warning(
        "WebUI() rejected host/port kwargs - falling back to Brick defaults "
        "(port %s). config.DASHBOARD_HOST=%s was NOT applied.",
        config.WEB_PORT, config.DASHBOARD_HOST,
    )

register_web_ui(
    ui,
    DB_PATH,
    config.STRAIN_WINDOW_HOURS,
    config.STRAIN_BUCKET_SECONDS,
    lambda: _latest_live,
)


def loop():
    global _bridge_was_down

    time.sleep(1)
    ts = int(time.time())

    try:
        raw = Bridge.call("read_vitals")
        vitals = json.loads(raw)
    except Exception as e:
        logger.error("read_vitals failed, skipping this second: %s", e)
        _bridge_was_down = True
        publish_live(ts, None, None)
        return

    if _bridge_was_down:
        # read_vitals() just succeeded after a prior failure: Python was
        # unreachable for at least one tick, so recover whatever the M33
        # graded on its own while it couldn't reach us.
        drain_autonomous_log("reconnect")
        _bridge_was_down = False

    store_reading(ts, vitals)
    purge_old(ts)
    print(format_line(ts, vitals))

    alert_strain = None
    if vitals.get("hr_valid") and vitals.get("hr") is not None:
        alert_strain = process_indices(ts, vitals)

    maybe_push_thresholds()

    publish_live(ts, vitals, alert_strain)


App.run(user_loop=loop)
