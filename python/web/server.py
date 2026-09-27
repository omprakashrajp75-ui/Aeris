"""Dashboard backend, built on the WebUI-HTML Brick API confirmed in the
local examples and in DeskSense:

    from arduino.app_bricks.web_ui import WebUI
    ui = WebUI()
    ui.expose_api("GET", "/api/path", handler)   # REST, no-arg handler
    ui.on_message("channel", fn)                 # fn(client, data)
    ui.on_connect(lambda sid: ...)
    ui.send_message("channel", payload)          # push to all clients

Split of cadences:
  - The NOW panel and the baseline panel are pushed over the WebSocket once
    per second from main.py's loop (push_live), since both are already in
    memory there - no DB round trip, no polling.
  - The 8h strain chart and the alerts table are pulled over REST on a much
    slower timer. Re-sending an 8-hour series every second would be absurd
    (~28,800 rows/s), so those are downsampled and polled instead.

Every DB read here opens its own short-lived connection: the Brick serves
these handlers on its own thread, and a sqlite3 connection may only be used
from the thread that created it. main.py's connection stays on the app loop
thread; these never touch it.
"""
import logging
import sqlite3
import time

logger = logging.getLogger("aeris.web")

# PSI band thresholds, drawn as horizontal reference lines on the chart.
# Upper bound of each band, matching aeris.indices.psi_band().
BAND_THRESHOLDS = [
    {"label": "NONE", "upper": 2},
    {"label": "LOW", "upper": 4},
    {"label": "MODERATE", "upper": 6},
    {"label": "HIGH", "upper": 8},
    {"label": "VERY_HIGH", "upper": 10},
]


def _connect(db_path):
    conn = sqlite3.connect(db_path, timeout=10)
    conn.row_factory = sqlite3.Row
    return conn


def _start_of_today():
    """Unix seconds at local midnight today."""
    now = time.localtime()
    return int(time.mktime((now.tm_year, now.tm_mon, now.tm_mday, 0, 0, 0, 0, 0, -1)))


def get_strain_series(db_path, window_hours, bucket_seconds):
    """Downsampled strain over the last `window_hours`.

    Plots active_strain, not psi: active_strain is what the alert path
    actually graded on (see main.store_indices), so charting psi here would
    reintroduce exactly the disagreement active_* exists to prevent. Both
    the value and its source are returned so the UI can say which it is.
    """
    since = int(time.time()) - window_hours * 3600
    conn = _connect(db_path)
    try:
        cur = conn.execute(
            "SELECT (ts / ?) * ? AS bucket, "
            "       AVG(active_strain) AS strain, "
            "       MAX(active_source) AS source, "
            "       COUNT(active_strain) AS n "
            "FROM indices "
            "WHERE ts >= ? AND active_strain IS NOT NULL "
            "GROUP BY bucket ORDER BY bucket ASC",
            (bucket_seconds, bucket_seconds, since),
        )
        points = [
            {"ts": r["bucket"], "strain": r["strain"], "source": r["source"]}
            for r in cur.fetchall()
        ]
    finally:
        conn.close()

    return {
        "points": points,
        "since": since,
        "window_hours": window_hours,
        "thresholds": BAND_THRESHOLDS,
        "source": points[-1]["source"] if points else None,
    }


def get_alerts_today(db_path):
    """Today's alerts, newest first, with time-to-acknowledge in seconds."""
    conn = _connect(db_path)
    try:
        cur = conn.execute(
            "SELECT ts, band, acked_ts, false_alarm, strain_source "
            "FROM alerts WHERE ts >= ? ORDER BY ts DESC",
            (_start_of_today(),),
        )
        rows = cur.fetchall()
    finally:
        conn.close()

    alerts = []
    for r in rows:
        acked_ts = r["acked_ts"]
        alerts.append({
            "ts": r["ts"],
            "band": r["band"],
            "acked_ts": acked_ts,
            "ack_seconds": (acked_ts - r["ts"]) if acked_ts is not None else None,
            "false_alarm": bool(r["false_alarm"]),
            "strain_source": r["strain_source"],
        })
    return {"alerts": alerts}


def mark_false_alarm(db_path, ts):
    """Set false_alarm=1 on one alert row. Returns True if a row was hit."""
    conn = _connect(db_path)
    try:
        cur = conn.execute("UPDATE alerts SET false_alarm = 1 WHERE ts = ?", (int(ts),))
        conn.commit()
        return cur.rowcount > 0
    finally:
        conn.close()


def register_web_ui(ui, db_path, window_hours, bucket_seconds, get_live_payload):
    """Wire the WebUI Brick instance to our data sources. Call once at startup.

    `get_live_payload` is a zero-arg callable returning the current NOW +
    baseline dict, so a browser connecting mid-session paints immediately
    instead of waiting for the next 1-second push.
    """

    ui.expose_api("GET", "/api/strain", lambda: get_strain_series(db_path, window_hours, bucket_seconds))
    ui.expose_api("GET", "/api/alerts", lambda: get_alerts_today(db_path))
    # Fallback for the very first paint, before any socket message arrives.
    ui.expose_api("GET", "/api/live", get_live_payload)

    def on_mark_false_alarm(client, data):
        ts = (data or {}).get("ts")
        if ts is None:
            logger.warning("mark_false_alarm called without a ts")
            return
        try:
            if mark_false_alarm(db_path, ts):
                logger.info("Alert %s marked as a false alarm", ts)
            else:
                logger.warning("mark_false_alarm: no alert row at ts=%s", ts)
            # Push the refreshed table to every client, so a second browser
            # doesn't sit on a stale row until its next REST poll.
            ui.send_message("alerts", get_alerts_today(db_path))
        except Exception as exc:  # noqa: BLE001 - a UI action must not kill the app
            logger.error("mark_false_alarm failed: %s", exc)

    ui.on_message("mark_false_alarm", on_mark_false_alarm)

    ui.on_connect(lambda sid: _push_initial(ui, sid, db_path, get_live_payload))


def _push_initial(ui, sid, db_path, get_live_payload):
    try:
        ui.send_message("live", get_live_payload(), room=sid)
        ui.send_message("alerts", get_alerts_today(db_path), room=sid)
    except Exception as exc:  # noqa: BLE001
        logger.warning("Initial push to client %s failed: %s", sid, exc)


def push_live(ui, payload):
    """Called once per second from the app loop."""
    try:
        ui.send_message("live", payload)
    except Exception as exc:  # noqa: BLE001 - a dashboard hiccup must not kill sampling
        logger.warning("Failed to push live sample over WebSocket: %s", exc)
