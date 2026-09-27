"""Dashboard / web-server configuration for aeris.

Kept out of main.py so the bind address and port can be changed without
touching application source.
"""
import os

# ---- Web server ----
# The WebUI-HTML Brick manages its own server; it defaults to port 7000.
WEB_PORT = 7000

# Bind address for the dashboard. Defaults to every interface, so the board is
# reachable on whatever LAN address it currently holds without any private
# address being committed to the repo.
#
# Override with the AERIS_DASHBOARD_HOST environment variable if the server
# ever needs pinning to one specific interface.
DASHBOARD_HOST = os.environ.get("AERIS_DASHBOARD_HOST", "0.0.0.0")

# ---- Dashboard data windows ----
# "STRAIN TODAY" chart window.
STRAIN_WINDOW_HOURS = 8

# The chart is downsampled server-side into buckets of this many seconds.
# At 1 reading/second an 8h window is ~28,800 rows; 120s buckets bring that
# to ~240 points, which is all a line chart of this size can resolve anyway.
STRAIN_BUCKET_SECONDS = 120

# How often the browser re-pulls the chart + alert table over REST. The NOW
# panel does not use this - it is pushed over the WebSocket every second.
DASHBOARD_REST_POLL_SECONDS = 30
