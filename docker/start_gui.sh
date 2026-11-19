#!/usr/bin/env bash
# Start a virtual X display plus VNC/noVNC so Gazebo and RViz can be used
# from a browser (http://localhost:6080/vnc.html).  No XQuartz required.
set -e

export DISPLAY=:99

if ! pgrep -f "Xvfb :99" >/dev/null 2>&1; then
  # A container restart leaves the old lock/socket behind.
  rm -f /tmp/.X99-lock /tmp/.X11-unix/X99
  mkdir -p /tmp/.X11-unix
  chmod 1777 /tmp/.X11-unix
  Xvfb :99 -screen 0 1600x1000x24 -ac +extension GLX +render -noreset \
    >/tmp/xvfb.log 2>&1 &
  for _ in $(seq 1 20); do
    xdpyinfo -display :99 >/dev/null 2>&1 && break
    sleep 0.5
  done
fi

if ! pgrep -x fluxbox >/dev/null 2>&1; then
  fluxbox >/tmp/fluxbox.log 2>&1 &
fi

if ! pgrep -x x11vnc >/dev/null 2>&1; then
  x11vnc -display :99 -forever -shared -nopw -rfbport 5900 \
    -o /tmp/x11vnc.log >/dev/null 2>&1 &
fi

if ! pgrep -f "websockify.*6080" >/dev/null 2>&1; then
  websockify --web=/usr/share/novnc 6080 localhost:5900 \
    >/tmp/websockify.log 2>&1 &
fi

# Frame the classroom as soon as the Gazebo GUI is reachable.
(
  for _ in $(seq 1 120); do
    if ign service -l 2>/dev/null | grep -qx '/gui/move_to'; then
      sleep 2
      ign service -s /gui/move_to \
        --reqtype ignition.msgs.StringMsg \
        --reptype ignition.msgs.Boolean \
        --timeout 5000 \
        --req 'data: "classroom"' >/dev/null 2>&1
      break
    fi
    sleep 1
  done
) &

sleep 1
echo "GUI ready -> http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
