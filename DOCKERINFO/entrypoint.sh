#!/bin/bash
# Start Xvfb (a virtual display server for GUI applications)
Xvfb :1 -screen 0 1024x768x16 &

# Start the VNC server
x11vnc -display :1 -nopw -forever &

# Start web-based VNC access with NoVNC
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &

# Start the XFCE desktop environment
startxfce4 &

# Keep the container running
exec bash
