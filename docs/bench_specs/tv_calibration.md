# Task: make camera calibration painless -- "hold the camera in front of the TV"

The builder keeps skipping checkerboard calibration (skr-06: >= 15 views, RMS <= 1.0 px,
`scripts/calibrate_camera.py`). He has a TV connected to his Windows laptop. Build a guided
tool so the whole job is: start it, wave the camera at the TV for ~2 minutes, done.

PHYSICS YOU MUST RESPECT: a flat screen is ONE plane. Warping the pattern on screen does not
create new views; intrinsics need the CAMERA to see the plane from different angles. So the
camera (Pi 5 + OV9281 on a cable, handheld or on the desk tripod) is moved/tilted by hand;
the screen shows a fixed, bright, full-screen pattern. Use a ChArUco board if the Pi's OpenCV
(4.10) has cv2.aruco (partial views near the image edges still count -- important for this
118-degree HFOV lens); fall back to a checkerboard.
Pieces:
1. `scripts/bench/calib_pattern.html` -- one self-contained page the builder opens full
   screen on the TV (F11): draws the board as crisp SVG/canvas at native resolution, prints
   the square size in PIXELS, and has an input for the TV's visible width in mm so it shows
   the square size in mm (the tool needs it; ask for a ruler measurement as the fallback).
2. `scripts/bench/calib_live.py` on the Pi (ssh admin@192.168.0.42; picamera2, 1280x800,
   short fixed exposure to avoid blur/flicker -- test 1-4 ms against the TV's refresh): grabs
   frames, detects the board, auto-KEEPS a frame only when it is sharp and its pose differs
   enough from those already kept, and tracks COVERAGE: a 4x3 grid of image regions x tilt
   bins. It serves a tiny web page (http://192.168.0.42:8000) showing the live view, the
   coverage map and one plain instruction at a time ("tilt left", "move closer", "put the
   board in the top-right corner"), so he can watch it on his phone. Stops at >= 25 good views
   with full coverage, then solves BOTH a pinhole+distortion model and a fisheye model
   (cv2.fisheye), reports RMS for each, and writes the intrinsics file in the format
   `scripts/calibrate_camera.py` / `flight/camera.py` already use (read them first).
3. A dry run you do yourself: you cannot move the camera, but you CAN verify the capture,
   detection on a synthetic rendered board image, the coverage logic, the solver on synthetic
   views with known intrinsics (recover fx within 1%), and that the web page serves.
   Tests in `tests/test_calib_live.py` (synthetic, no camera needed).
Report: exact 5-line instructions for the builder, what you verified vs could not, any
package you had to install (say before installing; pip --user only). Do not run git.
