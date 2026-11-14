# RoboCup USB camera integration (Pi 5)

Source: \`camera.py\` + \`robot_mapping5.py\` in this directory.

## What is implemented

- One V4L2 owner for \`/dev/video0\`, shared with browser and detection.
- OpenCV MJPEG USB capture, latest-frame-only buffering.
- Red/yellow/green HSV detection (~10Hz, 5-of-7 confirmation).
- Experimental H/S/U OCR using optional Tesseract (~1.3Hz, 3-of-5 confirmations).
- Existing tile map receives color observations, plus provisional \`camera_letters\` observations.
- Web dashboard at \`http://trin1.local:8000/\`: LiDAR, tile map, camera.
- Preferred WebRTC H.264 browser playback (needs \`aiortc\` and \`av\`), with MJPEG fallback.
- Existing emergency stop, motor and navigation code is not intentionally altered.

**Safety:** H/S/U OCR is experimental. It can misread letters, and does not trigger
kit release or alter route decisions. Map annotations are provisional until verified
on the actual course. Neither WebRTC nor 60 fps is guaranteed without Pi testing.

## Install

Stop standalone \`ustreamer\`, \`ffmpeg\` and any Python process opening the same
camera. Do not run multiple copies of \`robot_mapping5.py\`.

On the Pi, using the existing project virtualenv:

\`\`\`bash
cd ~/RobCup_V3
source .venv/bin/activate
python -m pip install aiortc av pytesseract
sudo apt-get install -y tesseract-ocr
\`\`\`

The existing project's \`fastapi\`, \`uvicorn\`, \`opencv-python\`,
\`numpy\`, \`pyserial\`, and motor/driver dependencies are still required.
If aiortc cannot be installed, MJPEG works without it. If pytesseract or
Tesseract is absent, OCR stays disabled while colors and video continue.

Copy *both* files into \`~/RobCup_V3/sts3215/\` from this same branch;
back up the current \`robot_mapping5.py\` first. Validate:

\`\`\`bash
python -m py_compile sts3215/camera.py sts3215/robot_mapping5.py
python sts3215/robot_mapping5.py --view-only --color-camera /dev/video0 --camera-fps 30 --webrtc-fps 30
\`\`\`

Use \`--no-ocr\` to disable text recognition during troubleshooting.
The \`--view-only\` flag leaves motors disabled but requires the project's
LiDAR and usual display dependencies.

- Dashboard: http://trin1.local:8000/
- Camera MJPEG fallback: http://trin1.local:8000/color/stream
- Camera state: http://trin1.local:8000/color/state
- Latest JPEG: http://trin1.local:8000/color.jpg

After checking camera and LiDAR on a stationary robot, perform a separate,
controlled test of navigation; do not activate motors just to test the video.

## Performance

\`\`\`bash
# Prefer a reliable 30 fps before attempting 60 fps.
python sts3215/robot_mapping5.py --view-only --camera-fps 30 --webrtc-fps 30

# Optional target after observing CPU, browser fps and latency.
python sts3215/robot_mapping5.py --view-only --camera-fps 60 --webrtc-fps 60
\`\`\`

\`--camera-fps\` limits how often decoded frames are published; it does not
guarantee the USB driver actually operates at that requested frame interval.
On the tested camera the driver rounded 30 and 60 fps requests to ~120 fps.
A lower published rate avoids old frames accumulating but limits actual
viewing fps. Browser WebRTC can still be constrained by software H.264
encoding on the Raspberry Pi.

## Rollback

Stop the robot program, restore the backed up \`robot_mapping5.py\`, and
keep the pre-integration \`camera.py\` file out of the active project path.
Avoid merging the draft PR to \`main\` until the actual Pi checks pass.


## GPIO22 button integration

\`Code/Display_Process/display_manager5.py\` on this branch now launches
\`~/RobCup_V3/sts3215/robot_mapping5.py\` (not \`robot_mapping5_2.py\`).
The launch command uses the already-tested USB camera and browser settings:

- \`--web-port 8010\` (8000 currently belongs to another Python process)
- \`--color-camera /dev/video0\`
- \`--camera-fps 30 --webrtc-fps 30\`
- \`--lcd-mode map\`
- \`--no-ocr\` by default until OCR accuracy is verified

These defaults can be overridden in the environment of the display manager:
\`ROBOT_WEB_PORT\`, \`ROBOT_CAMERA_DEVICE\`, \`ROBOT_CAMERA_FPS\`,
\`ROBOT_WEBRTC_FPS\`, and \`ROBOT_CAMERA_OCR=1\` to enable OCR.

Existing GPIO behavior is preserved:
- GPIO22 (Button1): long-press 2 sec to launch when stopped; short press
  once after launch to start motion; double press within 0.5 sec to stop/exit.
- GPIO27 (Button3): emergency stop, reinitialize and wait.
- GPIO17 (Button4): reserved.

The display manager remains the **sole GPIO22 owner**, forwarding button
presses to the robot through the existing UNIX datagram socket. The robot
still prevents duplicate launch with its instance lock.

### Deployment

The operator should first safely stop any manually launched
\`--view-only\` instance of \`robot_mapping5.py\` using Ctrl+C.
Otherwise the button monitor detects an already running robot and does
not start a second one.

Back up the existing \`Display_Process/display_manager5.py\` and deploy
the branch version. Validate syntax with
\`python3 -m py_compile Display_Process/display_manager5.py\` before
restarting the actual display manager service (check the real unit name
with \`systemctl list-units --all --type=service\` first).

**Safety:** button-launched mode is **not** \`--view-only\`. After launch
the robot waits for a subsequent Button1 press before motion. Keep the
robot's wheels off the ground for the first button-control test. Avoid
sending Button1 press events merely to check the camera feed.

- Browser dashboard: \`http://trin1.local:8010/\`
- Camera stream: \`http://trin1.local:8010/color/stream\`
- Camera state: \`http://trin1.local:8010/color/state\`
