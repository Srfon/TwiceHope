import cv2
import time
import subprocess

print("Auto controls disabled for all cameras.")

cams = [
    cv2.VideoCapture(0)
]

for cam in cams:
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cam_shutter_count = 0
base_time = time.time()
current_time = 0.0

while True:
    current_time = time.time()
    ret1, frame1 = cams[0].read()
    cam_shutter_count += 1

    diff_time = current_time - base_time
    if diff_time >= 1.0:
        fps = cam_shutter_count
        cam_shutter_count = 0
        base_time = current_time

    if ret1:
        cv2.imshow("Realtek Cam 1", frame1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

for cam in cams:
    cam.release()
cv2.destroyAllWindows()
