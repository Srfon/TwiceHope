from lidar_drive import LidarScanner
import time

def fmt(x):
    return "---" if x == float("inf") else str(round(x))

s = LidarScanner()
s.start()
try:
    for _ in range(40):                      # ��12�b�Ԃ񌩂�
        with s._lock:
            n = len(s._points)               # �ێ����Ă���_�̐�
        print("data=", s.has_data(),
              " n=", n,
              " front=", fmt(s._nearest(-25, 25)),
              " left=", fmt(s._nearest(-70, -20)),
              " right=", fmt(s._nearest(20, 70)))
        time.sleep(0.3)
finally:
    s.stop()