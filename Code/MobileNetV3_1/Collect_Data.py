import cv2
import numpy as np
import time
import os

def extract_and_save_roi(frame, out_dir, margin=5):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 二値化（黒文字前提）
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 輪郭抽出
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        print("ブロブなし")
        return None, None

    # 最大ブロブ
    cnt = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(cnt)

    # マージン付与
    x2 = max(x - margin, 0)
    y2 = max(y - margin, 0)
    x3 = min(x + w + margin, frame.shape[1])
    y3 = min(y + h + margin, frame.shape[0])

    roi = gray[y2:y3, x2:x3]

    # 96x96 にリサイズ
    roi_resized = cv2.resize(roi, (96, 96), interpolation=cv2.INTER_AREA)

    # 保存
    os.makedirs(out_dir, exist_ok=True)
    filename = f"picture_u{time.time():.6f}.png"
    save_path = os.path.join(out_dir, filename)
    cv2.imwrite(save_path, roi_resized)

    print("保存:", save_path)

    # 描画用に座標返す
    return save_path, (x, y, w, h)


def run_camera_and_save(out_dir="output"):
    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display_frame = frame.copy()

        # 最大ブロブを毎フレーム描画
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("rawdata", display_frame)

        key = cv2.waitKey(1) & 0xFF

        # sキーでROI保存
        if key == ord('s'):
            extract_and_save_roi(frame, out_dir)

        # qキーで終了
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# 実行
run_camera_and_save("detaset_V3_input\\train\\unknown")
