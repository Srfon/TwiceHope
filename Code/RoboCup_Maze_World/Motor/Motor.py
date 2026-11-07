#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sts3215 import STS3215
import time

class Motor:
    # ID1　右後ろ
    # ID2　左後ろ 
    # ID3　右前
    # ID4　左前
    ## ID2, ID4 は逆回転する
    def __init__(self, State):
        self.PORT = "/dev/ttyUSB0"
        self.BAUDRATE = 1_000_000
        self.SPEED_MIN = -1023
        self.SPEED_MAX = 1023
        self._state = State
        self._servo = STS3215(self.PORT, self.BAUDRATE)
        self._servo.open()

        motorIDs = [1,2,3,4]
        for motorID in motorIDs:
            print(self._servo.get_mode(motorID))
            if self._servo.get_mode(motorID) != 1:
                self._servo.set_mode(motorID, 1)

    def stop(self):
        self._servo.set_speed(1, 0)
        self._servo.set_speed(3, 0)
        self._servo.set_speed(2, 0)
        self._servo.set_speed(4, 0)

    def close(self):
        self._servo.close()

    def set_speed(self, id, speed):
        if speed < self.SPEED_MIN or speed > self.SPEED_MAX:
            raise ValueError("Speed must be between {} and {}".format(self.SPEED_MIN, self.SPEED_MAX))
        self._servo.set_speed(id, speed)

    def get_speed(self, id):
        return self._servo.get_speed(id)

    def set_position(self, id, position):
        self._servo.set_position(id, position)

    def get_position(self, id):
        return self._servo.get_position(id)

    def set_torque(self, id, torque):
        self._servo.set_torque(id, torque)

    def get_torque(self, id):
        return self._servo.get_torque(id)

    def get_status(self, id):
        return self._servo.status(id)

    def move_tank(self, left_speed, right_speed):
        self._servo.set_speed(1, left_speed)
        self._servo.set_speed(3, left_speed)

        self._servo.set_speed(2, right_speed)
        self._servo.set_speed(4, right_speed)

    def move_tank_for_degrees(self, left_degrees, right_degrees, left_speed, right_speed):
        self._servo.set_speed(1, left_speed)
        self._servo.set_speed(3, left_speed)
        self._servo.set_speed(2, right_speed*-1)
        self._servo.set_speed(4, right_speed*-1)

        base_fr_offset = self._servo.get_position(3)
        base_fl_offset = self._servo.get_position(4)
        base_br_offset = self._servo.get_position(1)
        base_bl_offset = self._servo.get_position(2)

        ok_fr = False
        ok_fl = False
        ok_br = False
        ok_bl = False

        # 角度→エンコーダ値へ変換
        def deg_to_pos(deg):
            return (deg % 360) * 4096 / 360

        target_left_pos = deg_to_pos(left_degrees)
        target_right_pos = deg_to_pos(right_degrees)

        def calc_rot(current, offset, reverse=False):
            # 基準点との差分
            diff = current - offset

            # オーバーフロー補正
            if diff > 2048:
                diff -= 4096
            elif diff < -2048:
                diff += 4096

            # 左モーターは逆回転なので符号反転
            return -diff if reverse else diff
        
        while True:
            base_fr_deg = self._servo.get_position(3)
            base_fl_deg = self._servo.get_position(4)
            base_br_deg = self._servo.get_position(1)
            base_bl_deg = self._servo.get_position(2)

            fr_rot = calc_rot(base_fr_deg, base_fr_offset, reverse=False)
            fl_rot = calc_rot(base_fl_deg, base_fl_offset, reverse=True)
            br_rot = calc_rot(base_br_deg, base_br_offset, reverse=False)
            bl_rot = calc_rot(base_bl_deg, base_bl_offset, reverse=True)
            
            fr_diff = right_degrees - fr_rot
            fl_diff = left_degrees - fl_rot
            br_diff = right_degrees - br_rot
            bl_diff = left_degrees - bl_rot
            
            if not ok_fr and fr_diff < 5:
                ok_fr = True
            if not ok_fl and fl_diff < 5:
                ok_fl = True
            if not ok_br and br_diff < 5:
                ok_br = True
            if not ok_bl and bl_diff < 5:
                ok_bl = True

            if ok_fr and ok_fl and ok_br and ok_bl:
                self.stop()
                print("All motors reached target position")
                break

            print(f"FR_diff: {fr_diff}, FL_diff: {fl_diff}, BR_diff: {br_diff}, BL_diff: {bl_diff}")
            time.sleep(0.05)

def main():
    motor = Motor(None)
    try:
        motor.move_tank_for_degrees(5000, 5000, 1000, 1000)
        #time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.stop()
        motor.close()

if __name__ == "__main__":
    main()

