import csv
import time
import sys
import os
from datetime import datetime
from pathlib import Path
import math

# MORAI module import
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "MORAI_UDP_NetworkModule"))

from lib.network.UDP import Receiver
from lib.define.EgoNoisyInfoStatus import EgoNoisyInfoStatus


RECV_IP = "127.0.0.1"
RECV_PORT = 9104


class GlobalPathMaker:
    """
    MORAI Sim에서 Ego 차량 주행 데이터를 수집하여
    path_gui.py에서 사용하는 CSV 형식과 동일하게 저장하는 도구.
    """

    def __init__(self, recv_ip, recv_port):
        print("Initializing GlobalPathMaker...")
        self.receiver = Receiver(recv_ip, recv_port, EgoNoisyInfoStatus())
        self.path_log = []

        # 샘플링 기준 거리 (meter)
        self.min_sampling_distance = 0.8  # 지나치게 촘촘하지 않도록 필터링

        # last stored point
        self.last_x = None
        self.last_y = None

        # 최초 움직임 감지 여부
        self.is_recording_started = False

    def has_position_changed(self, x, y):
        """
        좌표 변화가 있는지 확인
        last_x 또는 last_y가 None이면 초기 수신이므로 True 반환
        """
        if self.last_x is None or self.last_y is None:
            return True

        # 변화 감지 기준: 좌표가 완전히 동일하면 변화 없음
        if abs(x - self.last_x) < 1e-6 and abs(y - self.last_y) < 1e-6:
            return False

        return True

    def run(self):
        print("=" * 60)
        print(" Global Path Maker - Waiting for movement...")
        print(" Waiting for MORAI UDP packets...")
        print(" Start MORAI Simulator and Vehicle!")
        print(" 종료: CTRL+C")
        print("=" * 60)
    
        try:
            while True:
                try:
                    ego = self.receiver.get_data()
                except Exception as e:
                    print(f"Receiver error: {e}")
                    time.sleep(0.1)
                    continue
                
                # 1) 데이터 없음
                if ego is None:
                    time.sleep(0.05)
                    continue
                
                # 2) noisy 데이터 접근 방어
                try:
                    x = float(ego.noisy_pos_e)
                    y = float(ego.noisy_pos_n)
                    z = float(ego.noisy_pos_u)
                except Exception:
                    print("Invalid ego data received... waiting.")
                    time.sleep(0.05)
                    continue
                
                # Z 보정
                if abs(z) < 1e-6:
                    z = -0.500
    
                # 3) 좌표 변화 없으면 기록하지 않음
                if not self.has_position_changed(x, y):
                    time.sleep(0.05)
                    continue
                
                # 최초 움직임 감지
                if not self.is_recording_started:
                    self.is_recording_started = True
                    print("Movement detected → Recording started.")
    
                # 4) 최소 이동 거리 필터링
                if self.last_x is not None:
                    dist = math.hypot(x - self.last_x, y - self.last_y)
                    if dist < self.min_sampling_distance:
                        time.sleep(0.05)
                        continue
                    
                # 5) 정상 기록
                self.path_log.append((round(x, 3), round(y, 3), round(z, 3)))
                print(f"Recorded: ({x:.3f}, {y:.3f}, {z:.3f})  total={len(self.path_log)}")
    
                self.last_x = x
                self.last_y = y
    
                time.sleep(0.05)
    
        except KeyboardInterrupt:
            print("\nRecording stopped by user.")
            self.save_csv()
    
    
    def save_csv(self):
        # 동일한 디렉토리 구조 사용 (path_gui 스타일)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = PROJECT_ROOT / "data"
        output_dir.mkdir(parents=True, exist_ok=True)

        output_file = output_dir / "path.csv"

        with open(output_file, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["LinkID", "PositionX (m)", "PositionY (m)", "PositionZ (m)"])

            for i, (x, y, z) in enumerate(self.path_log):
                link_id = f"AUTO_{i:06d}"
                writer.writerow([link_id, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])

        print(f"Path saved to: {output_file}")
        print(f"Total waypoints: {len(self.path_log)}")
        print("=" * 60)


if __name__ == "__main__":
    maker = GlobalPathMaker(RECV_IP, RECV_PORT)
    maker.run()
