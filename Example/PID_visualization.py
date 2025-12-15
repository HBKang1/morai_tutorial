import csv
import time
import math
import sys
import threading
import queue
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# project_root = 현재 실행 디렉토리
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "MORAI_UDP_NetworkModule"))

from lib.network.UDP import Receiver, Sender
from lib.define.EgoNoisyInfoStatus import EgoNoisyInfoStatus
from lib.define.EgoCtrlCmd import EgoCtrlCmd

RECV_IP = '127.0.0.1'
RECV_PORT = 9104
SEND_PORT = 9093
csv_path = "data/path.csv"

# =====================================================
# Global Path Loader
# =====================================================
def load_global_path(csv_path):
    xs, ys = [], []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['PositionX (m)']))
            ys.append(float(row['PositionY (m)']))
    return xs, ys


# =====================================================
# print_step_log
# =====================================================
def print_step_log(step, x, y, yaw, speed, steer, accel, dt):
    print(f"[Step {step}] Pos=({x:.2f}, {y:.2f})  "
          f"Yaw={math.degrees(yaw):.2f}°  "
          f"Speed={speed:.2f} m/s  "
          f"Steer={math.degrees(steer):.2f}°  "
          f"Accel={accel:.3f}  dt={dt:.3f}")


# =====================================================
# Pure Pursuit Predicted Path
# =====================================================
def predict_pure_pursuit_path(x, y, yaw, steer, wheelbase, horizon=20, ds=0.5):
    k = math.tan(steer * 0.6) / wheelbase
    pred_x = [x]
    pred_y = [y]
    pred_yaw = yaw
    for _ in range(int(horizon / ds)):
        x += ds * math.cos(pred_yaw)
        y += ds * math.sin(pred_yaw)
        pred_yaw += ds * k
        pred_x.append(x)
        pred_y.append(y)
    return pred_x, pred_y


# =====================================================
# Pure Pursuit Controller
# =====================================================
class PurePursuit:
    def __init__(self, lookahead=20.0, wheelbase=2.8):
        self.lookahead = lookahead
        self.wheelbase = wheelbase

    def compute_steering(self, x, y, yaw, path_x, path_y):
        min_dist = float('inf')         
        closest_idx = 0                 
        for i in range(len(path_x)):
            d = (path_x[i] - x)**2 + (path_y[i] - y)**2
            if d < min_dist:
                min_dist = d
                closest_idx = i

        target_idx = closest_idx
        for i in range(closest_idx, len(path_x)):
            if math.hypot(path_x[i] - x, path_y[i] - y) >= self.lookahead:
                target_idx = i
                break

        tx, ty = path_x[target_idx], path_y[target_idx]
        dx = math.cos(-yaw) * (tx - x) - math.sin(-yaw) * (ty - y)
        dy = math.sin(-yaw) * (tx - x) + math.cos(-yaw) * (ty - y)
        steer = math.atan2(2 * self.wheelbase * dy, self.lookahead**2)
        print(f"dx = {dx}, dy = {dy}, lookahead = {self.lookahead}, steer = {steer}")
        return max(min(steer / 0.6, 1.0), -1.0)
    


# =====================================================
# PID Controller
# =====================================================
class PID:
    def __init__(self, kp, ki, kd, target_speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_speed = target_speed
        self.integral = 0.0
        self.prev_error = 0.0

        self.log_error = []
        self.log_P = []
        self.log_I = []
        self.log_D = []
        self.log_output = []
        self.log_target_speed = []
        self.log_current_speed = []

    def compute(self, current_speed, dt):
        if dt <= 0:
            derivative = 0.0
        else:
            error = self.target_speed - current_speed
            derivative = (error - self.prev_error) / dt
            self.prev_error = error

        error = self.target_speed - current_speed
        self.integral += error * max(dt, 1e-6)

        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * derivative
        output = P + I + D

        self.log_error.append(error)
        self.log_P.append(P)
        self.log_I.append(I)
        self.log_D.append(D)
        self.log_output.append(output)
        self.log_target_speed.append(self.target_speed)
        self.log_current_speed.append(current_speed)

        return max(min(output, 1.0), 0.0)


# =====================================================
# Controller Worker
# =====================================================
def controller_worker(path_x, path_y, state_q: queue.Queue, stop_event: threading.Event):
    receiver = Receiver(RECV_IP, RECV_PORT, EgoNoisyInfoStatus())
    sender = Sender(RECV_IP, SEND_PORT)
    pp = PurePursuit(lookahead=20.0)
    
    # PID Gain 튜닝값
    pid = PID(kp=2.0, ki=0, kd=0, target_speed=150.0)

    prev_time = time.time()
    step = 0

    while not stop_event.is_set():
        step += 1
        now = time.time()
        dt = now - prev_time
        prev_time = now

        if dt > 1.0: dt = 0.01

        try:
            ego = receiver.get_data()
        except Exception:
            time.sleep(0.1)
            continue

        x, y = ego.noisy_pos_e, ego.noisy_pos_n
        yaw = math.radians(ego.noisy_ori_y)
        speed = math.hypot(ego.noisy_vel_e, ego.noisy_vel_n)

        accel = pid.compute(speed, dt)
        steer = pp.compute_steering(x, y, yaw, path_x, path_y)

        cmd = EgoCtrlCmd()
        cmd.ctrl_mode = 2
        cmd.cmd_type = 1
        cmd.gear = 4
        cmd.accel = accel
        cmd.brake = 0.0
        cmd.steer = steer
        try:
            sender.send(cmd)
        except Exception:
            pass

        print_step_log(step, x, y, yaw, speed, steer, accel, dt)

        snapshot = {
            'step': step,
            'p_term': list(pid.log_P),
            'i_term': list(pid.log_I),
            'd_term': list(pid.log_D),
            'error': list(pid.log_error),
            'current': list(pid.log_current_speed),
            'target': list(pid.log_target_speed),
        }

        try:
            if state_q.full():
                try: state_q.get_nowait()
                except queue.Empty: pass
            state_q.put_nowait(snapshot)
        except Exception:
            pass

# =====================================================
# Main Thread: Hybrid Visualization (P,I=Full / D=Sliding)
# =====================================================
def gui_loop(path_x, path_y, state_q: queue.Queue, stop_event: threading.Event):
    plt.ion()
    plot = False
    
    fig, axs = plt.subplots(4, 1, figsize=(16, 10))
    fig.suptitle("PID Analysis: P/I (Full History) vs D (Sliding Window)", fontsize=16)

    ax_p = axs[0]
    ax_i = axs[1]
    ax_d = axs[2]
    ax_speed = axs[3]

    # 라벨 설정
    ax_p.set_ylabel("P Value", color='red', fontweight='bold')
    ax_i.set_ylabel("I Value", color='green', fontweight='bold')
    ax_d.set_ylabel("D Value", color='blue', fontweight='bold')

    # --- 1. P-Term (Full History) ---
    ax_p.set_title("1. P-Term (Full History)")
    ax_p.grid(True)
    line_p, = ax_p.plot([], [], color='r', linewidth=2)
    ax_p_err = ax_p.twinx()
    ax_p_err.set_ylabel("Error", color='gray')
    line_p_err, = ax_p_err.plot([], [], color='gray', linestyle=':', alpha=0.5)

    # --- 2. I-Term (Full History) ---
    ax_i.set_title("2. I-Term (Full History)")
    ax_i.grid(True)
    line_i, = ax_i.plot([], [], color='g', linewidth=2)
    ax_i_err = ax_i.twinx()
    ax_i_err.set_ylabel("Error", color='gray')
    line_i_err, = ax_i_err.plot([], [], color='gray', linestyle=':', alpha=0.5)

    # --- 3. D-Term (Sliding Window) ---
    ax_d.set_title("3. D-Term (Recent 1000 Steps)", color='blue')
    ax_d.grid(True)
    line_d, = ax_d.plot([], [], color='b', linewidth=2)
    ax_d_err = ax_d.twinx()
    ax_d_err.set_ylabel("Error", color='gray')
    line_d_err, = ax_d_err.plot([], [], color='gray', linestyle=':', alpha=0.5)

    # --- 4. Speed (Full History) ---
    ax_speed.set_title("4. Speed Tracking (Full History)")
    ax_speed.set_ylabel("Speed (km/h)", color='black')
    ax_speed.grid(True)
    line_target, = ax_speed.plot([], [], color='black', linestyle='--', linewidth=2, label='Target')
    line_current, = ax_speed.plot([], [], color='magenta', linewidth=2, label='Current')
    ax_speed.legend(loc='lower right')

    plt.tight_layout()
    plt.show(block=False)

    last_snapshot = None
    
    # D 그래프용 슬라이딩 윈도우 크기
    D_WINDOW_SIZE = 1000

    # Y축 자동 스케일링 함수
    def set_smart_ylim(ax, data, margin_ratio=0.1):
        if not data: return
        mn, mx = min(data), max(data)
        span = mx - mn
        if span == 0:
            offset = 0.01 if mx == 0 else abs(mx) * 0.1
            ax.set_ylim(mn - offset, mx + offset)
        else:
            margin = span * margin_ratio
            ax.set_ylim(mn - margin, mx + margin)

    try:
        while not stop_event.is_set():
            try:
                snapshot = state_q.get_nowait()
                last_snapshot = snapshot
            except queue.Empty:
                snapshot = last_snapshot

            if snapshot is not None:
                p_data = snapshot.get('p_term', [])
                i_data = snapshot.get('i_term', [])
                d_data = snapshot.get('d_term', [])
                err_data = snapshot.get('error', [])
                cur_data = snapshot.get('current', [])
                tgt_data = snapshot.get('target', [])

                full_len = len(p_data)
                xs_full = range(full_len)

                if full_len > 0:
                    # 1. P Graph (Full)
                    line_p.set_data(xs_full, p_data)
                    line_p_err.set_data(xs_full, err_data)
                    
                    ax_p.set_xlim(0, full_len)
                    set_smart_ylim(ax_p, p_data)
                    set_smart_ylim(ax_p_err, err_data)

                    # 2. I Graph (Full)
                    line_i.set_data(xs_full, i_data)
                    line_i_err.set_data(xs_full, err_data)
                    
                    ax_i.set_xlim(0, full_len)
                    set_smart_ylim(ax_i, i_data)
                    ax_i_err.set_ylim(ax_p_err.get_ylim())

                    # 3. D Graph (Sliding)
                    if full_len > D_WINDOW_SIZE:
                        xs_d = range(full_len - D_WINDOW_SIZE, full_len)
                        d_view = d_data[-D_WINDOW_SIZE:]
                        err_view_d = err_data[-D_WINDOW_SIZE:]
                    else:
                        xs_d = xs_full
                        d_view = d_data
                        err_view_d = err_data

                    line_d.set_data(xs_d, d_view)
                    line_d_err.set_data(xs_d, err_view_d)
                    
                    ax_d.set_xlim(xs_d[0], xs_d[-1])
                    set_smart_ylim(ax_d, d_view) 
                    set_smart_ylim(ax_d_err, err_view_d)

                    # 4. Speed Graph (Full)
                    line_target.set_data(xs_full, tgt_data)
                    line_current.set_data(xs_full, cur_data)
                    
                    ax_speed.set_xlim(0, full_len)
                    if len(tgt_data) > 0:
                        all_vals = tgt_data + cur_data
                        set_smart_ylim(ax_speed, all_vals)

                fig.canvas.draw_idle()

                if len(cur_data) > 10000 and plot is False:
                    plt.figure()
                    plt.plot(xs_full, cur_data)
                    plt.plot(xs_full, tgt_data)
                    plot = True
                    plt.show()


            plt.pause(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.close('all')


if __name__ == '__main__':
    path_x, path_y = load_global_path(csv_path)
    state_q = queue.Queue(maxsize=1)
    stop_event = threading.Event()

    worker_thread = threading.Thread(
        target=controller_worker,
        args=(path_x, path_y, state_q, stop_event),
        daemon=True
    )
    worker_thread.start()

    try:
        gui_loop(path_x, path_y, state_q, stop_event)
    finally:
        stop_event.set()
        worker_thread.join(timeout=1.0)
        print("Exited cleanly.")