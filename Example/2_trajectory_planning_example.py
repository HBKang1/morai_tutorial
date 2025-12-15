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

        # 로그
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
# Controller Worker (백그라운드 스레드)
# =====================================================
def controller_worker(path_x, path_y, state_q: queue.Queue, stop_event: threading.Event):
    receiver = Receiver(RECV_IP, RECV_PORT, EgoNoisyInfoStatus())
    sender = Sender(RECV_IP, SEND_PORT)
    pp = PurePursuit(lookahead=20.0)
    pid = PID(kp=0.3, ki=0, kd=0.05, target_speed=150.0)

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
        except Exception as e:
            print("Receiver error:", e)
            time.sleep(0.01)
            continue

        x, y = ego.noisy_pos_e, ego.noisy_pos_n
        yaw = math.radians(ego.noisy_ori_y)
        speed = math.hypot(ego.noisy_vel_e, ego.noisy_vel_n)

        accel = pid.compute(speed, dt)
        steer = pp.compute_steering(x, y, yaw, path_x, path_y)

        pred_x, pred_y = predict_pure_pursuit_path(x, y, yaw, steer, pp.wheelbase, horizon=20, ds=0.3)

        cmd = EgoCtrlCmd()
        cmd.ctrl_mode = 2
        cmd.cmd_type = 1
        cmd.gear = 4
        cmd.accel = accel
        cmd.brake = 0.0
        cmd.steer = steer
        try:
            sender.send(cmd)
        except Exception as e:
            print("Sender error:", e)

        print_step_log(step, x, y, yaw, speed, steer, accel, dt)

        snapshot = {
            'step': step,
            'x': x, 'y': y, 'yaw': yaw, 'speed': speed,
            'steer': steer, 'accel': accel, 'dt': dt,
            'pred_x': list(pred_x), 'pred_y': list(pred_y),
            'pid_error': list(pid.log_error),
            'pid_output': list(pid.log_output),
            'target_speed': list(pid.log_target_speed),
            'current_speed': list(pid.log_current_speed),
        }

        try:
            if state_q.full():
                try: state_q.get_nowait()
                except queue.Empty: pass
            state_q.put_nowait(snapshot)
        except Exception:
            pass


# =====================================================
# Main Thread: Matplotlib GUI (수정됨)
# =====================================================
def gui_loop(path_x, path_y, state_q: queue.Queue, stop_event: threading.Event):
    plt.ion()
    
    # 레이아웃 설정: 2행 2열
    # (0,0): Global Map, (1,0): Local Map (Zoomed)
    # (0,1): PID Error,  (1,1): PID Speed
    fig = plt.figure(constrained_layout=True, figsize=(14, 9))
    gs = fig.add_gridspec(2, 2)

    # 1. Global Map (전체 경로)
    ax_global = fig.add_subplot(gs[0, 0])
    ax_global.set_title("Global Path Map")
    ax_global.plot(path_x, path_y, label="Global Path", linewidth=1, color='gray')
    line_pred_g, = ax_global.plot([], [], 'r', label="Predicted", linewidth=2)
    ego_point_g, = ax_global.plot([], [], 'bo', label="Ego", markersize=5)
    ax_global.legend(loc='upper right')
    ax_global.set_aspect('equal', adjustable='datalim')
    ax_global.grid(True)

    # 2. Local Map (차량 기준 확대, 추적)
    ax_local = fig.add_subplot(gs[1, 0])
    ax_local.set_title("Local Map (Tracking)")
    ax_local.plot(path_x, path_y, label="Global Path", linewidth=1, color='gray', linestyle='--')
    line_pred_l, = ax_local.plot([], [], 'r', label="Predicted", linewidth=3)
    ego_point_l, = ax_local.plot([], [], 'bo', label="Ego", markersize=8)
    
    # 화살표로 차량 방향 표시 (quiver)
    quiver_l = ax_local.quiver([0], [0], [1], [0], color='blue', scale=20)
    
    ax_local.set_aspect('equal')
    ax_local.grid(True)
    # 초기 범위 설정 (나중에 update에서 덮어씌워짐)
    ax_local.set_xlim(-50, 50)
    ax_local.set_ylim(-50, 50)


    # 3. PID Graphs
    ax_pid_error = fig.add_subplot(gs[0, 1])
    ax_pid_error.set_title("PID Error & Output")
    line_err, = ax_pid_error.plot([], [], label='Error (Target - Current)')
    line_out, = ax_pid_error.plot([], [], label='Output (Accel Cmd)')
    ax_pid_error.legend(loc='upper right')
    ax_pid_error.grid(True)

    ax_pid_speed = fig.add_subplot(gs[1, 1])
    ax_pid_speed.set_title("Speed Tracking")
    line_t_speed, = ax_pid_speed.plot([], [], 'g--', label='Target Speed')
    line_c_speed, = ax_pid_speed.plot([], [], 'b', label='Current Speed')
    ax_pid_speed.legend(loc='lower right')
    ax_pid_speed.grid(True)

    plt.show(block=False)

    last_snapshot = None
    try:
        while not stop_event.is_set():
            try:
                snapshot = state_q.get_nowait()
                last_snapshot = snapshot
            except queue.Empty:
                snapshot = last_snapshot

            if snapshot is not None:
                x = snapshot['x']
                y = snapshot['y']
                yaw = snapshot['yaw']
                pred_x = snapshot['pred_x']
                pred_y = snapshot['pred_y']

                # --- 1. Global Map Update ---
                line_pred_g.set_xdata(pred_x)
                line_pred_g.set_ydata(pred_y)
                ego_point_g.set_xdata([x])
                ego_point_g.set_ydata([y])

                # --- 2. Local Map Update ---
                # 데이터 업데이트
                line_pred_l.set_xdata(pred_x)
                line_pred_l.set_ydata(pred_y)
                ego_point_l.set_xdata([x])
                ego_point_l.set_ydata([y])
                
                # Quiver (화살표) 업데이트: 차량의 헤딩 방향
                quiver_l.set_offsets([x, y])
                quiver_l.set_UVC(math.cos(yaw), math.sin(yaw))

                # *** View Tracking Logic (핵심) ***
                # 요구사항: 전방 80m, 후방 20m, 좌우 20m (대략적인 비율)
                # 이를 위해 차량 위치에서 Yaw 방향으로 30m만큼 중심을 이동시킴
                # (전방 80 - 후방 20) / 2 = +30m 오프셋
                
                offset_dist = 30.0
                center_x = x + offset_dist * math.cos(yaw)
                center_y = y + offset_dist * math.sin(yaw)

                # 보여줄 박스 크기: 100m x 100m (전방 80+후방 20 = 100m 커버)
                view_radius = 50.0  # 반경 50m (지름 100m)
                
                ax_local.set_xlim(center_x - view_radius, center_x + view_radius)
                ax_local.set_ylim(center_y - view_radius, center_y + view_radius)


                # --- 3. PID Graphs Update ---
                err = snapshot.get('pid_error', [])
                out = snapshot.get('pid_output', [])
                t_spd = snapshot.get('target_speed', [])
                c_spd = snapshot.get('current_speed', [])
                
                # 데이터 길이 맞추기 (시각화용)
                display_len = 500  # 최근 500개만 보여주기 (속도 최적화)
                if len(err) > display_len:
                    xs = range(len(err) - display_len, len(err))
                    line_err.set_data(xs, err[-display_len:])
                    line_out.set_data(xs, out[-display_len:])
                    line_t_speed.set_data(xs, t_spd[-display_len:])
                    line_c_speed.set_data(xs, c_spd[-display_len:])
                    
                    ax_pid_error.set_xlim(xs.start, xs.stop)
                    ax_pid_speed.set_xlim(xs.start, xs.stop)
                else:
                    xs = range(len(err))
                    line_err.set_data(xs, err)
                    line_out.set_data(xs, out)
                    line_t_speed.set_data(xs, t_spd)
                    line_c_speed.set_data(xs, c_spd)
                    
                    ax_pid_error.relim()
                    ax_pid_error.autoscale_view()
                    ax_pid_speed.relim()
                    ax_pid_speed.autoscale_view()

                # 전체 캔버스 갱신
                fig.canvas.draw_idle()

            plt.pause(0.03)

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