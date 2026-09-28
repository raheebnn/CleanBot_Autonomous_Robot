"""
rpi5_sender.py
==============
Terminal 1 on RPi5. Run BEFORE rpi5_motor_brain_slam.py.

Sends to PC:
  camera frames  → PC :5000  @ 25 fps   ← CHANGED from 20
  IMU data       → PC :5001  @ 100 Hz
  YOLO dets      → PC :5002  @ 10 Hz

Receives from PC:
  SLAM pose      ← PC :5003  ("x y z state")

Forwards to motor brain (localhost same RPi5):
  pose           → localhost :6100
  detections     → localhost :6101

CHANGES vs previous version:
  - 25 fps  (FRAME_INTERVAL = 1/25)
  - pose forwarding confirmed working with debug counter
"""

import os
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
os.environ["QT_QPA_PLATFORM"]  = "xcb"

from picamera2 import Picamera2
from mpu6050  import mpu6050
import numpy as np
import cv2, time, threading, json, torch, pickle, io, socket, struct, math

# ══════════════════════════════════════════════════════
# NETWORK
# ══════════════════════════════════════════════════════
PC_IP     = os.environ.get("CLEANBOT_PC_IP", "192.168.0.107")   # ← your PC IP (or set CLEANBOT_PC_IP)
DATA_DIR  = os.environ.get("CLEANBOT_DATA_DIR", "/home/pi")     # calibration + model files
CAM_PORT  = 5000
IMU_PORT  = 5001
DET_PORT  = 5002
POSE_PORT = 5003   # recv pose ← PC

LOCAL_POSE_PORT = 6100   # → motor brain
LOCAL_DET_PORT  = 6101   # → motor brain

DEG2RAD = math.pi / 180.0

# ── Send sockets ──────────────────────────────────────
cam_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
det_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cam_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4*1024*1024)
imu_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64*1024)

# Local forwarding sockets → motor brain (same RPi5)
local_pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
local_det_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ── Receive socket (pose back from PC) ────────────────
pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pose_sock.bind(("0.0.0.0", POSE_PORT))
pose_sock.settimeout(0.005)

# ══════════════════════════════════════════════════════
# LOAD MODELS
# ══════════════════════════════════════════════════════
class CPUUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module == 'torch.storage' and name == '_load_from_bytes':
            return lambda b: torch.load(
                io.BytesIO(b), map_location='cpu', weights_only=False)
        return super().find_class(module, name)

print("Loading calibrations...")
try:
    K         = np.load(os.path.join(DATA_DIR, 'camera_matrix.npy'))
    dist_coef = np.load(os.path.join(DATA_DIR, 'dist_coefficients.npy'))
    newK, _   = cv2.getOptimalNewCameraMatrix(K, dist_coef, (640,480), 1, (640,480))
    print("  [OK] Camera calibration")
except:
    K = dist_coef = newK = None
    print("  [--] No camera calibration")

try:
    with open(os.path.join(DATA_DIR, 'imu_calibration.json')) as f:
        cal = json.load(f)
    if 'accelerometer' in cal:
        # Format written by imu_calibration/imu_calibration_mpu6050.py
        # (accel bias in g, gyro bias in deg/s). mpu6050 lib reports m/s².
        accel_bias = np.array(cal['accelerometer']['bias_g']) * 9.80665
        gyro_bias  = np.array(cal['gyroscope']['bias_dps'])
    else:
        # Legacy flat format: {"accel_bias": [...], "gyro_bias": [...]}
        accel_bias = np.array(cal['accel_bias'])
        gyro_bias  = np.array(cal['gyro_bias'])
    print("  [OK] IMU calibration")
except:
    accel_bias = np.zeros(3); gyro_bias = np.zeros(3)
    print("  [--] No IMU calibration")

print("Loading YOLO...")
try:
    with open(os.path.join(DATA_DIR, 'yolo_model26s.pkl'), 'rb') as f:
        model = CPUUnpickler(f).load()
    torch.set_num_threads(4)
    yolo_ok = True
    print("  [OK] YOLO loaded")
except Exception as e:
    model = None; yolo_ok = False
    print(f"  [--] YOLO failed: {e}")

CLASSES = {0: "liquid_obstacle", 1: "solid_dirt"}
COLORS  = {"liquid_obstacle": (0,140,255), "solid_dirt": (0,0,220)}

# ══════════════════════════════════════════════════════
# INIT CAMERA  — 25 fps
# ══════════════════════════════════════════════════════
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640,480), "format": "BGR888"},
    controls={
        "FrameRate": 25,        # ← 25 fps
        "AeEnable": True, "AwbEnable": True,
        "AeMeteringMode": 0, "AeExposureMode": 0,
        "Brightness": 0.1, "Contrast": 1.2, "Sharpness": 2.0,
    }
))
picam2.start(); time.sleep(2)
print("  [OK] Camera ready @ 25fps")

# ══════════════════════════════════════════════════════
# INIT IMU
# ══════════════════════════════════════════════════════
try:
    imu = mpu6050(0x68)
    imu.set_gyro_range(imu.GYRO_RANGE_250DEG)
    imu.set_accel_range(imu.ACCEL_RANGE_4G)
    imu_ok = True
    print("  [OK] MPU6050 ready")
except Exception as e:
    imu = None; imu_ok = False
    print(f"  [--] IMU failed: {e}")

# ══════════════════════════════════════════════════════
# SHARED STATE
# ══════════════════════════════════════════════════════
latest_frame = None
frame_lock   = threading.Lock()
detections   = []
detect_lock  = threading.Lock()
slam_pose    = {"x": 0.0, "y": 0.0, "z": 0.0, "state": 3}
pose_lock    = threading.Lock()
running      = True

# Pose forwarding debug counter
_pose_fwd_count = 0

# ══════════════════════════════════════════════════════
# IMU THREAD — 100 Hz
# ══════════════════════════════════════════════════════
def imu_thread():
    interval = 0.01
    while running:
        t0 = time.time()
        try:
            if imu_ok:
                accel = imu.get_accel_data()
                gyro  = imu.get_gyro_data()
                ax = accel['x'] - accel_bias[0]
                ay = accel['y'] - accel_bias[1]
                az = accel['z'] - accel_bias[2]
                gx = (gyro['x'] - gyro_bias[0]) * DEG2RAD
                gy = (gyro['y'] - gyro_bias[1]) * DEG2RAD
                gz = (gyro['z'] - gyro_bias[2]) * DEG2RAD
            else:
                ax=ay=gx=gy=gz=0.0; az=9.81
            pkt = struct.pack('d6f', time.time(), ax, ay, az, gx, gy, gz)
            imu_sock.sendto(pkt, (PC_IP, IMU_PORT))
        except Exception as e:
            print(f"[IMU] {e}")
        elapsed = time.time() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)

# ══════════════════════════════════════════════════════
# DETECTION THREAD — 10 Hz
# ══════════════════════════════════════════════════════
def detection_thread():
    if not yolo_ok:
        return
    while running:
        try:
            with frame_lock:
                if latest_frame is None:
                    time.sleep(0.05); continue
                frame = latest_frame.copy()
            results = model.predict(frame, conf=0.45, imgsz=320,
                                    verbose=False, device='cpu')
            found = []
            for r in results:
                for box in r.boxes:
                    cls          = int(box.cls[0])
                    conf         = float(box.conf[0])
                    x1,y1,x2,y2 = map(int, box.xyxy[0])
                    x1=max(0,min(639,x1)); y1=max(0,min(479,y1))
                    x2=max(0,min(639,x2)); y2=max(0,min(479,y2))
                    found.append({"label": CLASSES.get(cls,"unknown"),
                                  "conf":  round(conf,3),
                                  "box":   [x1,y1,x2,y2]})
            with detect_lock:
                detections.clear(); detections.extend(found)

            # Send to PC
            det_sock.sendto(json.dumps(found).encode(), (PC_IP, DET_PORT))
            # Forward to motor brain on localhost
            local_det_sock.sendto(json.dumps(found).encode(),
                                  ("127.0.0.1", LOCAL_DET_PORT))
        except Exception as e:
            print(f"[YOLO] {e}")
        time.sleep(0.1)

# ══════════════════════════════════════════════════════
# POSE RECEIVE THREAD
# Receives "x y z state" from PC on port 5003
# Forwards SAME message to motor brain on localhost:6100
# ══════════════════════════════════════════════════════
def recv_pose_thread():
    global _pose_fwd_count
    STATE_NAMES = {0:"TRACKING",1:"RECENTLY_LOST",2:"LOST",3:"INITIALIZING"}
    print(f"[POSE] Listening on 0.0.0.0:{POSE_PORT}  (from PC)")
    while running:
        try:
            data, addr = pose_sock.recvfrom(128)
            parts = data.decode().strip().split()
            if len(parts) >= 4:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                s = int(parts[3])

                # Update local state
                with pose_lock:
                    slam_pose.update({"x":x,"y":y,"z":z,"state":s})

                # ── Forward to motor brain (localhost:6100) ──
                # Use the exact same bytes received from PC
                local_pose_sock.sendto(data, ("127.0.0.1", LOCAL_POSE_PORT))
                _pose_fwd_count += 1

                # Debug — log first pose and then every 100
                if _pose_fwd_count == 1:
                    print(f"[POSE] ✓ First pose forwarded to localhost:{LOCAL_POSE_PORT}  "
                          f"state={STATE_NAMES.get(s,'?')}  x={x:.3f} z={z:.3f}")
                elif _pose_fwd_count % 100 == 0:
                    print(f"[POSE] Forwarded {_pose_fwd_count} poses  "
                          f"state={STATE_NAMES.get(s,'?')}")

        except socket.timeout:
            pass
        except Exception as e:
            print(f"[POSE] Error: {e}")

# ══════════════════════════════════════════════════════
# START THREADS
# ══════════════════════════════════════════════════════
threading.Thread(target=imu_thread,       daemon=True).start()
threading.Thread(target=detection_thread, daemon=True).start()
threading.Thread(target=recv_pose_thread, daemon=True).start()

STATE_NAMES  = {0:"TRACKING", 1:"RECENTLY_LOST", 2:"LOST", 3:"INITIALIZING"}
STATE_COLORS = {0:(0,255,0), 1:(0,200,255), 2:(0,0,255), 3:(0,255,255)}

# ══════════════════════════════════════════════════════
# MAIN LOOP — 25 fps
# ══════════════════════════════════════════════════════
FRAME_INTERVAL = 1.0 / 25.0    # ← 25 fps

fps=0; fps_count=0; fps_time=time.time(); frames_sent=0

print("="*58)
print("  RPi5 SENDER — 25 fps")
print(f"  Camera  → {PC_IP}:{CAM_PORT} @ 25fps")
print(f"  IMU     → {PC_IP}:{IMU_PORT} @ 100Hz")
print(f"  YOLO    → {PC_IP}:{DET_PORT} @ 10Hz")
print(f"  Pose   ← 0.0.0.0:{POSE_PORT}  (from PC)")
print(f"  Pose   → localhost:{LOCAL_POSE_PORT}  (motor brain)")
print(f"  Dets   → localhost:{LOCAL_DET_PORT}   (motor brain)")
print("  Run motor brain in Terminal 2:")
print("    python3 rpi5_motor_brain_slam.py")
print("="*58)

try:
    while True:
        t_loop = time.time()

        frame = picam2.capture_array()
        if K is not None:
            frame = cv2.undistort(frame, K, dist_coef, None, newK)

        with frame_lock:
            latest_frame = frame.copy()

        _, jpg = cv2.imencode('.jpg', frame,
                              [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        jpg_bytes = jpg.tobytes()
        if len(jpg_bytes) >= 60000:
            _, jpg = cv2.imencode('.jpg', frame,
                                  [int(cv2.IMWRITE_JPEG_QUALITY), 55])
            jpg_bytes = jpg.tobytes()

        cam_sock.sendto(struct.pack('d', time.time()) + jpg_bytes,
                        (PC_IP, CAM_PORT))
        frames_sent += 1

        with detect_lock: cur_dets = list(detections)
        with pose_lock:   pose     = dict(slam_pose)

        # ── Preview window ────────────────────────────
        preview = frame.copy()

        for det in cur_dets:
            x1, y1, x2, y2 = det["box"]
            label = det["label"]
            conf  = det["conf"]
            col   = COLORS.get(label, (255, 255, 255))

            # ── 1. Filled semi-transparent box overlay ────────────────
            overlay = preview.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), col, -1)
            cv2.addWeighted(overlay, 0.10, preview, 0.90, 0, preview)

            # ── 2. Solid border rectangle (3 px) ─────────────────────
            cv2.rectangle(preview, (x1, y1), (x2, y2), col, 3)

            # ── 3. Corner tick marks — L-shaped brackets ──────────────
            # Length of each corner tick = 15% of the shorter side, min 12 px
            tick = max(12, int(min(x2-x1, y2-y1) * 0.15))
            tk   = 4   # tick line thickness
            corners = [
                # top-left
                ((x1, y1+tick), (x1, y1), (x1+tick, y1)),
                # top-right
                ((x2-tick, y1), (x2, y1), (x2, y1+tick)),
                # bottom-left
                ((x1, y2-tick), (x1, y2), (x1+tick, y2)),
                # bottom-right
                ((x2-tick, y2), (x2, y2), (x2, y2-tick)),
            ]
            for p1, vertex, p2 in corners:
                cv2.line(preview, p1, vertex, (255,255,255), tk)
                cv2.line(preview, vertex, p2, (255,255,255), tk)

            # ── 4. Filled label pill (background + text) ─────────────
            tag    = f"{label.replace('_',' ').upper()}  {int(conf*100)}%"
            font   = cv2.FONT_HERSHEY_SIMPLEX
            fscale = 0.52
            fthick = 1
            (tw, th), baseline = cv2.getTextSize(tag, font, fscale, fthick)
            pad    = 5
            # Position label above box; clamp so it never goes off the top
            ly1    = max(y1 - th - 2*pad, 0)
            ly2    = ly1 + th + 2*pad
            lx2    = min(x1 + tw + 2*pad, 639)
            lx1    = max(lx2 - tw - 2*pad, 0)
            # Filled background
            cv2.rectangle(preview, (lx1, ly1), (lx2, ly2), col, -1)
            # White text on top
            cv2.putText(preview, tag,
                        (lx1 + pad, ly2 - pad),
                        font, fscale, (255, 255, 255), fthick, cv2.LINE_AA)

            # ── 5. Confidence bar (thin strip at bottom of box) ───────
            bar_h   = 5
            bar_w   = x2 - x1
            bar_fill= int(bar_w * conf)
            cv2.rectangle(preview,
                           (x1, y2 - bar_h), (x2, y2),
                           (40, 40, 40), -1)                       # background
            cv2.rectangle(preview,
                           (x1, y2 - bar_h), (x1 + bar_fill, y2),
                           col, -1)                                 # fill

            # ── 6. Centre crosshair (dirt only — this is the drive target) ─
            if label == "solid_dirt":
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                arm = 10   # crosshair arm length
                cv2.line(preview, (cx-arm, cy), (cx+arm, cy), (255,255,255), 2)
                cv2.line(preview, (cx, cy-arm), (cx, cy+arm), (255,255,255), 2)
                cv2.circle(preview, (cx, cy), 4, col, -1)
                cv2.circle(preview, (cx, cy), 4, (255,255,255), 1)

        # ── Status bar ────────────────────────────────────────────────
        s    = pose["state"]
        scol = STATE_COLORS.get(s, (180,180,180))
        snam = STATE_NAMES.get(s, "?")
        # Slightly taller bar so text doesn't crowd
        cv2.rectangle(preview, (0, 0), (640, 36), (15, 15, 15), -1)
        cv2.putText(preview,
                    f"SLAM:{snam}  X:{pose['x']:.2f}  Z:{pose['z']:.2f}",
                    (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, scol, 1,
                    cv2.LINE_AA)
        # Detection count badge
        n_dirt   = sum(1 for d in cur_dets if d["label"]=="solid_dirt")
        n_liquid = sum(1 for d in cur_dets if d["label"]=="liquid_obstacle")
        det_str  = ""
        if n_dirt:
            det_str += f"  DIRT:{n_dirt}"
        if n_liquid:
            det_str += f"  LIQUID:{n_liquid}"
        cv2.putText(preview,
                    f"Sent:{frames_sent}  FPS:{int(fps)}  Fwd:{_pose_fwd_count}"
                    + det_str,
                    (6, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 150, 150), 1,
                    cv2.LINE_AA)

        fps_count += 1
        now = time.time()
        if now - fps_time >= 1.0:
            fps = fps_count / (now - fps_time)
            fps_count = 0; fps_time = now

        cv2.imshow("RPi5 Sender — 25fps", preview)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        elapsed = time.time() - t_loop
        if elapsed < FRAME_INTERVAL:
            time.sleep(FRAME_INTERVAL - elapsed)

except KeyboardInterrupt:
    print("\nStopped")
finally:
    running = False
    time.sleep(0.3)
    cam_sock.close(); imu_sock.close()
    det_sock.close(); pose_sock.close()
    local_pose_sock.close(); local_det_sock.close()
    picam2.stop(); picam2.close()
    cv2.destroyAllWindows()
    print("Done!")
