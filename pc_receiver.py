"""
pc_receiver.py
==============
Run on PC.

Run order:
  Terminal 1:  cd ~/ORB_SLAM3
               export LD_LIBRARY_PATH=~/ORB_SLAM3/lib:/usr/local/lib:$LD_LIBRARY_PATH
               ./Examples/Monocular-Inertial/build/slam_receiver \
                   Vocabulary/ORBvoc.txt \
                   Examples/Monocular-Inertial/webcam.yaml
  Terminal 2:  python3 pc_receiver.py

Receives:
  camera frames  ← RPi5 :5000
  IMU            ← RPi5 :5001
  YOLO dets      ← RPi5 :5002
  SLAM pose      ← slam_receiver :9999
  SLAM map pts   ← slam_receiver :9996

Forwards:
  frames → slam_receiver :9997  @ 25fps
  IMU    → slam_receiver :9998  @ 100Hz
  pose   → RPi5          :5003

CHANGES vs previous:
  - 25 fps frame forwarding (INTERVAL = 1/25)
  - Open3D point-cloud viewer runs in a SEPARATE PROCESS so its
    OpenGL context never conflicts with OpenCV's X11 window.
    Press  P  in the OpenCV window to open/refresh the 3-D viewer.
"""

# ── must set Qt platform before any GUI import ────────────────────
import os
os.environ["QT_QPA_PLATFORM"]  = "xcb"
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
os.environ["DISPLAY"]          = os.environ.get("DISPLAY", ":0")

import numpy as np
import cv2
import time
import threading
import heapq
import socket
import struct
import json
import multiprocessing as mp

# ══════════════════════════════════════════════════════
# OPEN3D VIEWER — runs in a child PROCESS
# ══════════════════════════════════════════════════════
def _open3d_process(queue: mp.Queue):
    """
    Standalone process that owns the Open3D OpenGL context.

    Improvements over v1:
      - 1280x720 window
      - Height-based coloring (Y axis): blue=floor, green=mid, red=ceiling
      - Voxel downsampling: caps at ~50 000 pts for smooth rendering
      - Coordinate frame axes drawn at origin
      - Bounding-box overlay so you can gauge the room scale
      - Point size 3.0 for better visibility
      - Prints live stats (pts, bbox) every 5 s
      - Graceful exit when window is closed by user
      - X11/XWayland forced; headless EGL fallback if display unavailable
    """
    import os as _os

    # ── Force GLFW onto X11 (XWayland) — must be set BEFORE open3d import ──
    _os.environ["DISPLAY"]                  = _os.environ.get("DISPLAY", ":0")
    _os.environ["PYOPENGL_PLATFORM"]        = "egl"
    _os.environ["MESA_GL_VERSION_OVERRIDE"] = "3.3"
    _os.environ["GLFW_PLATFORM"]            = "x11"
    _os.environ["GDK_BACKEND"]              = "x11"
    _os.environ["QT_QPA_PLATFORM"]          = "xcb"

    try:
        import open3d as o3d
    except ImportError:
        print("[O3D ] open3d not installed — pip install open3d")
        return

    # ── Try windowed mode ────────────────────────────────────────────────────
    vis = None
    try:
        vis = o3d.visualization.Visualizer()
        vis.create_window("SLAM Point Cloud  |  scroll=zoom  drag=rotate",
                          width=1280, height=720)
        if not vis.poll_events():
            raise RuntimeError("poll_events() returned False immediately")
        print("[O3D ] Windowed viewer opened (1280×720, X11/XWayland)")
    except Exception as e:
        print(f"[O3D ] Windowed viewer failed ({e}), switching to headless mode")
        if vis is not None:
            try: vis.destroy_window()
            except: pass
        vis = None

    # ── Headless / EGL fallback ──────────────────────────────────────────────
    headless = vis is None
    renderer = None
    if headless:
        try:
            renderer = o3d.visualization.rendering.OffscreenRenderer(1280, 720)
            mat = o3d.visualization.rendering.MaterialRecord()
            mat.shader = "defaultUnlit"
            print("[O3D ] Headless OffscreenRenderer active")
        except Exception as e:
            print(f"[O3D ] Headless renderer also failed: {e}")
            while True:
                try:
                    item = queue.get(timeout=1)
                    if item == "STOP": break
                except: pass
            return

    # ── Render options (windowed only) ───────────────────────────────────────
    if not headless:
        try:
            opt = vis.get_render_option()
            if opt is not None:
                opt.background_color = np.array([0.04, 0.04, 0.10])  # deep navy
                opt.point_size       = 3.0
                opt.show_coordinate_frame = True   # draws X/Y/Z axes at origin
        except Exception as e:
            print(f"[O3D ] render_option warning (non-fatal): {e}")

    # ── Geometry objects ─────────────────────────────────────────────────────
    pcd  = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3), dtype=np.float64))
    bbox = o3d.geometry.AxisAlignedBoundingBox()   # room-scale bounding box
    bbox.color = (0.3, 0.3, 0.3)

    if not headless:
        vis.add_geometry(pcd)
        vis.add_geometry(bbox)
    else:
        renderer.scene.add_geometry("pcd",  pcd,  mat)
        renderer.scene.set_background([0.04, 0.04, 0.10, 1.0])

    # ── Helpers ──────────────────────────────────────────────────────────────
    MAX_POINTS   = 50_000   # voxel-downsample target
    VOXEL_SIZE   = 0.02     # metres — increase if map is very large
    first_update = True
    last_stat_t  = time.time()

    def _height_colors(arr):
        """Map Y coordinate → blue(low) → green(mid) → red(high)."""
        y    = arr[:, 1]
        ymin, ymax = y.min(), y.max()
        span = max(ymax - ymin, 1e-6)
        t    = np.clip((y - ymin) / span, 0, 1)   # 0=floor, 1=ceiling
        # blue → cyan → green → yellow → red
        r = np.clip(2 * t - 0,   0, 1)
        g = np.clip(2 * t,        0, 1) * np.clip(2 - 2 * t, 0, 1)
        b = np.clip(1 - 2 * t,   0, 1)
        return np.column_stack([r, g, b])

    # ── Main render loop ─────────────────────────────────────────────────────
    while True:
        pts = None
        try:
            while True:
                item = queue.get_nowait()
                if item == "STOP":
                    if not headless: vis.destroy_window()
                    return
                pts = item
        except Exception:
            pass

        if pts is not None and len(pts) > 0:
            arr = np.asarray(pts, dtype=np.float64)

            # ── Voxel downsample if too many points ──────────────────────────
            if len(arr) > MAX_POINTS:
                tmp = o3d.geometry.PointCloud()
                tmp.points = o3d.utility.Vector3dVector(arr)
                tmp = tmp.voxel_down_sample(VOXEL_SIZE)
                arr = np.asarray(tmp.points)

            pcd.points = o3d.utility.Vector3dVector(arr)
            pcd.colors = o3d.utility.Vector3dVector(_height_colors(arr))

            # Update bounding box
            new_bbox = pcd.get_axis_aligned_bounding_box()
            new_bbox.color = (0.3, 0.3, 0.3)

            if not headless:
                vis.update_geometry(pcd)
                vis.remove_geometry(bbox, reset_bounding_box=False)
                bbox.min_bound = new_bbox.min_bound
                bbox.max_bound = new_bbox.max_bound
                vis.add_geometry(bbox, reset_bounding_box=False)
                if first_update:
                    vis.reset_view_point(True)
                    first_update = False
            else:
                renderer.scene.remove_geometry("pcd")
                renderer.scene.add_geometry("pcd", pcd, mat)
                if first_update:
                    bounds = pcd.get_axis_aligned_bounding_box()
                    renderer.setup_camera(60.0, bounds, bounds.get_center())
                    first_update = False

            # ── Print stats every 5 s ────────────────────────────────────────
            now = time.time()
            if now - last_stat_t >= 5.0:
                ext = new_bbox.get_extent()
                print(f"[O3D ] {len(arr)} pts  "
                      f"bbox {ext[0]:.1f}×{ext[1]:.1f}×{ext[2]:.1f} m")
                last_stat_t = now

        if not headless:
            if not vis.poll_events():
                print("[O3D ] Window closed by user")
                break
            vis.update_renderer()

        time.sleep(0.033)   # ~30 fps

    if not headless:
        vis.destroy_window()


class Open3DViewer:
    """
    Thin wrapper — spawns the child process on first update or on demand.
    """
    def __init__(self):
        self._proc  = None
        self._queue = None

    def _ensure_running(self):
        if self._proc is None or not self._proc.is_alive():
            self._queue = mp.Queue(maxsize=4)
            self._proc  = mp.Process(target=_open3d_process,
                                     args=(self._queue,), daemon=True)
            self._proc.start()
            print("[O3D ] Point-cloud viewer process started (pid "
                  f"{self._proc.pid})")

    def update(self, points):
        """Push a fresh point cloud (list or Nx3 array) to the viewer."""
        if not points:
            return
        self._ensure_running()
        try:
            # Non-blocking put — drop frame if queue is full
            self._queue.put_nowait(list(points))
        except Exception:
            pass

    def open(self):
        """Explicitly (re)open the viewer window."""
        self._ensure_running()

    def stop(self):
        if self._proc and self._proc.is_alive():
            try:
                self._queue.put_nowait("STOP")
                self._proc.join(timeout=2)
            except Exception:
                pass
            if self._proc.is_alive():
                self._proc.terminate()
        self._proc  = None
        self._queue = None

    def ready(self):
        return self._proc is not None and self._proc.is_alive()


# ══════════════════════════════════════════════════════
# NETWORK CONFIG
# ══════════════════════════════════════════════════════
RPI5_IP        = os.environ.get("CLEANBOT_RPI_IP", "192.168.0.108")   # ← RPi5 IP (or set CLEANBOT_RPI_IP)

CAM_PORT       = 5000
IMU_PORT       = 5001
DET_PORT       = 5002
POSE_BACK_PORT = 5003

SLAM_FRAME_PORT = 9997
SLAM_IMU_PORT   = 9998
SLAM_POSE_PORT  = 9999
SLAM_MAP_PORT   = 9996

# ══════════════════════════════════════════════════════
# SOCKETS
# ══════════════════════════════════════════════════════
def _udp_recv(port, timeout=0.5, bufsize=8*1024*1024):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # SO_REUSEADDR lets us restart the script immediately without
    # waiting for the OS to release the port (fixes "Address already in use")
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, bufsize)
    s.bind(("0.0.0.0", port))
    s.settimeout(timeout)
    return s

def _udp_send(bufsize=4*1024*1024):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, bufsize)
    return s

# Sockets are created inside setup_sockets() which is called only from
# the if __name__ == "__main__" block.  This prevents the spawned Open3D
# child process from re-executing socket.bind() calls and hitting
# "OSError: [Errno 98] Address already in use".
cam_sock = imu_sock = det_sock = slam_pose_sock = slam_map_sock = None
pose_back_sock = slam_frame_sock = slam_imu_sock = None

def setup_sockets():
    global cam_sock, imu_sock, det_sock, slam_pose_sock, slam_map_sock
    global pose_back_sock, slam_frame_sock, slam_imu_sock
    cam_sock       = _udp_recv(CAM_PORT,       timeout=0.5)
    imu_sock       = _udp_recv(IMU_PORT,       timeout=0.1)
    det_sock       = _udp_recv(DET_PORT,       timeout=0.5)
    slam_pose_sock = _udp_recv(SLAM_POSE_PORT, timeout=0.01)
    slam_map_sock  = _udp_recv(SLAM_MAP_PORT,  timeout=0.01)
    pose_back_sock  = _udp_send()
    slam_frame_sock = _udp_send(bufsize=8*1024*1024)
    slam_imu_sock   = _udp_send()
    print("="*60)
    print("  PC RECEIVER — ORB-SLAM3 + Open3D + A* + PID  [25fps]")
    print("="*60)
    print(f"  Camera   <- 0.0.0.0:{CAM_PORT}   (from RPi5)")
    print(f"  IMU      <- 0.0.0.0:{IMU_PORT}   (from RPi5)")
    print(f"  YOLO     <- 0.0.0.0:{DET_PORT}   (from RPi5)")
    print(f"  Pose     -> RPi5 {RPI5_IP}:{POSE_BACK_PORT}")
    print(f"  Frame    -> slam_receiver :9997  @ 25fps")
    print(f"  IMU fwd  -> slam_receiver :9998  @ 100Hz")
    print(f"  Pose     <- slam_receiver :9999")
    print(f"  Map      <- slam_receiver :9996")
    print("="*60)
    print("  Keys: G=set goal  O=obstacle  C=clear path")
    print("        R=reset  P=open 3D map  Q=quit")
print("="*60)

# ══════════════════════════════════════════════════════
# SHARED STATE
# ══════════════════════════════════════════════════════
latest_frame    = None
frame_lock      = threading.Lock()

latest_imu      = {"ax":0.,"ay":0.,"az":9.81,
                   "gx":0.,"gy":0.,"gz":0.,"ts":0.}
imu_lock        = threading.Lock()

detections      = []
detect_lock     = threading.Lock()

slam_position   = np.zeros(3, dtype=np.float32)
slam_state      = "WAITING"
slam_trajectory = []
slam_lock       = threading.Lock()

cloud_points    = []
cloud_lock      = threading.Lock()

running         = True
frames_received = 0

# ══════════════════════════════════════════════════════
# A* PLANNER
# ══════════════════════════════════════════════════════
class AStarPlanner:
    def __init__(self, grid_size=200, resolution=0.05):
        self.grid           = np.zeros((grid_size,grid_size), dtype=np.int8)
        self.grid_size      = grid_size
        self.resolution     = resolution
        self.origin         = np.array([grid_size//2, grid_size//2])
        self.dirt_locations = []

    def world_to_grid(self, x, y):
        col = int(x/self.resolution) + self.origin[0]
        row = int(y/self.resolution) + self.origin[1]
        return (max(0,min(self.grid_size-1,row)),
                max(0,min(self.grid_size-1,col)))

    def grid_to_world(self, row, col):
        return ((col-self.origin[0])*self.resolution,
                (row-self.origin[1])*self.resolution)

    def mark_obstacle(self, x, y, radius=3):
        row,col = self.world_to_grid(x,y)
        for dr in range(-radius,radius+1):
            for dc in range(-radius,radius+1):
                r=max(0,min(self.grid_size-1,row+dr))
                c=max(0,min(self.grid_size-1,col+dc))
                self.grid[r][c]=1

    def mark_dirt(self, x, y):
        self.dirt_locations.append((x,y))

    def find_path(self, start_world, goal_world):
        start=self.world_to_grid(*start_world)
        goal =self.world_to_grid(*goal_world)
        if self.grid[goal[0]][goal[1]]==1: return None
        open_set=[]; heapq.heappush(open_set,(0,start))
        came_from={}; g_score={start:0}
        while open_set:
            current=heapq.heappop(open_set)[1]
            if current==goal:
                path=[]
                while current in came_from:
                    wx,wy=self.grid_to_world(*current)
                    path.append((wx,wy)); current=came_from[current]
                path.reverse(); return path
            row,col=current
            for dr,dc in [(-1,0),(1,0),(0,-1),(0,1),
                           (-1,-1),(-1,1),(1,-1),(1,1)]:
                nr,nc=row+dr,col+dc
                if (0<=nr<self.grid_size and 0<=nc<self.grid_size
                        and self.grid[nr][nc]==0):
                    neighbor=(nr,nc)
                    cost=1.414 if (dr and dc) else 1.0
                    tg=g_score[current]+cost
                    if tg<g_score.get(neighbor,float('inf')):
                        came_from[neighbor]=current; g_score[neighbor]=tg
                        f=tg+abs(nr-goal[0])+abs(nc-goal[1])
                        heapq.heappush(open_set,(f,neighbor))
        return None

    def visualize_2d(self, current_pos, trajectory=None, path=None, goal=None):
        vis=np.ones((self.grid_size,self.grid_size,3),dtype=np.uint8)*240
        for i in range(0,self.grid_size,10):
            cv2.line(vis,(0,i),(self.grid_size,i),(210,210,210),1)
            cv2.line(vis,(i,0),(i,self.grid_size),(210,210,210),1)
        vis[self.grid==1]=[60,60,60]
        if trajectory and len(trajectory)>1:
            for i in range(1,len(trajectory)):
                r1,c1=self.world_to_grid(*trajectory[i-1])
                r2,c2=self.world_to_grid(*trajectory[i])
                cv2.line(vis,(c1,r1),(c2,r2),(100,180,100),1)
        for dx,dy in self.dirt_locations:
            dr,dc=self.world_to_grid(dx,dy)
            cv2.circle(vis,(dc,dr),5,(0,0,200),-1)
        if path:
            for wx,wy in path:
                r,c=self.world_to_grid(wx,wy)
                cv2.circle(vis,(c,r),2,(255,140,0),-1)
        if goal:
            gr,gc=self.world_to_grid(*goal)
            cv2.circle(vis,(gc,gr),7,(0,200,255),-1)
        cr,cc=self.world_to_grid(*current_pos)
        cv2.circle(vis,(cc,cr),7,(0,200,0),-1)
        cv2.circle(vis,(cc,cr),7,(255,255,255),1)
        oc=int(self.origin[0]); or_=int(self.origin[1])
        cv2.line(vis,(oc-5,or_),(oc+5,or_),(150,0,150),1)
        cv2.line(vis,(oc,or_-5),(oc,or_+5),(150,0,150),1)
        return vis

# ══════════════════════════════════════════════════════
# PID + DIFFERENTIAL DRIVE
# ══════════════════════════════════════════════════════
class PIDController:
    def __init__(self, kp, ki, kd, limit=5.0):
        self.kp=kp; self.ki=ki; self.kd=kd; self.limit=limit
        self._i=0.; self._prev=0.

    def compute(self, err, dt):
        self._i += err*dt
        d = (err-self._prev)/dt if dt>0 else 0.
        self._prev = err
        return float(np.clip(self.kp*err+self.ki*self._i+self.kd*d,
                             -self.limit, self.limit))
    def reset(self): self._i=0.; self._prev=0.

class RobotNavigator:
    def __init__(self):
        self.x=0.; self.y=0.; self.theta=0.
        self.L=0.20; self.r=0.05
        self.pid_h=PIDController(2.0,0.01,0.30)
        self.pid_s=PIDController(1.5,0.00,0.10)
        self.last_t=time.time()

    def step(self, goal):
        now=time.time()
        dt=float(np.clip(now-self.last_t,0.001,0.1))
        self.last_t=now
        if goal is None:
            return 0.,0.,self.x,self.y,self.theta
        dx=goal[0]-self.x; dy=goal[1]-self.y
        dist=np.sqrt(dx**2+dy**2)
        dh=np.arctan2(dy,dx)-self.theta
        dh=(dh+np.pi)%(2*np.pi)-np.pi
        turn =self.pid_h.compute(dh,dt)
        speed=self.pid_s.compute(dist,dt)*max(0.,1.-abs(dh)/np.pi)
        vl=float(np.clip(speed-turn,-3,3))
        vr=float(np.clip(speed+turn,-3,3))
        v=self.r*(vr+vl)/2.; w=self.r*(vr-vl)/self.L
        self.theta+=w*dt
        self.x+=v*np.cos(self.theta)*dt
        self.y+=v*np.sin(self.theta)*dt
        return vl,vr,self.x,self.y,self.theta

    def reset(self):
        self.x=0.; self.y=0.; self.theta=0.
        self.pid_h.reset(); self.pid_s.reset()

# ══════════════════════════════════════════════════════
# RECEIVE THREADS
# ══════════════════════════════════════════════════════
def recv_camera_thread():
    global frames_received, latest_frame
    print(f"[CAM ] Waiting on port {CAM_PORT}...")
    while running:
        try:
            data, addr = cam_sock.recvfrom(65535)
            if len(data) < 9: continue
            jpg   = np.frombuffer(data[8:], dtype=np.uint8)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            if frame is not None:
                with frame_lock:
                    latest_frame = frame
                frames_received += 1
                if frames_received == 1:
                    print(f"[CAM ] First frame from {addr[0]}!")
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[CAM ] Error: {e}")

def recv_imu_thread():
    sz = struct.calcsize('d6f')
    print(f"[IMU ] Waiting on port {IMU_PORT}...")
    while running:
        try:
            data,_ = imu_sock.recvfrom(64)
            if len(data) >= sz:
                ts,ax,ay,az,gx,gy,gz = struct.unpack('d6f', data[:sz])
                with imu_lock:
                    latest_imu.update({
                        "ax":ax,"ay":ay,"az":az,
                        "gx":gx,"gy":gy,"gz":gz,"ts":ts
                    })
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[IMU ] Error: {e}")

def recv_detections_thread():
    print(f"[DET ] Waiting on port {DET_PORT}...")
    while running:
        try:
            data,_ = det_sock.recvfrom(8192)
            dets = json.loads(data.decode())
            with detect_lock:
                detections.clear(); detections.extend(dets)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[DET ] Error: {e}")

def recv_slam_pose_thread():
    STATE_NAMES = {0:"TRACKING", 1:"RECENTLY_LOST",
                   2:"LOST",     3:"INITIALIZING"}
    print(f"[POSE] Waiting on port {SLAM_POSE_PORT}...")
    while running:
        try:
            data,_ = slam_pose_sock.recvfrom(256)
            parts  = data.decode().strip().split()
            if len(parts) >= 4:
                x=float(parts[0]); y=float(parts[1])
                z=float(parts[2]); s=int(parts[3])
                with slam_lock:
                    global slam_position, slam_state
                    slam_position = np.array([x,y,z], dtype=np.float32)
                    slam_state    = STATE_NAMES.get(s, "LOST")
                    if s == 0:   # only record when TRACKING
                        slam_trajectory.append((x, z))
                        if len(slam_trajectory) > 2000:
                            slam_trajectory.pop(0)
                # Forward pose back to RPi5
                msg = f"{x:.4f} {y:.4f} {z:.4f} {s}"
                pose_back_sock.sendto(msg.encode(),
                                      (RPI5_IP, POSE_BACK_PORT))
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[POSE] Error: {e}")

def recv_slam_map_thread():
    print(f"[MAP ] Waiting on port {SLAM_MAP_PORT}...")
    while running:
        try:
            data,_ = slam_map_sock.recvfrom(65535)
            n_bytes = len(data)
            if n_bytes < 12: continue
            n_floats = n_bytes // 4
            n_pts    = n_floats // 3
            if n_pts == 0: continue
            floats = struct.unpack(f'{n_pts*3}f', data[:n_pts*12])
            batch  = [[floats[i*3], floats[i*3+1], floats[i*3+2]]
                      for i in range(n_pts)]
            with cloud_lock:
                cloud_points.extend(batch)
                if len(cloud_points) > 60_000:
                    del cloud_points[:-60_000]
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[MAP ] Error: {e}")

def forward_frame_thread():
    """
    Forward camera frames to slam_receiver at 25fps.
    CRITICAL: use PC time.time() — same clock as IMU forwarder.
    """
    INTERVAL = 1.0 / 25.0      # ← 25 fps
    print(f"[FWD ] Frame forwarder → :9997 @ 25fps")
    while running:
        t0 = time.time()
        try:
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None
            if frame is not None:
                _, jpg = cv2.imencode('.jpg', frame,
                                      [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                ts     = time.time()          # PC clock — matches IMU
                header = struct.pack('d', ts)
                slam_frame_sock.sendto(header + jpg.tobytes(),
                                       ("127.0.0.1", SLAM_FRAME_PORT))
        except Exception as e:
            print(f"[FWD-CAM] Error: {e}")
        elapsed = time.time() - t0
        if elapsed < INTERVAL:
            time.sleep(INTERVAL - elapsed)

def forward_imu_thread():
    """
    Forward IMU to slam_receiver at 100Hz.
    ALWAYS use PC time.time() — never RPi5 clock.
    """
    INTERVAL = 1.0 / 100.0
    print(f"[FWD ] IMU forwarder → :9998 @ 100Hz")
    while running:
        t0 = time.time()
        try:
            with imu_lock:
                iv = dict(latest_imu)
            ts = time.time()
            imu_pkt = struct.pack('d6f',
                ts,
                iv['ax'], iv['ay'], iv['az'],
                iv['gx'], iv['gy'], iv['gz'])
            slam_imu_sock.sendto(imu_pkt, ("127.0.0.1", SLAM_IMU_PORT))
        except Exception as e:
            print(f"[FWD-IMU] Error: {e}")
        elapsed = time.time() - t0
        if elapsed < INTERVAL:
            time.sleep(INTERVAL - elapsed)

# ══════════════════════════════════════════════════════
# INIT
# ══════════════════════════════════════════════════════
# ── spawn is the correct multiprocessing mode here ──────────────────────────
# spawn = fresh Python interpreter for the child.
#   - No inherited X11/XCB connections  → no "xcb_request_check" crash
#   - No re-bound UDP sockets           → no "Address already in use"
#   - DISPLAY is passed via spawn's os.environ copy, so Open3D finds it
# fork would inherit the parent's XCB connection (opened by OpenCV) and
# pass it to the Open3D child, causing the assertion crash seen in the error.
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    # Sockets are created HERE so the spawned child never sees this code.
    setup_sockets()

    planner   = AStarPlanner(grid_size=200, resolution=0.05)
    navigator = RobotNavigator()
    map3d     = Open3DViewer()

    obstacle_positions = []
    dirt_queue         = []
    robot_state        = "EXPLORING"
    current_goal       = None
    path               = None

    # Start the map receiver thread (was missing from thread list before)
    recv_slam_map_th = threading.Thread(target=recv_slam_map_thread, daemon=True)
    recv_slam_map_th.start()

    for tgt in [recv_camera_thread, recv_imu_thread, recv_detections_thread,
                recv_slam_pose_thread,
                forward_frame_thread, forward_imu_thread]:
        threading.Thread(target=tgt, daemon=True).start()

    COLORS       = {"liquid_obstacle":(0,140,255), "solid_dirt":(0,0,220)}
    STATE_COLORS = {"TRACKING":(0,255,0), "RECENTLY_LOST":(0,200,255),
                    "LOST":(0,0,255),     "INITIALIZING":(0,220,220),
                    "WAITING":(180,180,180)}
    fps=0; fps_count=0; fps_t=time.time(); frame_count=0

    # How often (in frames) to push point cloud to the 3-D viewer
    O3D_PUSH_EVERY = 30   # ~every 1 s at 25fps

    # ══════════════════════════════════════════════════════
    # WAITING SCREEN
    # ══════════════════════════════════════════════════════
    def waiting_canvas():
        c = np.zeros((420,960,3), dtype=np.uint8); c[:]=(22,22,32)
        cv2.putText(c,"WAITING FOR RPi5...",
                    (250,120),cv2.FONT_HERSHEY_SIMPLEX,1.2,(0,200,255),2)
        cv2.putText(c,f"Camera  <- port {CAM_PORT}",
                    (340,175),cv2.FONT_HERSHEY_SIMPLEX,0.65,(150,150,150),1)
        cv2.putText(c,"Make sure rpi5_sender.py is running!",
                    (240,220),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,180,50),1)
        cv2.putText(c,"Make sure slam_receiver is running in Terminal 1!",
                    (180,265),cv2.FONT_HERSHEY_SIMPLEX,0.6,(100,255,100),1)
        cv2.putText(c,"Point camera at TEXTURED scene",
                    (270,310),cv2.FONT_HERSHEY_SIMPLEX,0.58,(200,200,100),1)
        cv2.putText(c,"P = open 3D map   Q = quit",
                    (340,395),cv2.FONT_HERSHEY_SIMPLEX,0.6,(180,180,180),1)
        return c

    # ══════════════════════════════════════════════════════
    # MAIN LOOP
    # ══════════════════════════════════════════════════════
    WIN = "PC - ORB-SLAM3 + A* [25fps]"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 960, 420)
    print("[CV2 ] Window created — stream will appear when RPi5 connects")
    try:
        while True:
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None

            if frame is None:
                cv2.imshow(WIN, waiting_canvas())
                if cv2.waitKey(30) & 0xFF == ord('q'): break
                continue

            with slam_lock:
                pos  = slam_position.copy()
                stat = slam_state
                traj = list(slam_trajectory)
            with imu_lock:
                iv = dict(latest_imu)
            with detect_lock:
                dets = list(detections)

            curr_pos = (float(pos[0]), float(pos[2]))
            _,_,sx,sy,sth = navigator.step(current_goal)

            # ── Robot brain (PC side — visualization only) ─
            for det in dets:
                lbl = det["label"]; cf = det["conf"]
                if lbl == "solid_dirt" and cf > 0.6:
                    dp = (pos[0]+0.5, pos[2])
                    if dp not in dirt_queue:
                        dirt_queue.append(dp); planner.mark_dirt(*dp)
                elif lbl == "liquid_obstacle" and cf > 0.5:
                    op = (pos[0]+0.3, pos[2])
                    planner.mark_obstacle(*op)
                    if op not in obstacle_positions:
                        obstacle_positions.append(op)

            if robot_state == "EXPLORING" and dirt_queue and path is None:
                path = planner.find_path(curr_pos, dirt_queue[0])
                if path:
                    current_goal = dirt_queue[0]
                    robot_state  = "NAVIGATING_TO_DIRT"

            if current_goal and path:
                dtg = np.hypot(pos[0]-current_goal[0], pos[2]-current_goal[1])
                if dtg < 0.1:
                    if current_goal in dirt_queue: dirt_queue.remove(current_goal)
                    current_goal = None; path = None; robot_state = "EXPLORING"

            if current_goal and frame_count % 30 == 0:
                path = planner.find_path(curr_pos, current_goal)

            # ── Push cloud to Open3D viewer periodically ──
            if frame_count % O3D_PUSH_EVERY == 0:
                with cloud_lock:
                    pts_snapshot = list(cloud_points)
                if pts_snapshot:
                    map3d.update(pts_snapshot)

            # ── Build display ─────────────────────────────
            disp = cv2.resize(frame, (640,360))

            for det in dets:
                x1,y1,x2,y2 = det["box"]
                y1=int(y1*360/480); y2=int(y2*360/480)
                col = COLORS.get(det["label"],(255,255,255))
                cv2.rectangle(disp,(x1,y1),(x2,y2),col,2)
                lbl = det["label"].upper()+f" {int(det['conf']*100)}%"
                (tw,th),_ = cv2.getTextSize(lbl,cv2.FONT_HERSHEY_SIMPLEX,0.5,1)
                cv2.rectangle(disp,(x1,y1-th-8),(x1+tw+6,y1),col,-1)
                cv2.putText(disp,lbl,(x1+3,y1-4),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)

            sc = {"EXPLORING":(20,90,20),"NAVIGATING_TO_DIRT":(90,20,20),
                  "AVOIDING":(20,80,120)}.get(robot_state,(40,40,40))
            ov = disp.copy(); cv2.rectangle(ov,(0,0),(640,58),sc,-1)
            cv2.addWeighted(ov,0.8,disp,0.2,0,disp)
            scol = STATE_COLORS.get(stat,(200,200,200))
            cv2.putText(disp,f"ROBOT: {robot_state}",
                        (8,20),cv2.FONT_HERSHEY_SIMPLEX,0.65,(255,255,255),2)
            cv2.putText(disp,f"SLAM: {stat}",
                        (8,42),cv2.FONT_HERSHEY_SIMPLEX,0.52,scol,1)
            cv2.putText(disp,
                        f"X:{pos[0]:.3f}  Y:{pos[1]:.3f}  Z:{pos[2]:.3f}"
                        f"  Frames:{frames_received}",
                        (8,72),cv2.FONT_HERSHEY_SIMPLEX,0.42,(100,255,100),1)
            cv2.putText(disp,
                        f"Accel {iv['ax']:.2f} {iv['ay']:.2f} {iv['az']:.2f}",
                        (5,305),cv2.FONT_HERSHEY_SIMPLEX,0.38,(0,255,150),1)
            cv2.putText(disp,
                        f"Gyro  {iv['gx']:.3f} {iv['gy']:.3f} {iv['gz']:.3f} rad/s",
                        (5,320),cv2.FONT_HERSHEY_SIMPLEX,0.38,(255,180,50),1)

            # 3D map status badge
            o3d_color = (0,255,150) if map3d.ready() else (100,100,100)
            with cloud_lock:
                n_pts = len(cloud_points)
            cv2.putText(disp,
                        f"3D:{n_pts}pts {'[LIVE]' if map3d.ready() else '[P=open]'}",
                        (5,340),cv2.FONT_HERSHEY_SIMPLEX,0.38,o3d_color,1)

            fps_count+=1; now=time.time()
            if now-fps_t>=1.0:
                fps=fps_count/(now-fps_t); fps_count=0; fps_t=now
            cv2.putText(disp,f"FPS:{int(fps)}",
                        (590,354),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,100),1)

            canvas = np.zeros((420,960,3), dtype=np.uint8)
            canvas[0:360, 0:640] = disp

            mv = planner.visualize_2d(curr_pos, traj, path, current_goal)
            mv = cv2.resize(mv,(300,300))
            canvas[30:330, 648:948] = mv
            cv2.putText(canvas,"2D MAP",
                        (770,22),cv2.FONT_HERSHEY_SIMPLEX,0.5,(180,180,180),1)

            canvas[360:420, 0:960] = (12,12,22)
            cv2.putText(canvas,
                        "G=set goal   O=mark obstacle   C=clear path   "
                        "R=full reset   P=3D map   Q=quit",
                        (10,390),cv2.FONT_HERSHEY_SIMPLEX,0.45,(160,200,255),1)

            cv2.imshow(WIN, canvas)
            frame_count += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                # Open (or re-open) the 3D point-cloud viewer
                map3d.open()
                with cloud_lock:
                    pts_snapshot = list(cloud_points)
                if pts_snapshot:
                    map3d.update(pts_snapshot)
                else:
                    print("[O3D ] No map points yet — keep the SLAM running")
            elif key == ord('g'):
                mg = (pos[0]+1.0, pos[2])
                path = planner.find_path(curr_pos, mg)
                current_goal = mg; robot_state = "NAVIGATING_TO_DIRT"
            elif key == ord('o'):
                planner.mark_obstacle(*curr_pos)
                obstacle_positions.append(curr_pos)
                if current_goal: path = planner.find_path(curr_pos, current_goal)
            elif key == ord('c'):
                path = None; current_goal = None; robot_state = "EXPLORING"
            elif key == ord('r'):
                planner.grid[:] = 0; planner.dirt_locations.clear()
                dirt_queue.clear(); obstacle_positions.clear()
                path = None; current_goal = None; robot_state = "EXPLORING"
                navigator.reset()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        running = False
        time.sleep(0.3)
        for s in [cam_sock, imu_sock, det_sock, pose_back_sock,
                  slam_frame_sock, slam_imu_sock, slam_pose_sock, slam_map_sock]:
            try: s.close()
            except: pass
        map3d.stop()
        cv2.destroyAllWindows()
        print("Done!")
