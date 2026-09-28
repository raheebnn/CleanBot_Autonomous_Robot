"""
rpi5_motor_brain_slam.py  —  BEST VERSION
==========================================
Terminal 2 on RPi5. Run AFTER rpi5_sender.py.

Receives from rpi5_sender.py (localhost):
  SLAM pose  ← localhost :6100  ("x y z state")
  YOLO dets  ← localhost :6101  (JSON)

Controls:
  BTS7960 Module 1 — LEFT  motor (GPIO 12,13,24,25)
  BTS7960 Module 2 — RIGHT motor (GPIO 18,19,20,21)

STATE MACHINE (your requirements):
  INITIALIZING → SPIN slowly in place to help SLAM find features
                 If YOLO sees solid_dirt while spinning → queue it
                 but DON'T drive to it yet (no SLAM pose)

  TRACKING     → Drive forward exploring
                 If YOLO sees solid_dirt → go to EXACT spot immediately
                 Arrive at dirt → WAIT 1 second → continue exploring
                 If liquid_obstacle ahead → turn right to avoid

  RECENTLY_LOST/LOST → SPIN slowly to recover tracking
                        (same as INITIALIZING)

SLAM state codes from slam_receiver.cc:
  0=TRACKING  1=RECENTLY_LOST  2=LOST  3=INITIALIZING
"""

import time, socket, json, math, heapq, threading, signal, sys
import numpy as np

# ══════════════════════════════════════════════════════
# LGPIO
# ══════════════════════════════════════════════════════
try:
    import lgpio
    LGPIO_OK = True
except ImportError:
    LGPIO_OK = False
    print("[WARN] lgpio missing — dry-run mode (motors won't move)")

# ══════════════════════════════════════════════════════
# PORTS
# ══════════════════════════════════════════════════════
LOCAL_POSE_PORT = 6100
LOCAL_DET_PORT  = 6101

# ══════════════════════════════════════════════════════
# BTS7960 MOTOR DRIVER — gpiochip4 = RPi5
# ══════════════════════════════════════════════════════
L_RPWM, L_LPWM, L_R_EN, L_L_EN = 12, 13, 24, 25
R_RPWM, R_LPWM, R_R_EN, R_L_EN = 18, 19, 20, 21
PWM_FREQ = 1000

_chip = None

def motors_init():
    global _chip
    if not LGPIO_OK: return
    _chip = lgpio.gpiochip_open(4)
    for pin in [L_RPWM, L_LPWM, L_R_EN, L_L_EN,
                R_RPWM, R_LPWM, R_R_EN, R_L_EN]:
        lgpio.gpio_claim_output(_chip, pin, 0)
    for pin in [L_R_EN, L_L_EN, R_R_EN, R_L_EN]:
        lgpio.gpio_write(_chip, pin, 1)
    for pin in [L_RPWM, L_LPWM, R_RPWM, R_LPWM]:
        lgpio.tx_pwm(_chip, pin, PWM_FREQ, 0)
    print("  [OK] BTS7960 ready (gpiochip4)")

def _pwm(rpwm, lpwm, duty):
    if not LGPIO_OK or _chip is None: return
    duty = max(-100.0, min(100.0, float(duty)))
    if duty > 0:
        lgpio.tx_pwm(_chip, rpwm, PWM_FREQ, duty)
        lgpio.tx_pwm(_chip, lpwm, PWM_FREQ, 0.0)
    elif duty < 0:
        lgpio.tx_pwm(_chip, rpwm, PWM_FREQ, 0.0)
        lgpio.tx_pwm(_chip, lpwm, PWM_FREQ, -duty)
    else:
        lgpio.tx_pwm(_chip, rpwm, PWM_FREQ, 0.0)
        lgpio.tx_pwm(_chip, lpwm, PWM_FREQ, 0.0)

def motors_set(left_duty, right_duty):
    _pwm(L_RPWM, L_LPWM, left_duty)
    _pwm(R_RPWM, R_LPWM, right_duty)

def motors_stop():
    motors_set(0, 0)

def motors_shutdown():
    if not LGPIO_OK or _chip is None: return
    motors_stop()
    for pin in [L_R_EN, L_L_EN, R_R_EN, R_L_EN]:
        lgpio.gpio_write(_chip, pin, 0)
    lgpio.gpiochip_close(_chip)
    print("[MTR ] GPIO released")

# ══════════════════════════════════════════════════════
# DIFFERENTIAL DRIVE
# ══════════════════════════════════════════════════════
WHEEL_BASE = 0.20   # m
MAX_SPEED  = 0.25   # m/s
MAX_DUTY   = 60.0   # %

def diff_drive(linear_mps, angular_rads):
    half_w = WHEEL_BASE / 2.0
    vl = linear_mps - angular_rads * half_w
    vr = linear_mps + angular_rads * half_w
    max_v = max(abs(vl), abs(vr), 1e-6)
    if max_v > MAX_SPEED:
        vl *= MAX_SPEED / max_v
        vr *= MAX_SPEED / max_v
    return (vl / MAX_SPEED) * MAX_DUTY, (vr / MAX_SPEED) * MAX_DUTY

# ══════════════════════════════════════════════════════
# PID
# ══════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, limit=5.0):
        self.kp=kp; self.ki=ki; self.kd=kd; self.limit=limit
        self._i=0.0; self._prev=0.0

    def compute(self, err, dt):
        self._i   += err * dt
        d          = (err - self._prev) / dt if dt > 0 else 0.0
        self._prev = err
        return float(max(-self.limit, min(self.limit,
               self.kp*err + self.ki*self._i + self.kd*d)))

    def reset(self): self._i=0.0; self._prev=0.0

# ══════════════════════════════════════════════════════
# A* PLANNER
# ══════════════════════════════════════════════════════
class AStarPlanner:
    def __init__(self, grid_size=200, resolution=0.05):
        self.grid      = np.zeros((grid_size, grid_size), dtype=np.int8)
        self.grid_size = grid_size
        self.res       = resolution
        self.origin    = np.array([grid_size//2, grid_size//2])

    def _w2g(self, x, y):
        col = int(x/self.res) + self.origin[0]
        row = int(y/self.res) + self.origin[1]
        return (max(0,min(self.grid_size-1,row)),
                max(0,min(self.grid_size-1,col)))

    def _g2w(self, row, col):
        return ((col-self.origin[0])*self.res,
                (row-self.origin[1])*self.res)

    def mark_obstacle(self, x, y, radius=3):
        row,col = self._w2g(x,y)
        for dr in range(-radius,radius+1):
            for dc in range(-radius,radius+1):
                r=max(0,min(self.grid_size-1,row+dr))
                c=max(0,min(self.grid_size-1,col+dc))
                self.grid[r][c]=1

    def find_path(self, start_w, goal_w):
        start=self._w2g(*start_w); goal=self._w2g(*goal_w)
        if self.grid[goal[0]][goal[1]]==1: return None
        open_set=[]; heapq.heappush(open_set,(0,start))
        came_from={}; g_score={start:0}
        while open_set:
            current=heapq.heappop(open_set)[1]
            if current==goal:
                path=[]
                while current in came_from:
                    wx,wy=self._g2w(*current)
                    path.append((wx,wy))
                    current=came_from[current]
                path.reverse(); return path
            row,col=current
            for dr,dc in [(-1,0),(1,0),(0,-1),(0,1),
                           (-1,-1),(-1,1),(1,-1),(1,1)]:
                nr,nc=row+dr,col+dc
                if (0<=nr<self.grid_size and
                    0<=nc<self.grid_size and
                    self.grid[nr][nc]==0):
                    neighbor=(nr,nc)
                    cost=1.414 if (dr and dc) else 1.0
                    tg=g_score[current]+cost
                    if tg<g_score.get(neighbor,float('inf')):
                        came_from[neighbor]=current
                        g_score[neighbor]=tg
                        f=tg+abs(nr-goal[0])+abs(nc-goal[1])
                        heapq.heappush(open_set,(f,neighbor))
        return None

# ══════════════════════════════════════════════════════
# ROBOT BRAIN
# ══════════════════════════════════════════════════════
GOAL_REACH_DIST = 0.12    # m — how close = "arrived at dirt"
AVOID_DURATION  = 2.0     # s — how long to turn away from liquid
SPIN_DUTY       = 30.0    # % — spin speed during INITIALIZING/LOST
DIRT_WAIT_SECS  = 1.0     # s — wait at dirt before continuing
LOOK_AHEAD      = 2       # waypoints ahead to target

# ══════════════════════════════════════════════════════
# SMALL DIRT MODE
# ══════════════════════════════════════════════════════
# True  → only target SMALL dirt boxes (distinct small patches).
#          Big blobs (robot already on top, or large area) are ignored.
#          Among all small detections, goes to the CLOSEST one first.
# False → original behaviour (largest box = closest, go there)
SMALL_DIRT_MODE    = True
SMALL_BOX_MAX_AREA = 12000   # px² — boxes larger than this are "big", skipped
SMALL_BOX_MIN_AREA = 400     # px² — ignore tiny noise detections

# ══════════════════════════════════════════════════════
# ZIGZAG SWEEP EXPLORER
# ══════════════════════════════════════════════════════
SWEEP_LEG_LEN  = 1.5    # m  — length of each forward leg
SWEEP_WIDTH    = 0.40   # m  — side step between legs
SWEEP_LEGS     = 8      # number of legs before resetting sweep origin
SWEEP_SPEED    = 0.15   # m/s — speed while sweeping (slower = better SLAM features)


class ZigzagExplorer:
    """
    Boustrophedon (lawnmower) sweep path generator.

    Generates waypoints in a zigzag pattern anchored to the robot's
    position when the sweep starts. Each leg alternates direction so
    the robot covers a rectangular area without retracing its path.
    After SWEEP_LEGS legs it resets from current position to keep
    moving into unexplored territory.

    Leg layout (top view):
      origin ──────────────────► leg 0 end
             ◄────────────────── leg 1 end
      ──────────────────────────► leg 2 end  ...
    """

    def __init__(self):
        self._wps = []   # list of (x, z) waypoints for current sweep
        self._idx = 0    # index of current target waypoint

    def reset(self, ox, oz, heading):
        """
        Build SWEEP_LEGS waypoints from (ox, oz) aligned to heading.
        """
        fx =  math.cos(heading)   # forward unit vector
        fz =  math.sin(heading)
        rx =  fz                  # rightward unit vector (90° CW)
        rz = -fx

        self._wps = []
        for leg in range(SWEEP_LEGS):
            side    = leg * SWEEP_WIDTH         # side offset for this leg
            fwd_end = SWEEP_LEG_LEN if (leg % 2 == 0) else 0.0
            wx = ox + fwd_end * fx + side * rx
            wz = oz + fwd_end * fz + side * rz
            self._wps.append((round(wx, 3), round(wz, 3)))

        self._idx = 0
        print(f"[SWEEP] New sweep: {len(self._wps)} waypoints  "
              f"origin=({ox:.2f},{oz:.2f})  "
              f"heading={math.degrees(heading):.0f}°")

    def current_waypoint(self):
        if self._idx < len(self._wps):
            return self._wps[self._idx]
        return None

    def advance(self):
        self._idx += 1

    def done(self):
        return self._idx >= len(self._wps)

    def remaining(self):
        return max(0, len(self._wps) - self._idx)


class RobotBrain:
    def __init__(self):
        self.planner  = AStarPlanner()
        self.pid_head = PID(kp=2.0, ki=0.01, kd=0.30, limit=2.0)
        self.pid_spd  = PID(kp=1.5, ki=0.00, kd=0.10, limit=MAX_SPEED)

        self.rx    = 0.0
        self.rz    = 0.0
        self.theta = 0.0

        self.state        = "SPINNING"   # start spinning immediately
        self.dirt_queue   = []           # list of (x,z) world positions
        self.path         = []
        self.avoid_until  = 0.0
        self.wait_until   = 0.0          # waiting at dirt spot
        self._last_t      = time.time()
        self._spin_dir    = 1.0          # +1=left, -1=right
        self._spin_switch = time.time()  # alternates spin direction

        # Immediate dirt target — when YOLO sees dirt while TRACKING,
        # drop everything and go there right away
        self._immediate_target = None

        # Zigzag sweep explorer
        self._sweep        = ZigzagExplorer()
        self._sweep_active = False   # True once SLAM is tracking and sweep started
        self._sweep_path   = []      # A* path to current sweep waypoint

    def update_pose(self, x, z):
        self.rx = x
        self.rz = z

    def _dist(self, tx, tz):
        return math.sqrt((tx-self.rx)**2 + (tz-self.rz)**2)

    def _heading_to(self, tx, tz):
        return math.atan2(tz-self.rz, tx-self.rx)

    def _liquid_ahead(self, dets):
        for d in dets:
            if d["label"]=="liquid_obstacle" and d["conf"]>0.5:
                x1,y1,x2,y2 = d["box"]
                cx   = (x1+x2)/2.0
                area = (x2-x1)*(y2-y1)
                if 180 < cx < 460 and area > 8000:
                    return True
        return False

    def _best_dirt_detection(self, dets):
        """
        SMALL_DIRT_MODE = True  (default):
          Filter to only SMALL dirt boxes (distinct patches, not big blobs).
          Among those, return the CLOSEST one (largest box in the filtered set).
          Skips boxes larger than SMALL_BOX_MAX_AREA (robot already on it / big blob).
          Skips boxes smaller than SMALL_BOX_MIN_AREA (noise).

        SMALL_DIRT_MODE = False  (original):
          Return detection with largest area (closest dirt).
        """
        candidates = []
        for d in dets:
            if d["label"] != "solid_dirt" or d["conf"] <= 0.55:
                continue
            x1, y1, x2, y2 = d["box"]
            area = (x2 - x1) * (y2 - y1)
            if SMALL_DIRT_MODE:
                if SMALL_BOX_MIN_AREA < area < SMALL_BOX_MAX_AREA:
                    candidates.append((area, d))
            else:
                candidates.append((area, d))

        if not candidates:
            return None

        # Closest = largest area in filtered set
        candidates.sort(key=lambda t: t[0], reverse=True)
        return candidates[0][1]

    def _all_small_dirt(self, dets):
        """
        Return ALL small dirt detections sorted closest first.
        Used to queue multiple small patches to visit in order.
        """
        result = []
        for d in dets:
            if d["label"] != "solid_dirt" or d["conf"] <= 0.55:
                continue
            x1, y1, x2, y2 = d["box"]
            area = (x2 - x1) * (y2 - y1)
            if SMALL_DIRT_MODE:
                if not (SMALL_BOX_MIN_AREA < area < SMALL_BOX_MAX_AREA):
                    continue
            result.append((area, d))
        result.sort(key=lambda t: t[0], reverse=True)
        return [d for _, d in result]

    def _det_to_world(self, det):
        """
        Convert a YOLO bounding box to an estimated world (x,z) position.
        Uses box center X to estimate lateral offset and box size to estimate distance.
        Box center near image center = straight ahead.
        Distance estimated from box height (larger box = closer).
        """
        x1,y1,x2,y2 = det["box"]
        cx = (x1+x2)/2.0   # 0..640
        cy = (y1+y2)/2.0
        bh = y2 - y1        # box height in pixels

        # Distance estimate: calibrated roughly for 640x480 @ 15deg tilt
        # box_height 200px ≈ 0.3m away,  50px ≈ 1.2m away
        dist = max(0.2, min(2.0, 60.0 / max(bh, 10)))

        # Lateral offset: cx=320 = straight, each 100px ≈ 0.15m lateral at 1m
        lateral = (cx - 320.0) / 320.0 * dist * 0.5

        # Project into world frame using current heading
        wx = self.rx + dist   * math.cos(self.theta) \
                     - lateral * math.sin(self.theta)
        wz = self.rz + dist   * math.sin(self.theta) \
                     + lateral * math.cos(self.theta)
        return (round(wx, 2), round(wz, 2))

    def step(self, slam_state, dets):
        """
        Call at 20 Hz.
        slam_state: 0=TRACKING  1=RECENTLY_LOST  2=LOST  3=INITIALIZING
        Returns (left_duty%, right_duty%)
        """
        now = time.time()
        dt  = max(0.001, min(0.1, now - self._last_t))
        self._last_t = now

        # ═══════════════════════════════════════════════
        # SLAM NOT TRACKING → SPIN to find features
        # State: INITIALIZING (3), RECENTLY_LOST (1), LOST (2)
        # ═══════════════════════════════════════════════
        if slam_state != 0:
            if self.state != "SPINNING":
                print(f"[BRAIN] → SPINNING  (SLAM state={slam_state}  "
                      f"{'INITIALIZING' if slam_state==3 else 'LOST'})")
                self.state = "SPINNING"
                self.path  = []
                self._immediate_target = None
                self.pid_head.reset()
                self.pid_spd.reset()

            # While spinning: if YOLO sees dirt, queue the position
            # We can't compute world position without SLAM pose,
            # so just record that dirt exists — will navigate when tracking
            best_det = self._best_dirt_detection(dets)
            if best_det is not None:
                # Store detection without world coords — will compute on tracking
                if len(self.dirt_queue) == 0:
                    print("[BRAIN] Dirt seen while SPINNING — will navigate when TRACKING")

            # Alternate spin direction every 3 seconds so robot doesn't
            # spin forever in one direction (helps SLAM see new features)
            if now - self._spin_switch > 3.0:
                self._spin_dir    *= -1.0
                self._spin_switch  = now

            # Spin in place: one wheel forward, one backward
            l = -SPIN_DUTY * self._spin_dir
            r =  SPIN_DUTY * self._spin_dir
            motors_set(l, r)
            return l, r

        # ═══════════════════════════════════════════════
        # SLAM IS TRACKING (state=0)
        # ═══════════════════════════════════════════════

        if self.state == "SPINNING":
            print("[BRAIN] → EXPLORING  (SLAM now TRACKING)")
            self.state        = "EXPLORING"
            self._sweep_active = False    # fresh sweep from current pos

        # ── Register liquid obstacles into map ─────────
        for d in dets:
            if d["label"]=="liquid_obstacle" and d["conf"]>0.5:
                ox = self.rx + 0.3 * math.cos(self.theta)
                oz = self.rz + 0.3 * math.sin(self.theta)
                self.planner.mark_obstacle(ox, oz)

        # ── DIRT RESPONSE ──────────────────────────────
        # SMALL_DIRT_MODE=True : only small boxes qualify,
        #   queue ALL visible small patches, navigate to closest first via A*.
        # SMALL_DIRT_MODE=False: original — go to largest (closest) box.
        best_det = self._best_dirt_detection(dets)
        if best_det is not None and self.state != "WAITING_AT_DIRT":
            world_pos = self._det_to_world(best_det)

            # Queue additional small patches seen right now (if SMALL_DIRT_MODE)
            if SMALL_DIRT_MODE:
                all_dets = self._all_small_dirt(dets)
                for extra in all_dets[1:]:   # skip index 0 = the one we're going to
                    wpos = self._det_to_world(extra)
                    # Add to queue only if not already close to an existing entry
                    already = any(
                        math.sqrt((wpos[0]-q[0])**2 + (wpos[1]-q[1])**2) < 0.30
                        for q in self.dirt_queue
                    )
                    if not already:
                        self.dirt_queue.append(wpos)

            # Only re-plan if this is a genuinely new target (>0.3m away)
            update = True
            if self._immediate_target is not None:
                dx = world_pos[0] - self._immediate_target[0]
                dz = world_pos[1] - self._immediate_target[1]
                if math.sqrt(dx**2 + dz**2) < 0.30:
                    update = False   # same dirt, don't re-plan

            if update:
                self._immediate_target = world_pos
                new_path = self.planner.find_path(
                    (self.rx, self.rz), world_pos)
                if new_path:
                    self.path  = new_path
                    self.state = "GOING_TO_DIRT"
                    mode_tag   = "small" if SMALL_DIRT_MODE else "closest"
                    print(f"[BRAIN] → GOING_TO_DIRT ({mode_tag})  "
                          f"target=({world_pos[0]:.2f},{world_pos[1]:.2f})  "
                          f"conf={best_det['conf']:.2f}  "
                          f"path={len(new_path)} wpts  "
                          f"queued={len(self.dirt_queue)}")
                else:
                    # No A* path — drive straight toward it
                    self.path  = [world_pos]
                    self.state = "GOING_TO_DIRT"
                    print(f"[BRAIN] → GOING_TO_DIRT (direct)  "
                          f"target=({world_pos[0]:.2f},{world_pos[1]:.2f})")

        # ── LIQUID OBSTACLE — turn right ───────────────
        if self._liquid_ahead(dets):
            if self.state != "AVOIDING":
                print("[BRAIN] → AVOIDING liquid obstacle")
            self.avoid_until       = now + AVOID_DURATION
            self.state             = "AVOIDING"
            self.path              = []
            self._immediate_target = None

        if self.state == "AVOIDING":
            if now < self.avoid_until:
                l, r = diff_drive(0.05, -1.5)
                motors_set(l, r)
                return l, r
            else:
                print("[BRAIN] → EXPLORING (avoid done)")
                self.state       = "EXPLORING"
                self._sweep_path = []   # rebuild sweep path from new position

        # ── WAITING AT DIRT ────────────────────────────
        if self.state == "WAITING_AT_DIRT":
            motors_stop()
            if now >= self.wait_until:
                self._immediate_target = None
                if self.dirt_queue:
                    # Go to the next queued small dirt patch
                    next_pos = self.dirt_queue.pop(0)
                    new_path = self.planner.find_path(
                        (self.rx, self.rz), next_pos)
                    if new_path:
                        self.path  = new_path
                        self.state = "GOING_TO_DIRT"
                        self._immediate_target = next_pos
                        print(f"[BRAIN] → GOING_TO_DIRT (queued next)  "
                              f"target=({next_pos[0]:.2f},{next_pos[1]:.2f})  "
                              f"remaining={len(self.dirt_queue)}")
                    else:
                        self.path  = [next_pos]
                        self.state = "GOING_TO_DIRT"
                        self._immediate_target = next_pos
                        print(f"[BRAIN] → GOING_TO_DIRT (queued direct)  "
                              f"target=({next_pos[0]:.2f},{next_pos[1]:.2f})")
                else:
                    print("[BRAIN] → EXPLORING (all dirt done)")
                    self.state = "EXPLORING"
            return 0, 0

        # ── GOING TO DIRT or NAVIGATING ───────────────
        if self.state in ("GOING_TO_DIRT", "NAVIGATING") and self.path:
            # Remove reached waypoints
            while self.path and self._dist(*self.path[0]) < GOAL_REACH_DIST:
                self.path.pop(0)

            if not self.path:
                # Arrived at dirt!
                print(f"[BRAIN] → WAITING_AT_DIRT  ({DIRT_WAIT_SECS}s)")
                motors_stop()
                self.state      = "WAITING_AT_DIRT"
                self.wait_until = now + DIRT_WAIT_SECS
                return 0, 0

            # Navigate toward look-ahead waypoint
            wp          = self.path[min(LOOK_AHEAD, len(self.path)-1)]
            target_h    = self._heading_to(*wp)
            heading_err = (target_h - self.theta + math.pi) % (2*math.pi) - math.pi
            dist        = self._dist(*wp)

            angular = self.pid_head.compute(heading_err, dt)
            linear  = self.pid_spd.compute(dist, dt)
            linear *= max(0.0, 1.0 - abs(heading_err)/math.pi)
            linear  = max(0.0, min(MAX_SPEED, linear))

            self.theta = (self.theta + angular*dt + math.pi) % (2*math.pi) - math.pi

            l, r = diff_drive(linear, angular)
            motors_set(l, r)
            return l, r

        # ── EXPLORING — ZIGZAG SWEEP ──────────────────
        # Boustrophedon (lawnmower) pattern.
        # Generates A* waypoints in a back-and-forth grid so the robot
        # covers the whole area instead of driving straight forever.
        if self.state != "EXPLORING":
            print("[BRAIN] → EXPLORING (zigzag sweep)")
            self.state = "EXPLORING"

        # Start or reset sweep when needed
        if not self._sweep_active or self._sweep.done():
            self._sweep.reset(self.rx, self.rz, self.theta)
            self._sweep_active = True
            self._sweep_path   = []

        # Get current sweep target waypoint
        wp = self._sweep.current_waypoint()
        if wp is None:
            # Sweep done — reset immediately from here
            self._sweep.reset(self.rx, self.rz, self.theta)
            wp = self._sweep.current_waypoint()

        # Build / rebuild A* path to the sweep waypoint
        if not self._sweep_path:
            new_path = self.planner.find_path((self.rx, self.rz), wp)
            if new_path:
                self._sweep_path = new_path
                print(f"[SWEEP] Leg {self._sweep._idx+1}/{SWEEP_LEGS}  "
                      f"→ ({wp[0]:.2f},{wp[1]:.2f})  "
                      f"path={len(new_path)} wpts  "
                      f"remaining={self._sweep.remaining()}")
            else:
                # A* failed (obstacle?) — skip this waypoint and move on
                print(f"[SWEEP] No path to leg {self._sweep._idx+1} — skipping")
                self._sweep.advance()
                l, r = diff_drive(SWEEP_SPEED, 0.0)
                motors_set(l, r)
                return l, r

        # Remove reached waypoints from sweep A* path
        while self._sweep_path and self._dist(*self._sweep_path[0]) < GOAL_REACH_DIST:
            self._sweep_path.pop(0)

        if not self._sweep_path:
            # Arrived at sweep waypoint — advance to next leg
            print(f"[SWEEP] Reached leg {self._sweep._idx+1}/{SWEEP_LEGS}")
            self._sweep.advance()
            l, r = diff_drive(0.0, 0.0)
            motors_stop()
            return 0, 0

        # Navigate along sweep path with PID
        target_wp   = self._sweep_path[min(LOOK_AHEAD, len(self._sweep_path)-1)]
        target_h    = self._heading_to(*target_wp)
        heading_err = (target_h - self.theta + math.pi) % (2*math.pi) - math.pi
        dist        = self._dist(*target_wp)

        angular = self.pid_head.compute(heading_err, dt)
        linear  = self.pid_spd.compute(dist, dt)
        linear *= max(0.0, 1.0 - abs(heading_err) / math.pi)
        linear  = max(0.0, min(SWEEP_SPEED, linear))   # cap at sweep speed

        self.theta = (self.theta + angular * dt + math.pi) % (2*math.pi) - math.pi

        l, r = diff_drive(linear, angular)
        motors_set(l, r)
        return l, r


# ══════════════════════════════════════════════════════
# UDP SOCKETS
# ══════════════════════════════════════════════════════
pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pose_sock.bind(("127.0.0.1", LOCAL_POSE_PORT))
pose_sock.settimeout(0.1)

det_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
det_sock.bind(("127.0.0.1", LOCAL_DET_PORT))
det_sock.settimeout(0.05)

# ══════════════════════════════════════════════════════
# SHARED STATE
# ══════════════════════════════════════════════════════
current_pose = {"x":0.0,"y":0.0,"z":0.0,"state":3}
current_dets = []
pose_lock    = threading.Lock()
det_lock     = threading.Lock()
running      = True
_pose_count  = 0

def pose_recv_thread():
    global _pose_count
    STATE_NAMES = {0:"TRACKING",1:"RECENTLY_LOST",2:"LOST",3:"INITIALIZING"}
    print(f"[POSE] Listening on localhost:{LOCAL_POSE_PORT}")
    while running:
        try:
            data,_ = pose_sock.recvfrom(128)
            parts  = data.decode().strip().split()
            if len(parts) >= 4:
                x,y,z,s = float(parts[0]),float(parts[1]),\
                           float(parts[2]),int(parts[3])
                with pose_lock:
                    current_pose.update({"x":x,"y":y,"z":z,"state":s})
                _pose_count += 1
                if _pose_count == 1:
                    print(f"[POSE] First pose!  "
                          f"state={STATE_NAMES.get(s,'?')}  "
                          f"x={x:.3f} z={z:.3f}")
        except socket.timeout: pass
        except Exception as e:
            if running: print(f"[POSE] {e}")

def det_recv_thread():
    print(f"[DET ] Listening on localhost:{LOCAL_DET_PORT}")
    while running:
        try:
            data,_ = det_sock.recvfrom(8192)
            dets   = json.loads(data.decode())
            with det_lock:
                current_dets.clear()
                current_dets.extend(dets)
        except socket.timeout: pass
        except Exception as e:
            if running: print(f"[DET ] {e}")

# ══════════════════════════════════════════════════════
# SHUTDOWN
# ══════════════════════════════════════════════════════
def shutdown(sig=None, frame=None):
    global running
    print("\n[MAIN] Shutting down...")
    running = False
    time.sleep(0.3)
    motors_shutdown()
    pose_sock.close()
    det_sock.close()
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ══════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════
def main():
    print()
    print("═"*52)
    print("  RPi5 Motor Brain — SLAM + YOLO Navigation")
    print("═"*52)
    print(f"  Pose  ← localhost:{LOCAL_POSE_PORT}")
    print(f"  Dets  ← localhost:{LOCAL_DET_PORT}")
    print(f"  LEFT  motor: RPWM=12 LPWM=13 EN=24,25")
    print(f"  RIGHT motor: RPWM=18 LPWM=19 EN=20,21")
    print(f"  lgpio: {'OK' if LGPIO_OK else 'MISSING — dry-run'}")
    print()
    print("  Behaviour:")
    print("    INITIALIZING → SPIN slowly (helps SLAM find features)")
    print("    TRACKING     → EXPLORE forward")
    if SMALL_DIRT_MODE:
        print("    YOLO dirt seen → TARGET SMALL boxes only (closest first)")
        print(f"    Small box: {SMALL_BOX_MIN_AREA}–{SMALL_BOX_MAX_AREA} px²  |  set SMALL_DIRT_MODE=False to disable")
    else:
        print("    YOLO dirt seen → GO TO closest dirt (original mode)")
    print("    Arrived at dirt → WAIT 1 second → next queued patch or EXPLORE")
    print("    Liquid ahead → AVOID (turn right 2s)")
    print("    LOST/RECENTLY_LOST → SPIN to recover")
    print("═"*52)
    print()

    motors_init()
    brain = RobotBrain()

    threading.Thread(target=pose_recv_thread, daemon=True).start()
    threading.Thread(target=det_recv_thread,  daemon=True).start()

    print("[MAIN] Starting immediately — spinning while SLAM initializes")
    print()

    INTERVAL   = 0.05   # 20 Hz
    STATE_NAMES = {0:"TRACKING",1:"RECENTLY_LOST",
                   2:"LOST",    3:"INITIALIZING"}

    while running:
        t0 = time.time()

        with pose_lock: pose = dict(current_pose)
        with det_lock:  dets = list(current_dets)

        brain.update_pose(pose["x"], pose["z"])
        l, r = brain.step(pose["state"], dets)

        # Status every 2 seconds
        if int(t0*0.5) != int((t0-INTERVAL)*0.5):
            sn = STATE_NAMES.get(pose["state"], "?")
            dirt_str = (f"→({brain._immediate_target[0]:.2f},"
                        f"{brain._immediate_target[1]:.2f})"
                        if brain._immediate_target else "none")
            sweep_str = (f"leg {brain._sweep._idx+1}/{SWEEP_LEGS}"
                         if brain._sweep_active and brain.state == "EXPLORING"
                         else "—")
            print(f"[STAT] SLAM:{sn:<14} Brain:{brain.state:<16} "
                  f"L:{l:+.0f}%  R:{r:+.0f}%  "
                  f"Dirt:{dirt_str}  Sweep:{sweep_str}  "
                  f"Poses:{_pose_count}")

        elapsed = time.time() - t0
        if elapsed < INTERVAL:
            time.sleep(INTERVAL - elapsed)

if __name__ == "__main__":
    main()
