import time
from smbus2 import SMBus
import pygame
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import subprocess
import os
import numpy as np
import datetime
import signal

signal.signal(signal.SIGTERM, lambda s, f: os._exit(0))
signal.signal(signal.SIGINT, lambda s, f: os._exit(0))

# ------------------------
# --- Launch ROS2 Stack ---
# ------------------------
ros_launch_proc = None

def launch_ros_stack():
    global ros_launch_proc
    env = os.environ.copy()
    env['DISPLAY'] = ':0'
    try:
        ros_launch_proc = subprocess.Popen(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'source /home/outthawazoo/traxxas_ws/install/setup.bash && '
             'ros2 launch traxxas_slam slam_launch.py'],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print("ROS2 SLAM stack launched (RViz opening...)")
    except Exception as e:
        print(f"Failed to launch ROS2 stack: {e}")

def save_map():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    map_name = f"/home/outthawazoo/map_{timestamp}"
    print(f"Saving map to {map_name}...")
    try:
        subprocess.Popen(
            ['bash', '-c',
             f'source /opt/ros/humble/setup.bash && '
             f'ros2 run nav2_map_server map_saver_cli -f {map_name}'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"Map save triggered: {map_name}.pgm / .yaml")
    except Exception as e:
        print(f"Map save failed: {e}")

# ------------------------
# --- Hallway Navigator Tuning ---
# ------------------------
DEBUG = False   # set True to enable verbose PID/sector logging

STOP_DISTANCE      = 0.6    # end wall detection (m) — slightly larger margin
OBSTACLE_DISTANCE  = 0.8    # obstacle avoidance trigger (m)

# PD tuning (integral removed — causes drift accumulation in hallways)
Kp = 0.35
Kd = 0.18   # acts on time-normalized, low-pass filtered derivative
MAX_STEER = 0.4

# --- Stop-and-Go Parameters (user confirmed: 0.4 drive, 1.0 wait) ---
DRIVE_DURATION     = 0.4    # seconds to drive forward each burst
STOP_DURATION      = 1.3    # seconds to stop and sense

# LiDAR sector angles
FRONT_ANGLE        = 0
LEFT_ANGLE         = 90
RIGHT_ANGLE        = 270

# Confirmed PWM values (tested physically):
# 220 = full RIGHT
# 453 = center
# 600 = full LEFT
STEER_RIGHT        = 220
STEER_CENTER       = 453
STEER_LEFT         = 600
locked_center      = 453    # updated by trim, used by steer_pwm

# ------------------------
# --- Setup Joystick ---
# ------------------------
pygame.init()
pygame.joystick.init()

# ------------------------
# --- PCA9685 Setup ---
# ------------------------
PCA9685_ADDRESS = 0x50
MODE1    = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
FREQ = 50

bus = SMBus(1)

def write_reg(reg, value):
    bus.write_byte_data(PCA9685_ADDRESS, reg, value)

write_reg(MODE1, 0x00)
prescale_val = int(math.floor(25000000.0 / (4096 * FREQ) - 1))
old_mode = bus.read_byte_data(PCA9685_ADDRESS, MODE1)
new_mode = (old_mode & 0x7F) | 0x10
write_reg(MODE1, new_mode)
write_reg(PRESCALE, prescale_val)
write_reg(MODE1, old_mode)
time.sleep(0.005)
write_reg(MODE1, old_mode | 0x80)

# ------------------------
# --- PWM Helper ---
# ------------------------
def set_pwm(channel, on, off):
    bus.write_byte_data(PCA9685_ADDRESS, LED0_ON_L + 4 * channel,     on  & 0xFF)
    bus.write_byte_data(PCA9685_ADDRESS, LED0_ON_L + 4 * channel + 1, on  >> 8)
    bus.write_byte_data(PCA9685_ADDRESS, LED0_ON_L + 4 * channel + 2, off & 0xFF)
    bus.write_byte_data(PCA9685_ADDRESS, LED0_ON_L + 4 * channel + 3, off >> 8)

# ------------------------
# --- Channels & Constants ---
# ------------------------
servo_channel = 15
motor_channel = 14

PWM_MIN    = 150
PWM_MAX    = 600
CENTER_PWM = 453
TRIM_STEP  = 1

MOTOR_STOP_PWM    = 300
MOTOR_FORWARD_MAX = 315     # user confirmed working value
MOTOR_REVERSE_MAX = 280
DEADBAND = 0.02

MODE_MANUAL     = "MANUAL"
MODE_AUTONOMOUS = "AUTONOMOUS"
mode = MODE_MANUAL

last_a_state = 0
last_b_state = 0

# ------------------------
# --- Utility Functions ---
# ------------------------
def map_range(value, in_min, in_max, out_min, out_max):
    value = max(in_min, min(in_max, value))
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def stick_to_pwm(x):
    span = (PWM_MAX - PWM_MIN) / 2
    return int(CENTER_PWM + x * span)

def get_throttle_pwm(rt_val, lt_val):
    if rt_val > DEADBAND:
        return map_range(rt_val, 0, 1, MOTOR_STOP_PWM, MOTOR_FORWARD_MAX)
    elif lt_val > DEADBAND:
        return map_range(lt_val, 0, 1, MOTOR_STOP_PWM, MOTOR_REVERSE_MAX)
    return MOTOR_STOP_PWM

def clamp_trigger(val):
    return 0 if val < DEADBAND else val

def apply_deadband(x, deadband=DEADBAND):
    return 0.0 if abs(x) < deadband else x

# Steering inversion — change ONLY this constant if servo direction is reversed.
# +1 = steering correct as-is, -1 = physically reversed.
STEER_INVERT = -1

def steer_pwm(strength):
    """
    strength: -1.0 (full RIGHT) to +1.0 (full LEFT), 0.0 = center.
    Linear mapping. All inversion lives in STEER_INVERT — nowhere else.
    Center is hard-clamped so spans never go negative regardless of trim drift.
    """
    global locked_center
    strength = max(-1.0, min(1.0, strength)) * STEER_INVERT
    # Hard clamp ensures both spans stay positive even if trim drifts far off-center
    center     = max(STEER_RIGHT + 10, min(STEER_LEFT - 10, locked_center))
    left_span  = STEER_LEFT  - center
    right_span = center - STEER_RIGHT
    if strength >= 0:
        return int(center + strength * left_span)
    else:
        return int(center + strength * right_span)

# ------------------------
# --- Initialize Actuators ---
# ------------------------
set_pwm(servo_channel, 0, CENTER_PWM)
set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
time.sleep(0.5)

# ------------------------
# --- Wait for Controller ---
# ------------------------
sweep_direction = 1
sweep_pwm = CENTER_PWM
SWEEP_STEP = 2
SWEEP_MIN  = PWM_MIN + 50
SWEEP_MAX  = PWM_MAX - 50
sweep_delay = 0.02

print("--- RC Car Initializing ---")
print("Waiting for Xbox controller... Steering will sweep slowly.")

while pygame.joystick.get_count() == 0:
    sweep_pwm += sweep_direction * SWEEP_STEP
    if sweep_pwm >= SWEEP_MAX:
        sweep_pwm = SWEEP_MAX
        sweep_direction = -1
    elif sweep_pwm <= SWEEP_MIN:
        sweep_pwm = SWEEP_MIN
        sweep_direction = 1
    set_pwm(servo_channel, 0, sweep_pwm)
    pygame.joystick.quit()
    pygame.joystick.init()
    time.sleep(sweep_delay)

js = pygame.joystick.Joystick(0)
js.init()

print(f"Controller detected: {js.get_name()}")
print("Press X button to terminate program.")
print("Launching ROS2 SLAM stack...")
launch_ros_stack()
time.sleep(5)
print("ROS2 stack ready!")

# ------------------------
# --- ROS2 Nodes ---
# ------------------------
class CmdVelListener(Node):
    def __init__(self):
        super().__init__('traxxas_cmd_vel')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.linear_x  = 0.0
        self.angular_z = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_x  = msg.linear.x
        self.angular_z = msg.angular.z


class ScanListener(Node):
    def __init__(self):
        super().__init__('traxxas_scan_listener')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.ranges          = None
        self.active          = threading.Event()  # Fix 1: lock-free atomic flag
        self.angle_min       = None
        self.angle_increment = None
        self.last_scan_time  = 0.0

    def scan_callback(self, msg):
        if not self.active.is_set():
            return
        # Fix 7: explicit copy prevents view-sharing with ROS internal buffer
        self.ranges          = np.array(msg.ranges, copy=True)
        self.angle_min       = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.last_scan_time  = time.time()
        self.ranges = np.where(np.isfinite(self.ranges), self.ranges, np.nan)
        hallway.lidar_warmup = min(hallway.lidar_warmup + 1, 10)


rclpy.init()
cmd_vel_node  = CmdVelListener()
scan_node     = ScanListener()

def ros_spin():
    executor = MultiThreadedExecutor()
    executor.add_node(cmd_vel_node)
    executor.add_node(scan_node)
    executor.spin()

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()
print("ROS2 nodes started (cmd_vel + scan listeners)")

# ------------------------
# --- LiDAR Sector Helper ---
# ------------------------
def get_sector_distance(ranges, angle_center_deg, width_deg=20):
    """
    Uses actual angle_min + angle_increment from the scan message to find
    the correct index for any requested angle. This is immune to scan
    direction (CW vs CCW) and zero-reference offsets.
    Falls back to uniform-index math if metadata isn't available yet.
    Returns median of valid readings, or None if fewer than 5 valid points.
    """
    total = len(ranges)
    angle_center_rad = np.deg2rad(angle_center_deg)
    half_width_rad   = np.deg2rad(width_deg / 2.0)

    if scan_node.angle_min is not None and scan_node.angle_increment is not None:
        # Anchor to actual scan geometry from the message
        angle_min = scan_node.angle_min
        angle_inc = scan_node.angle_increment
        # All angle values for every index
        angles = angle_min + np.arange(total) * angle_inc
        # Wrap to [-pi, pi] for consistent comparison
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        target = (angle_center_rad + np.pi) % (2 * np.pi) - np.pi
        # Select indices within the requested sector window
        mask = np.abs(angles - target) <= half_width_rad
        vals = ranges[mask]
    else:
        # Fallback: uniform index math (original method)
        idx_center = int(((angle_center_deg + 180) % 360) / 360.0 * total) % total
        idx_half   = int(width_deg / 2 / 360.0 * total)
        indices    = [(idx_center + i) % total for i in range(-idx_half, idx_half)]
        vals       = ranges[np.array(indices)]

    # Keep only finite readings in valid range
    vals = vals[np.isfinite(vals) & (vals > 0.15) & (vals < 5.0)]

    if len(vals) < 5:
        return None  # not enough valid points — caller must handle
    return float(np.median(vals))  # median is robust against outlier spikes

# ------------------------
# --- Hallway State (PID) ---
# ------------------------
class HallwayState:
    def __init__(self):
        self.state            = 'WAITING'   # WAITING, DRIVE, STOP, DONE
        self.state_start_time = time.time()
        self.map_saved        = False
        # PD state
        self.prev_error       = 0.0
        self.derivative       = 0.0        # filtered derivative (low-pass)
        self.last_time        = time.time() # for real dt measurement
        self.current_steering = CENTER_PWM
        self.pid_initialized  = False      # True after first valid scan seeds prev_error
        self.stop_confirm     = 0          # consecutive scans below STOP_DISTANCE needed to stop
        # Output smoothing
        self.prev_steer_input = 0.0        # for slew limiter
        self.smooth_steer_pwm = CENTER_PWM # for steering output low-pass filter
        # Sensor state (owned here to avoid global thread races)
        self.lidar_warmup     = 0          # incremented by ROS scan callback

hallway = HallwayState()

# ------------------------
# --- Steering Computation (PID) ---
# ------------------------
def compute_steering(ranges):
    # Time-based gate: block until LiDAR has been publishing for at least 0.5s
    # and a scan arrived recently. Immune to scan rate variations.
    if scan_node.last_scan_time == 0 or time.time() - scan_node.last_scan_time > 0.5:
        if DEBUG:
            print("[WARMUP] waiting for stable LiDAR stream")
        return steer_pwm(0.0), 999.0, MOTOR_STOP_PWM

    # Narrow front sector (30°) — prevents side walls during turns from triggering stop
    front = get_sector_distance(ranges, FRONT_ANGLE, width_deg=30)
    left  = get_sector_distance(ranges, LEFT_ANGLE,  width_deg=40)
    right = get_sector_distance(ranges, RIGHT_ANGLE, width_deg=40)

    # Sector data insufficient — hold last steering, reduce speed but don't stop
    if left is None or right is None:
        if DEBUG:
            print(f"[SKIP] insufficient sector data — L:{left} R:{right}")
        safe_speed = int(MOTOR_STOP_PWM + (MOTOR_FORWARD_MAX - MOTOR_STOP_PWM) * 0.6)
        return steer_pwm(hallway.prev_steer_input), front if front is not None else 999.0, safe_speed

    # Front sector fallback
    if front is None:
        front = 999.0

    if DEBUG:
        print(f"[READ] F:{front:.2f} L:{left:.2f} R:{right:.2f}  |  "
              f"RAW_ERR_SIGN: {'LEFT' if (right-left) > 0 else 'RIGHT'}")

    # End wall: 3 consecutive scans required to avoid false stops from doorways/turns.
    # When confirmed, slow + straighten — state machine transition happens in DRIVE caller.
    STOP_CONFIRM_NEEDED = 3
    if front < STOP_DISTANCE:
        hallway.stop_confirm += 1
        if DEBUG:
            print(f"[STOP?] front:{front:.2f} confirm:{hallway.stop_confirm}/{STOP_CONFIRM_NEEDED}")
        if hallway.stop_confirm >= STOP_CONFIRM_NEEDED:
            # Signal caller: confirmed end wall
            return "DONE", front, MOTOR_STOP_PWM
        # Unconfirmed — slow down, hold heading, keep rolling
        approach_speed = int(MOTOR_STOP_PWM + (MOTOR_FORWARD_MAX - MOTOR_STOP_PWM) * 0.3)
        return steer_pwm(hallway.prev_steer_input), front, approach_speed
    else:
        hallway.stop_confirm = 0

    # Lost both walls — go straight at full speed
    if left > 5.0 and right > 5.0:
        return steer_pwm(0.0), front, MOTOR_FORWARD_MAX

    # Obstacle avoidance: steer bias toward open side — reduce speed but don't stop
    if front < OBSTACLE_DISTANCE:
        turn = (OBSTACLE_DISTANCE - front) / OBSTACLE_DISTANCE
        turn = min(0.4, turn)
        turn = turn if left > right else -turn
        obs_speed = int(MOTOR_STOP_PWM + (MOTOR_FORWARD_MAX - MOTOR_STOP_PWM) * 0.4)
        if DEBUG:
            print(f"[OBSTACLE] front:{front:.2f} bias:{turn:.2f}")
        return steer_pwm(turn), front, obs_speed

    # --- Distance low-pass filter (suppress per-scan LiDAR noise) ---
    alpha = 0.6
    if hasattr(hallway, 'left_smooth'):
        hallway.left_smooth  = alpha * hallway.left_smooth  + (1 - alpha) * left
        hallway.right_smooth = alpha * hallway.right_smooth + (1 - alpha) * right
    else:
        hallway.left_smooth  = left
        hallway.right_smooth = right
    left  = hallway.left_smooth
    right = hallway.right_smooth

    # --- Stable centerline geometry controller ---
    # Normalize by hallway width (not average) — scale-invariant, noise-resistant.
    # error > 0 → closer to right wall → steer left
    # error < 0 → closer to left wall  → steer right
    hall_width = left + right
    if hall_width < 0.2:
        return steer_pwm(0.0), front, MOTOR_FORWARD_MAX  # degenerate geometry
    error = (right - left) / hall_width
    error = max(-1.0, min(1.0, error))

    # Seed prev_error on first valid tick so derivative starts at zero
    if not hallway.pid_initialized:
        hallway.prev_error    = error
        hallway.derivative    = 0.0
        hallway.pid_initialized = True
        hallway.last_time     = time.time()
        if DEBUG:
            print("[PD] initialized")
        return steer_pwm(0.0), front, MOTOR_FORWARD_MAX

    # Real dt — loop timing is never perfectly 20ms due to I2C + ROS scheduling
    now = time.time()
    dt  = max(0.005, min(0.1, now - hallway.last_time))  # clamp 5ms–100ms
    hallway.last_time = now

    # Low-pass filtered derivative — removes LiDAR spike twitching
    raw_d            = (error - hallway.prev_error) / dt
    hallway.derivative = 0.7 * hallway.derivative + 0.3 * raw_d

    hallway.prev_error = error

    steer_input = Kp * error + Kd * hallway.derivative
    steer_input = max(-MAX_STEER, min(MAX_STEER, steer_input))

    # Tighter slew limiter — prevents direction flips from single bad scans
    MAX_DELTA   = 0.04
    steer_input = hallway.prev_steer_input + max(
        -MAX_DELTA, min(MAX_DELTA, steer_input - hallway.prev_steer_input))
    hallway.prev_steer_input = steer_input

    # Velocity: ONLY front distance controls speed — steering never brakes the robot.
    # Clamp minimum to 0.2 so it never fully stalls in a wide-open hallway.
    front_scale = min(1.0, max(0.2, front / 1.5))
    speed_pwm   = int(MOTOR_STOP_PWM +
                      (MOTOR_FORWARD_MAX - MOTOR_STOP_PWM) * front_scale)

    raw_pwm = steer_pwm(steer_input)

    # Higher-inertia output smoother — 85/15 weight kills micro-oscillation
    hallway.smooth_steer_pwm = int(
        0.85 * hallway.smooth_steer_pwm + 0.15 * raw_pwm)

    if DEBUG:
        print(f"[PD] err:{error:.3f} d:{hallway.derivative:.3f} dt:{dt:.4f} "
              f"steer:{steer_input:.3f} spd:{speed_pwm} pwm:{hallway.smooth_steer_pwm} "
              f"| L:{left:.2f} R:{right:.2f} W:{hall_width:.2f} fs:{front_scale:.2f}")
    return hallway.smooth_steer_pwm, front, speed_pwm

# ------------------------
# --- Hallway Step (PID + Stop-and-Go) ---
# ------------------------
def run_hallway_step():
    global locked_center
    elapsed = time.time() - hallway.state_start_time

    # --- STATE MACHINE ---
    if hallway.state == 'WAITING':
        set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
        set_pwm(servo_channel, 0, locked_center)
        if elapsed > 1.0:   # user confirmed 1s wait
            print("Starting hallway navigation!")
            hallway.state            = 'DRIVE'
            hallway.state_start_time = time.time()

    elif hallway.state == 'DRIVE':
        scan_fresh = (scan_node.ranges is not None and
                      scan_node.angle_increment is not None and
                      time.time() - scan_node.last_scan_time < 0.15)
        if scan_fresh:
            result = compute_steering(scan_node.ranges)
            if result is not None:
                steering, front, speed_pwm = result
                if steering == "DONE":
                    print(f"End wall at {front:.2f}m — done!")
                    set_pwm(servo_channel, 0, locked_center)
                    set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
                    hallway.state = 'DONE'
                    return
                hallway.current_steering = steering
                set_pwm(servo_channel, 0, hallway.current_steering)
                set_pwm(motor_channel, 0, speed_pwm)
        else:
            # Stale scan: hold last servo position, run at 60% — never full stop
            safe_speed = int(MOTOR_STOP_PWM + (MOTOR_FORWARD_MAX - MOTOR_STOP_PWM) * 0.6)
            set_pwm(motor_channel, 0, safe_speed)
        if elapsed > DRIVE_DURATION:
            set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
            hallway.state            = 'STOP'
            hallway.state_start_time = time.time()

    elif hallway.state == 'STOP':
        # Freeze motor; hold the smoothed steering output — same signal DRIVE writes,
        # so there is no discontinuity when DRIVE resumes.
        set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
        set_pwm(servo_channel, 0, hallway.smooth_steer_pwm)
        if elapsed > STOP_DURATION:
            hallway.state            = 'DRIVE'
            hallway.state_start_time = time.time()

    elif hallway.state == 'DONE':
        set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
        set_pwm(servo_channel, 0, locked_center)
        if not hallway.map_saved:
            save_map()
            hallway.map_saved = True

# ------------------------
# --- Main Control Loop ---
# ------------------------
throttle_pwm_val = MOTOR_STOP_PWM
last_trim_time   = 0
TRIM_DELAY       = 0.05

# Button indices: 0=A, 1=B, 2=Y, 3=X

try:
    while True:
        pygame.event.pump()

        if js.get_button(3):
            print("X button pressed! Exiting program...")
            os._exit(0)

        a_pressed = js.get_button(0)
        b_pressed = js.get_button(1)

        if a_pressed and not last_a_state:
            mode             = MODE_AUTONOMOUS
            throttle_pwm_val = MOTOR_STOP_PWM
            set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
            locked_center = CENTER_PWM          # freeze current trim as center
            # Reset hallway state machine + PID
            hallway.state            = 'WAITING'
            hallway.state_start_time = time.time()
            hallway.map_saved        = False
            hallway.current_steering = locked_center
            hallway.prev_error       = 0.0
            hallway.derivative       = 0.0
            hallway.last_time        = time.time()
            hallway.pid_initialized  = False
            hallway.prev_steer_input = 0.0
            hallway.smooth_steer_pwm = locked_center
            hallway.stop_confirm     = 0
            # Reset smoothed distance filters
            if hasattr(hallway, 'left_smooth'):
                del hallway.left_smooth
            if hasattr(hallway, 'right_smooth'):
                del hallway.right_smooth
            # Full scan buffer reset — clears any buffered stale geometry
            hallway.lidar_warmup       = 0
            scan_node.ranges           = None
            scan_node.angle_min        = None
            scan_node.angle_increment  = None
            scan_node.last_scan_time   = 0.0
            set_pwm(servo_channel, 0, locked_center)
            scan_node.active.set()       # Fix 1: atomic Event flag
            print(f">>> Switched to AUTONOMOUS mode | Center locked at PWM={locked_center}")

        if b_pressed and not last_b_state:
            mode             = MODE_MANUAL
            throttle_pwm_val = MOTOR_STOP_PWM
            set_pwm(servo_channel, 0, CENTER_PWM)
            set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
            scan_node.active.clear()     # Fix 1: atomic Event flag
            scan_node.ranges = None
            print(">>> Switched to MANUAL mode")

        last_a_state = a_pressed
        last_b_state = b_pressed

        # --- TRIM (manual mode only — sets center before going autonomous) ---
        if mode == MODE_MANUAL:
            hat_x, hat_y = js.get_hat(0)
            current_time = time.time()
            if hat_x != 0 and current_time - last_trim_time > TRIM_DELAY:
                CENTER_PWM  += hat_x * TRIM_STEP
                CENTER_PWM   = max(PWM_MIN, min(PWM_MAX, CENTER_PWM))
                locked_center = CENTER_PWM
                set_pwm(servo_channel, 0, CENTER_PWM)
                print(f"Trim: CENTER_PWM={CENTER_PWM}")
                last_trim_time = current_time

        # --- MANUAL MODE ---
        if mode == MODE_MANUAL:
            x       = -js.get_axis(0)
            pwm_val = stick_to_pwm(x)
            set_pwm(servo_channel, 0, pwm_val)

            rt_raw = js.get_axis(4)
            lt_raw = js.get_axis(5)

            rt = clamp_trigger((rt_raw + 1) / 2)
            lt = clamp_trigger((lt_raw + 1) / 2)

            target_pwm = MOTOR_STOP_PWM
            if rt > 0:
                target_pwm = get_throttle_pwm(rt, 0)
            elif lt > 0:
                target_pwm = get_throttle_pwm(0, lt)

            throttle_pwm_val = target_pwm
            set_pwm(motor_channel, 0, throttle_pwm_val)
            if DEBUG:
                print(f"Throttle PWM: {throttle_pwm_val}")

        # --- AUTONOMOUS MODE ---
        elif mode == MODE_AUTONOMOUS:
            run_hallway_step()

        time.sleep(0.02)  # 50 Hz — tighter loop for steering responsiveness

except Exception as e:
    print(f"Unexpected error: {e}")

finally:
    set_pwm(servo_channel, 0, CENTER_PWM)
    set_pwm(motor_channel, 0, MOTOR_STOP_PWM)
    try:
        rclpy.shutdown()
    except Exception:
        pass
    os._exit(0)
