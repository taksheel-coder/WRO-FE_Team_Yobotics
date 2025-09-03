
Full integrated navigator script.
Uses camera-only stuck detection (frame-diff window) and a robust recovery routine.
Keeps your original geometry, mask, ROIs and wiring:
  - Servo: GPIO 18
  - Motor ENA: GPIO 13, IN1: 17, IN2: 27

Behavior summary:
  - Normal forward with single main ROI + early ROI for pre-detection.
  - If early/main ROI detect a wall, do a timed reverse then scan and commit to a turn.
  - If camera shows almost-no-change for a whole window (FREEZE_SEC) -> robust recovery:
      tries plain reverse attempts, reverse+preferred-steer attempts, fallback scan.
  - Preferred side used when stuck is chosen by which side the bot turned more during the run.
  - Keeps debug windows to tune visually.
Tweak constants at top for speed/timings/sensitivity.

import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
import time

# ---------------- SETTINGS / HARDWARE --------------------------------
W, H = 640, 480
F_PIX = 96.0
PITCH_DEG = 20.0
PITCH = math.radians(PITCH_DEG)
H_CAM_CM = 13.0
CY = H / 2.0

D_TURN_CM = 20.0
D_STOP_CM = 10.0
EARLY_TURN_CM = 14.0  # early-turn distance (you can lower to turn later or raise to turn earlier)

X_MARGIN = 0.06
BOTTOM_PAD_PX = 6
TOP_PAD_PX = 8

# vision thresholds
HSV_S_MAX = 80
HSV_V_MAX = 80
ADAPT_BLOCK = 31
ADAPT_C = 5
KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
OPEN_ITERS = 1
CLOSE_ITERS = 2

MIN_BLACK_RATIO_ROI = 0.08
MIN_BLACK_RATIO_BOTTOM = 0.06

# servo/motor pins (as you provided)
SERVO_PIN = 18
SERVO_CENTER = 90
SERVO_MIN = 45
SERVO_MAX = 155

ENA = 13
IN1 = 17
IN2 = 27

# behavior tuning
BASE_SPEED = 85           # forward PWM %
TURN_SPEED = 45           # speed while committing a turn
REVERSE_DURATION = 1.8    # seconds to reverse when recovering
FREEZE_SEC = 1.0          # stuck detection window (seconds)
SCAN_SETTLE = 0.75        # seconds to wait after servo move during scan

# contour filtering for ignoring lane paint
MIN_CONTOUR_WIDTH = 12
MIN_CONTOUR_HEIGHT = 10
ASPECT_MIN = 0.25
ASPECT_MAX = 4.0

# recovery tuning
RECOVER_MAX_TRIES = 3
FRAME_MOVE_THRESH = 3.0   # mean-diff to consider that camera saw movement after an action

# frame-diff window thresholds
DIFF_WINDOW_SEC = FREEZE_SEC
MOTION_THRESHOLD = 2.5    # if every diff in window < this -> considered not moving

# ---------------- GPIO SETUP ------------------------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

servo_pwm = GPIO.PWM(SERVO_PIN, 50)    # 50Hz for servo
servo_pwm.start(0)

motor_pwm = GPIO.PWM(ENA, 1000)        # 1kHz for motor speed
motor_pwm.start(0)

# ---------------- ACTUATORS ------------------------------------------
def set_servo_angle(angle: int):
    duty = 2 + (angle / 18.0)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.06)
    servo_pwm.ChangeDutyCycle(0)

def motor_forward(speed_percent=BASE_SPEED):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(np.clip(speed_percent, 0, 100))

def motor_backward(speed_percent=BASE_SPEED):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(np.clip(speed_percent, 0, 100))

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)

# ---------------- VISION HELPERS -------------------------------------
def y_from_distance_cm(z_cm: float) -> int:
    if z_cm <= 0:
        return H - 1
    a = math.atan2(H_CAM_CM, z_cm) - PITCH
    v = CY + F_PIX * math.tan(a)
    return int(np.clip(v, 0, H - 1))

def build_roi_band(dist=D_TURN_CM, top_pad=TOP_PAD_PX, bottom_pad=BOTTOM_PAD_PX):
    y_top = y_from_distance_cm(dist) - top_pad
    y_bot = y_from_distance_cm(D_STOP_CM) + bottom_pad
    if y_bot <= y_top:
        y_bot = min(H - 1, y_top + 40)
    x1 = int(W * X_MARGIN)
    x2 = int(W * (1.0 - X_MARGIN))
    y1 = int(max(0, min(y_top, H - 2)))
    y2 = int(max(y1 + 10, min(H - 1, y_bot)))
    return (x1, y1, x2, y2)

def black_mask(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    mask_black = cv2.inRange(hsv, (0, 0, 0), (180, HSV_S_MAX, HSV_V_MAX))
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    adapt = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                  cv2.THRESH_BINARY_INV, ADAPT_BLOCK, ADAPT_C)
    mask = cv2.bitwise_or(mask_black, adapt)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=OPEN_ITERS)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=CLOSE_ITERS)
    return mask, hsv

def analyze_front_wall(roi_bgr):
    mask, hsv = black_mask(roi_bgr)
    h, w = mask.shape
    total_ratio = float(cv2.countNonZero(mask)) / (h * w + 1e-8)
    if total_ratio < MIN_BLACK_RATIO_ROI:
        return False, None, None, total_ratio, 0.0
    bottom_strip = mask[max(0, h - 20):h, :]
    bottom_ratio = float(cv2.countNonZero(bottom_strip)) / (bottom_strip.size + 1e-8)
    if bottom_ratio < MIN_BLACK_RATIO_BOTTOM:
        return False, None, None, total_ratio, bottom_ratio
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    touching = []
    for c in contours:
        x, y, w0, h0 = cv2.boundingRect(c)
        if w0 < MIN_CONTOUR_WIDTH or h0 < MIN_CONTOUR_HEIGHT:
            continue
        aspect = w0 / float(h0 + 1e-5)
        if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
            continue
        roi_hsv = hsv[y:y+h0, x:x+w0]
        if roi_hsv.size == 0:
            continue
        avg_hue = float(np.mean(roi_hsv[:, :, 0]))
        if (90 <= avg_hue <= 130 or 5 <= avg_hue <= 20) and (h0 < 28 or w0 < 28):
            continue
        if y + h0 >= h - 2:
            touching.append((c, x, y, w0, h0))
    if not touching:
        return False, None, None, total_ratio, bottom_ratio
    c, x, y, w0, h0 = max(touching, key=lambda t: t[3])
    if w0 < 0.4 * mask.shape[1] or h0 < 6:
        return False, None, None, total_ratio, bottom_ratio
    top_edge_row = y
    return True, (x, y, w0, h0), top_edge_row, total_ratio, bottom_ratio

def estimate_distance_from_top_edge(roi_y1, roi_top_edge_row):
    v = roi_y1 + roi_top_edge_row
    ang = PITCH + math.atan2((v - CY), F_PIX)
    if ang <= 1e-3:
        return float('inf')
    z = H_CAM_CM / math.tan(ang)
    return max(0.0, z)

# ---------------- Recovery scanning ----------------------------------
def scan_free_direction(cap, W, H):
    best_dir = 1.0
    best_free = -1.0
    for d in (-1.0, 1.0):
        angle = int(np.clip(SERVO_CENTER + d * (SERVO_MAX - SERVO_CENTER),
                            SERVO_MIN, SERVO_MAX))
        set_servo_angle(angle)
        time.sleep(SCAN_SETTLE)
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.resize(frame, (W, H))
        x1, y1, x2, y2 = build_roi_band(D_TURN_CM)
        roi = frame[y1:y2, x1:x2]
        mask, _ = black_mask(roi)
        h_roi, w_roi = mask.shape
        bottom_h = min(24, h_roi)
        bs = mask[h_roi-bottom_h:h_roi, :]
        left = bs[:, :w_roi // 2]
        right = bs[:, w_roi // 2:]
        left_free = 1.0 - float(cv2.countNonZero(left)) / (left.size + 1e-8)
        right_free = 1.0 - float(cv2.countNonZero(right)) / (right.size + 1e-8)
        free_metric = max(left_free, right_free)
        if free_metric > best_free:
            best_free = free_metric
            best_dir = d
    return best_dir

# ---------------- Robust recovery routine ----------------------------
def recover_from_stuck(cap, prev_frame_gray, preferred_side=None):
    """
    Try multiple reverse attempts and reverse+steer attempts until the camera frame changes.
    preferred_side: -1.0 for left, +1.0 for right, or None.
    Returns (succeeded:bool, new_gray, used_dir) where used_dir is -1/0/+1 (0 = plain reverse)
    """
    print("[RECOVER] enter robust recovery")
    set_servo_angle(SERVO_CENTER)
    motor_stop()
    time.sleep(0.08)

    def frame_changed():
        ok, cur = cap.read()
        if not ok:
            return False, None
        cur = cv2.resize(cur, (W, H))
        cur_gray = cv2.cvtColor(cur, cv2.COLOR_BGR2GRAY)
        d = cv2.absdiff(prev_frame_gray, cur_gray)
        mean_d = float(np.mean(d))
        return (mean_d > FRAME_MOVE_THRESH), cur_gray

    # 1) Plain reverse attempts
    for attempt in range(RECOVER_MAX_TRIES):
        print(f"[RECOVER] plain reverse attempt {attempt+1}")
        motor_backward(BASE_SPEED)
        time.sleep(REVERSE_DURATION)
        motor_stop()
        time.sleep(0.06)
        changed, new_gray = frame_changed()
        if changed:
            print("[RECOVER] moved on plain reverse")
            return True, new_gray, 0
        time.sleep(0.06)

    # 2) Try preferred side first if provided
    steer_order = []
    if preferred_side is not None:
        steer_order.append(preferred_side)
        steer_order.append(-preferred_side)
    else:
        steer_order = [-1.0, 1.0]

    for steer_dir in steer_order:
        for attempt in range(RECOVER_MAX_TRIES):
            angle = int(np.clip(SERVO_CENTER + steer_dir * (SERVO_MAX - SERVO_CENTER),
                                SERVO_MIN, SERVO_MAX))
            print(f"[RECOVER] reverse+steer {'LEFT' if steer_dir<0 else 'RIGHT'} attempt {attempt+1}")
            set_servo_angle(angle)
            time.sleep(0.06)
            motor_backward(BASE_SPEED)
            time.sleep(REVERSE_DURATION * 0.9)
            motor_stop()
            time.sleep(0.06)
            changed, new_gray = frame_changed()
            if changed:
                print("[RECOVER] moved with reverse+steer")
                return True, new_gray, int(np.sign(steer_dir))
            time.sleep(0.06)

    # 3) fallback: scan & commit forward briefly
    print("[RECOVER] fallback: scan & push forward briefly")
    best = scan_free_direction(cap, W, H)
    angle = int(np.clip(SERVO_CENTER + best * (SERVO_MAX - SERVO_CENTER),
                        SERVO_MIN, SERVO_MAX))
    set_servo_angle(angle)
    time.sleep(0.06)
    motor_forward(BASE_SPEED)
    time.sleep(0.6)
    motor_stop()
    time.sleep(0.06)
    changed, new_gray = frame_changed()
    if changed:
        print("[RECOVER] moved after fallback scan")
        return True, new_gray, int(np.sign(best))
    print("[RECOVER] failed to recover")
    return False, prev_frame_gray, 0

# ---------------- MAIN ------------------------------------------------
def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

    prev_frame_gray = None
    diff_history = []  # list of (timestamp, diff_mean)
    last_action_time = 0.0

    # counters to learn preferred side
    left_turns = 0
    right_turns = 0

    try:
        print("Starting: single-ROI border-follow with robust recovery + learned preferred side.")
        motor_forward(BASE_SPEED)
        set_servo_angle(SERVO_CENTER)

        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            frame = cv2.resize(frame, (W, H))
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # ---------- frame-diff window update ----------
            if prev_frame_gray is not None:
                diff = cv2.absdiff(prev_frame_gray, frame_gray)
                diff_mean = float(np.mean(diff))
                now = time.time()
                diff_history.append((now, diff_mean))
                # prune history older than DIFF_WINDOW_SEC
                cutoff = now - DIFF_WINDOW_SEC
                while diff_history and diff_history[0][0] < cutoff:
                    diff_history.pop(0)

                # check motion in the window: if all diffs < MOTION_THRESHOLD => stuck
                recent_max = max([v for (_, v) in diff_history]) if diff_history else 9999.0
                if recent_max < MOTION_THRESHOLD:
                    print(f"[STUCK DETECT] recent_max={recent_max:.2f} < {MOTION_THRESHOLD} -> starting recovery")
                    motor_stop()
                    time.sleep(0.08)
                    # pick preferred side from history counts
                    preferred = None
                    if left_turns + right_turns > 0:
                        preferred = -1.0 if left_turns >= right_turns else 1.0
                    succeeded, new_gray, used_dir = recover_from_stuck(cap, prev_frame_gray, preferred_side=preferred)
                    if succeeded:
                        prev_frame_gray = new_gray.copy()
                        diff_history.clear()
                        last_action_time = time.time()
                        # increment counts if we used a turn direction
                        if used_dir < 0:
                            left_turns += 1
                        elif used_dir > 0:
                            right_turns += 1
                        # after recovery, resume forward
                        motor_forward(BASE_SPEED)
                        set_servo_angle(SERVO_CENTER)
                        # small pause to let things settle
                        time.sleep(0.08)
                        continue
                    else:
                        # failed recovery: small wait and continue loop (will try again later)
                        diff_history.clear()
                        time.sleep(0.2)
                        motor_forward(BASE_SPEED)
                # else not stuck
            # save prev
            prev_frame_gray = frame_gray.copy()

            # ---------- ROI & front-wall analysis ----------
            x1, y1, x2, y2 = build_roi_band(D_TURN_CM)
            roi = frame[y1:y2, x1:x2]
            fw, bbox, top_edge_row, ratio_all, ratio_bottom = analyze_front_wall(roi)

            # early ROI check (a bit closer)
            x1e, y1e, x2e, y2e = build_roi_band(EARLY_TURN_CM)
            roi_e = frame[y1e:y2e, x1e:x2e]
            fw_early, _, _, _, _ = analyze_front_wall(roi_e)

            # compute left/right free measure in bottom strip of main roi
            mask_main, _ = black_mask(roi)
            h_roi, w_roi = mask_main.shape
            bottom_h = min(24, h_roi)
            bs = mask_main[h_roi-bottom_h:h_roi, :]
            left_black = float(cv2.countNonZero(bs[:, :w_roi//2])) / (bs[:, :w_roi//2].size + 1e-8)
            right_black = float(cv2.countNonZero(bs[:, w_roi//2:])) / (bs[:, w_roi//2:].size + 1e-8)
            left_free = 1.0 - left_black
            right_free = 1.0 - right_black

            # ---------- decision logic ----------
            steer = 0.0
            decision = "forward"

            # If early or close, do timed reverse + scan (but avoid spamming)
            if fw_early or (fw and (estimate_distance_from_top_edge(y1, top_edge_row) <= D_STOP_CM)):
                if time.time() - last_action_time > (REVERSE_DURATION + 0.3):
                    print("[EARLY] front wall detected -> reverse & scan")
                    motor_stop(); time.sleep(0.08)
                    motor_backward(BASE_SPEED)
                    time.sleep(REVERSE_DURATION)
                    motor_stop()
                    # pick side by scanning
                    best = scan_free_direction(cap, W, H)
                    angle = int(np.clip(SERVO_CENTER + best * (SERVO_MAX - SERVO_CENTER),
                                        SERVO_MIN, SERVO_MAX))
                    set_servo_angle(angle)
                    motor_forward(BASE_SPEED)
                    decision = "turn"
                    steer = best
                    last_action_time = time.time()
                    # increment learned counters
                    if best < 0:
                        left_turns += 1
                    elif best > 0:
                        right_turns += 1
            else:
                # normal forward: small steering bias toward freer side
                diff_free = right_free - left_free
                steer = float(np.clip(diff_free * 1.6, -1.0, 1.0))

            # ---------- actuate ----------
            servo_angle = int(np.clip(SERVO_CENTER + steer * (SERVO_MAX - SERVO_CENTER),
                                      SERVO_MIN, SERVO_MAX))
            set_servo_angle(servo_angle)
            if decision == "forward":
                motor_forward(BASE_SPEED)

            # ---------- debug visuals ----------
            disp = frame.copy()
            cv2.rectangle(disp, (x1, y1), (x2, y2), (0, 255, 255), 2)   # main ROI
            cv2.rectangle(disp, (x1e, y1e), (x2e, y2e), (255, 0, 0), 2) # early ROI
            if fw and bbox is not None:
                bx, by, bw, bh = bbox
                cv2.rectangle(disp, (x1 + bx, y1 + by), (x1 + bx + bw, y1 + by + bh), (0, 255, 0), 2)
                cv2.line(disp, (x1, y1 + top_edge_row), (x2, y1 + top_edge_row), (0, 200, 0), 1)

            txt = (f"{decision} servo={servo_angle} steer={steer:+.2f} "
                   f"ratio={ratio_all:.3f} bottom={ratio_bottom:.3f} "
                   f"leftTurns={left_turns} rightTurns={right_turns}")
            cv2.putText(disp, txt, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("navigator", disp)
            cv2.imshow("roi_mask", mask_main)

            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        motor_stop()
        servo_pwm.stop()
        motor_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()


