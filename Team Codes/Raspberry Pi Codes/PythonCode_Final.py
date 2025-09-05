#====== WRO 2025 Future Engineers Team 1121 - Headless ======#

import glob, time, os
import serial
import RPi.GPIO as GPIO
import cv2
import numpy as np
import collections

# ---------- Config ----------
DEBUG = True                    # set False after tuning
CLOSE_ONLY = True               # when True -> ONLY send R/L when dist <= CLOSE_DIST
CLOSE_DIST = 15.0               # cm threshold for close-range R/L
STRAIGHT_PERCENT = 0.90         # 90% threshold (unused if CLOSE_ONLY True but left for future)
MIN_PIXELS_FOR_DECISION = 200   # minimal pixels to consider a detection
MORPH_KERNEL = np.ones((5,5), np.uint8)
TARGET_WIDTH = 320
TARGET_HEIGHT = 240
TARGET_FPS = 30

# Preferred devices
PREFER_DEVICES = ['/dev/video0', '/dev/video2']

# ====== HSV Ranges (user provided) ======
lower_range1 = np.array([149, 92, 110], dtype=np.uint8)   # RED (R)
upper_range1 = np.array([179, 255, 255], dtype=np.uint8)
lower_range2 = np.array([57, 44, 95], dtype=np.uint8)     # GREEN (L)
upper_range2 = np.array([99, 186, 154], dtype=np.uint8)
lower_range1L = np.array([6, 172, 125], dtype=np.uint8)   # ORANGE line (cam2)
upper_range1L = np.array([179, 255, 255], dtype=np.uint8)
lower_range2L = np.array([96, 61, 80], dtype=np.uint8)    # BLUE line (cam2)
upper_range2L = np.array([156, 255, 255], dtype=np.uint8)
lower_range_pink = np.array([140, 50, 70], dtype=np.uint8)
upper_range_pink = np.array([170, 255, 255], dtype=np.uint8)
lower_range_block = np.array([0,0,200], dtype=np.uint8)
upper_range_block = np.array([180,30,255], dtype=np.uint8)

# ---------- GPIO / Serial ----------
TRIG = 23
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1.0)
time.sleep(2)
ser.reset_input_buffer()
print("Serial Communication Established.")

# ---------- Camera helpers ----------
def list_video_devices():
    import glob
    return sorted(glob.glob('/dev/video*'))

def devpath_to_index(dev_path):
    try: return int(dev_path.replace('/dev/video',''))
    except: return None

def try_open_device(dev_path, backend=cv2.CAP_V4L2, width=TARGET_WIDTH, height=TARGET_HEIGHT, fps=TARGET_FPS, timeout=1.5):
    idx = devpath_to_index(dev_path)
    if idx is None: return None
    cap = cv2.VideoCapture(idx, backend)
    start = time.time()
    while time.time() - start < timeout:
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            ret, _ = cap.read()
            if ret: return cap
        time.sleep(0.12)
    try: cap.release()
    except: pass
    return None

def open_first_n_cameras(n=2, prefer_list=None):
    devices = list_video_devices()
    if prefer_list:
        pref = [d for d in prefer_list if d in devices]
        others = [d for d in devices if d not in pref]
        devices = pref + others
    opened = []
    for dev in devices:
        if len(opened) >= n: break
        cap = try_open_device(dev, backend=cv2.CAP_V4L2)
        if not cap: cap = try_open_device(dev, backend=cv2.CAP_ANY)
        if cap:
            opened.append((dev, cap)); print(f"[CAM] Opened {dev}")
        else:
            if DEBUG: print(f"[CAM] Failed to open {dev}")
    return opened

def reopen_camera_if_needed(cap_obj, dev_path):
    if cap_obj is None or not cap_obj.isOpened():
        if DEBUG: print(f"[CAM] Reopen attempt for {dev_path}")
        newcap = try_open_device(dev_path, backend=cv2.CAP_V4L2)
        if not newcap: newcap = try_open_device(dev_path, backend=cv2.CAP_ANY)
        return newcap
    return cap_obj

def morph_mask(mask):
    return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL),
                            cv2.MORPH_CLOSE, MORPH_KERNEL)

def get_distance(timeout = 0.02):
    GPIO.output(TRIG, True); time.sleep(0.00001); GPIO.output(TRIG, False)
    start_time = time.time(); stop_time = time.time()
    while GPIO.input(ECHO) == 0: start_time = time.time()
    while GPIO.input(ECHO) == 1: stop_time = time.time()
    time_elapsed = stop_time - start_time
    return (time_elapsed * 34300) / 2

# ---------- Open cameras ----------
opened = open_first_n_cameras(2, prefer_list=PREFER_DEVICES)
if len(opened) == 0:
    raise RuntimeError("No camera devices could be opened. Check /dev/video* and camera connections.")

if len(opened) == 1:
    cap_device, cap = opened[0]
    cap2_device, cap2 = (None, None)
    print(f"[CAM] Only one camera found: {cap_device}. Running single-camera mode.")
else:
    cap_device, cap = opened[0]
    cap2_device, cap2 = opened[1]
    print(f"[CAM] cap -> {cap_device} | cap2 -> {cap2_device}")

# ---------- Startup: single-shot detect the FIRST line (orange or blue) ----------
def detect_first_line_once(cap2, min_area=400, bottom_roi_frac=0.30, morph_kernel_small=np.ones((3,3), np.uint8)):
    """
    Single-shot: detect the closest-to-bottom orange or blue line in camera2.
    Returns: 'orange', 'blue', or None
    """
    if cap2 is None or not cap2.isOpened():
        if DEBUG: print("[STARTUP] cap2 not available")
        return None

    ret, frame = cap2.read()
    if not ret or frame is None:
        if DEBUG: print("[STARTUP] failed to read frame")
        return None

    frame = cv2.resize(frame, (TARGET_WIDTH, TARGET_HEIGHT))
    h, w = frame.shape[:2]

    roi_y = int(h * (1.0 - bottom_roi_frac))
    roi = frame[roi_y:h, 0:w]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask_orange = cv2.inRange(hsv, lower_range1L, upper_range1L)
    mask_blue   = cv2.inRange(hsv, lower_range2L, upper_range2L)

    # small morphology
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, morph_kernel_small)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, morph_kernel_small)
    mask_blue   = cv2.morphologyEx(mask_blue,   cv2.MORPH_OPEN, morph_kernel_small)
    mask_blue   = cv2.morphologyEx(mask_blue,   cv2.MORPH_CLOSE, morph_kernel_small)

    best_orange = None
    cnts_o, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts_o:
        if cv2.contourArea(c) < min_area: continue
        x,y,wc,hc = cv2.boundingRect(c)
        bottom_y = y + hc  # ROI coords; larger = closer to bottom
        if best_orange is None or bottom_y > best_orange[0]:
            best_orange = (bottom_y, (x,y,wc,hc))

    best_blue = None
    cnts_b, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts_b:
        if cv2.contourArea(c) < min_area: continue
        x,y,wc,hc = cv2.boundingRect(c)
        bottom_y = y + hc
        if best_blue is None or bottom_y > best_blue[0]:
            best_blue = (bottom_y, (x,y,wc,hc))

    if best_orange is None and best_blue is None:
        if DEBUG: print("[STARTUP] no orange/blue detected in ROI")
        return None

    # choose which contour is lower (closer to bottom of frame)
    if best_orange and best_blue:
        chosen = "orange" if best_orange[0] > best_blue[0] else "blue"
    elif best_orange:
        chosen = "orange"
    else:
        chosen = "blue"

    return chosen

# state
frame_times = collections.deque(maxlen=30)
last_line_color = None
last_detection_time = 0
lockout_time = 1.5
line_count = 0
startup_sent = False   # will be True once OR/BL is sent at startup

# --- Run single-shot startup detection (only ONCE before the loop) ---
if cap2 is not None:
    first = detect_first_line_once(cap2, min_area=300, bottom_roi_frac=0.30)
    if first == "orange":
        try:
            ser.write("OR\n".encode('utf-8'))   # OR for orange
            if DEBUG: print("[STARTUP] Sent: OR (orange)")
        except Exception as e:
            if DEBUG: print(f"[STARTUP] Serial write failed for OR: {e}")
        last_line_color = "orange"
        last_detection_time = time.time()
        startup_sent = True
    elif first == "blue":
        try:
            ser.write("BL\n".encode('utf-8'))   # BL for blue
            if DEBUG: print("[STARTUP] Sent: BL (blue)")
        except Exception as e:
            if DEBUG: print(f"[STARTUP] Serial write failed for BL: {e}")
        last_line_color = "blue"
        last_detection_time = time.time()
        startup_sent = True
    else:
        if DEBUG: print("[STARTUP] No line detected on startup (single-shot).")
else:
    if DEBUG: print("[STARTUP] cap2 not present — skipping first-line detection.")

# Send setup-done 'S' once
try:
    ser.write("S\n".encode('utf-8'))
    if DEBUG: print("[STARTUP] Sent: S (setup done)")
except Exception as e:
    if DEBUG: print(f"[STARTUP] Failed to send 'S': {e}")

try:
    while True:
        # ensure cameras alive
        cap = reopen_camera_if_needed(cap, cap_device)
        cap2 = reopen_camera_if_needed(cap2, cap2_device) if cap2_device else None

        # ultrasonic
        dist = get_distance()
        if DEBUG: print(f"[ULTRA] Distance: {dist:.2f} cm")

        # frames
        ret1, frame1 = (False, None)
        ret2, frame2 = (False, None)
        if cap: ret1, frame1 = cap.read()
        if cap2: ret2, frame2 = cap2.read()

        if not ret1:
            if DEBUG: print(f"[CAM] frame1 read failed from {cap_device}, attempting reopen")
            cap = reopen_camera_if_needed(cap, cap_device)
            if cap: ret1, frame1 = cap.read()
        if not ret2 and cap2_device:
            if DEBUG: print(f"[CAM] frame2 read failed from {cap2_device}, attempting reopen")
            cap2 = reopen_camera_if_needed(cap2, cap2_device)
            if cap2: ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            if DEBUG: print("Camera read failed (ret1/ret2):", ret1, ret2)
            time.sleep(0.08); continue

        frame1 = cv2.resize(frame1, (TARGET_WIDTH, TARGET_HEIGHT))
        frame2 = cv2.resize(frame2, (TARGET_WIDTH, TARGET_HEIGHT))

        hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # masks
        mask_red = morph_mask(cv2.inRange(hsv1, lower_range1, upper_range1))
        mask_green = morph_mask(cv2.inRange(hsv1, lower_range2, upper_range2))
        mask_block = morph_mask(cv2.inRange(hsv1, lower_range_block, upper_range_block))

        mask_orange = morph_mask(cv2.inRange(hsv2, lower_range1L, upper_range1L))
        mask_blue   = morph_mask(cv2.inRange(hsv2, lower_range2L, upper_range2L))
        mask_pink   = morph_mask(cv2.inRange(hsv2, lower_range_pink, upper_range_pink))

        # pixel counts
        red_total = int(cv2.countNonZero(mask_red))
        green_total = int(cv2.countNonZero(mask_green))
        block_total = int(cv2.countNonZero(mask_block))
        orange_total = int(cv2.countNonZero(mask_orange))
        blue_total = int(cv2.countNonZero(mask_blue))
        pink_total = int(cv2.countNonZero(mask_pink))

        if DEBUG:
            print(f"[MASKS] red={red_total} green={green_total} block={block_total} orange={orange_total} blue={blue_total} pink={pink_total}")

        # ---------- CLOSE_ONLY behavior ----------
        if CLOSE_ONLY:
            # Only send R or L when color present AND dist <= CLOSE_DIST
            if dist <= CLOSE_DIST:
                if red_total >= MIN_PIXELS_FOR_DECISION:
                    ser.write("R\n".encode('utf-8'))
                    if DEBUG: print("Sent serial: R (Red close)")
                elif green_total >= MIN_PIXELS_FOR_DECISION:
                    ser.write("L\n".encode('utf-8'))
                    if DEBUG: print("Sent serial: L (Green close)")
                # do not send any other protocol messages in CLOSE_ONLY mode
            else:
                # not close enough — send nothing (no spam)
                if DEBUG: print("Not within close range; no serial sent (CLOSE_ONLY mode).")
        else:
            # Original behavior: send other commands as before

            # --- line detection (camera2)
            # We still detect colors for sequence logic, but DO NOT resend the startup OR/BL if startup_sent is True.
            current_time = time.time()
            current_line_color = None
            if orange_total >= MIN_PIXELS_FOR_DECISION: current_line_color = "orange"
            if blue_total >= MIN_PIXELS_FOR_DECISION: current_line_color = "blue"

            if current_line_color and (current_time - last_detection_time >= lockout_time):
                # send O/BL only if we DID NOT already send startup OR/BL
                if not startup_sent:
                    if current_line_color == "orange":
                        ser.write("O\n".encode('utf-8'))
                        if DEBUG: print("Sent serial: O  (Orange detected)")
                    elif current_line_color == "blue":
                        ser.write("BL\n".encode('utf-8'))
                        if DEBUG: print("Sent serial: BL (Blue detected)")
                # sequence logic still works the same
                if last_line_color is None:
                    last_line_color = current_line_color
                    last_detection_time = current_time
                else:
                    if last_line_color == "orange" and current_line_color == "blue":
                        ser.write("C\n".encode('utf-8')); line_count += 1
                        if DEBUG: print("Sent serial: C (Orange->Blue)")
                    elif last_line_color == "blue" and current_line_color == "orange":
                        ser.write("A\n".encode('utf-8')); line_count += 1
                        if DEBUG: print("Sent serial: A (Blue->Orange)")
                    else:
                        if DEBUG: print(f"Sequence ignored: {last_line_color}->{current_line_color}")

                    if line_count == 12:
                        ser.write("P\n".encode('utf-8'))
                        if DEBUG: print("Sent serial: P (Parking)")

                    last_line_color = None
                    last_detection_time = current_time

            # pink left/right
            cnts_pink, _ = cv2.findContours(mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            left_count_pink = right_count_pink = 0
            for c in cnts_pink:
                if cv2.contourArea(c) > 500:
                    x, y, w, h = cv2.boundingRect(c)
                    cx = x + w//2
                    if cx < frame2.shape[1]//2: left_count_pink += 1
                    else: right_count_pink += 1
            if left_count_pink > right_count_pink and left_count_pink > 0:
                ser.write("PL\n".encode('utf-8'));
                if DEBUG: print(f"Sent serial: PL (Pink Left) L={left_count_pink} R={right_count_pink}")
            elif right_count_pink > left_count_pink and right_count_pink > 0:
                ser.write("PR\n".encode('utf-8'));
                if DEBUG: print(f"Sent serial: PR (Pink Right) L={left_count_pink} R={right_count_pink}")

            # straight logic (pixel percentage)
            width1 = frame1.shape[1]
            red_left_pixels = int(cv2.countNonZero(mask_red[:, :width1//2]))
            red_right_pixels = int(cv2.countNonZero(mask_red[:, width1//2:]))
            red_total_check = red_left_pixels + red_right_pixels

            green_left_pixels = int(cv2.countNonZero(mask_green[:, :width1//2]))
            green_right_pixels = int(cv2.countNonZero(mask_green[:, width1//2:]))
            green_total_check = green_left_pixels + green_right_pixels

            if DEBUG:
                print(f"[PIXEL_SPLIT] red L={red_left_pixels} R={red_right_pixels} total={red_total_check} | green L={green_left_pixels} R={green_right_pixels} total={green_total_check}")

            if red_total_check >= MIN_PIXELS_FOR_DECISION:
                pct_left = red_left_pixels / float(red_total_check)
                if pct_left >= STRAIGHT_PERCENT:
                    ser.write("SR\n".encode('utf-8'))
                    if DEBUG: print(f"Sent serial: SR (Red->Right). red_total={red_total_check} left%={pct_left:.2f}")

            if green_total_check >= MIN_PIXELS_FOR_DECISION:
                pct_right = green_right_pixels / float(green_total_check)
                if pct_right >= STRAIGHT_PERCENT:
                    ser.write("SL\n".encode('utf-8'))
                    if DEBUG: print(f"Sent serial: SL (Green->Left). green_total={green_total_check} right%={pct_right:.2f}")

            # short-range R/L & border B as before
            if dist <= 15:
                if red_total >= MIN_PIXELS_FOR_DECISION:
                    ser.write("R\n".encode('utf-8'));
                    if DEBUG: print("Sent serial: R (Red close)")
                elif green_total >= MIN_PIXELS_FOR_DECISION:
                    ser.write("L\n".encode('utf-8'));
                    if DEBUG: print("Sent serial: L (Green close)")

            if dist <= 12.5 and block_total >= MIN_PIXELS_FOR_DECISION:
                ser.write("B\n".encode('utf-8'))
                if DEBUG: print("Sent serial: B (Border very close)")

        # pacing / fps
        frame_times.append(time.time())
        if len(frame_times) >= 2 and DEBUG and int(time.time()) % 5 == 0:
            fps = len(frame_times) / (frame_times[-1] - frame_times[0])
            print(f"Estimated loop FPS: {fps:.2f}")

        time.sleep(0.033)

except KeyboardInterrupt:
    print("Interrupted — cleaning up...")

finally:
    GPIO.cleanup()
    try: ser.close()
    except: pass
    try: cap.release()
    except: pass
    try: cap2.release()
    except: pass
    cv2.destroyAllWindows()
    print("Cleanup done. Exiting.")
