#====== WRO 2025 Future Engineers Team 1121 - Raspberry Pi Code ======#  

#====== Main Imports ======#  
import serial
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
import collections  
import os   
'''
#====== Create Desktop File - Test ======#c
desktop_path = os.path.expanduser("~/Desktop")
file_name = "brotein_itworks"
file_path = os.path.join(desktop_path, file_name)
with open(file_path, "w") as f:
    f.write("It works!\n")
print(f"Created file on Desktop: {file_path}")
'''

#====== Color Detection - HSV Ranges ======#
#Camera 1 - Blocks
lower_range1 = np.array([])     #Color 1 - RED (R)
upper_range1 = np.array([])

lower_range2 = np.array([])     #Color 2 - GREEN (L)
upper_range2 = np.array([])

lower_range_pink = np.array([]) #Color 3 - PINK
upper_range_pink = np.array([])

#FAILSAFE BLOCK
lower_range_block = np.array([])
upper_range_block = np.array([])

#Camera 2 - Lines
lower_range1L = np.array([])   #Color 3 - Orange Line
upper_range1L = np.array([])

lower_range2L = np.array([])    #Color 4 - Blue Line
upper_range2L = np.array([])

#===== GPIO Setup for Ultrasonic Sensor =====#
TRIG = 23 
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

#===== Serial Setup =====#
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial Communication Established.")

#===== Camera Setup =====#
cap = cv2.VideoCapture(0) #Logi 
cap.set(cv2.CAP_PROP_FPS, 30)
cap2 = cv2.VideoCapture(2) #HIV
cap2.set(cv2.CAP_PROP_FPS, 30)

#===== Ultrasonic Distance Function =====#
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    return distance
frame_times = collections.deque(maxlen=30)

#===== Line Detection Sequence Variables =====#
last_line_color = None
last_detection_time = 0
lockout_time = 1.5   #seconds
line_count = 0

#====== Main Loop ======#
try:
    while True:
        #===== Ultrasonic Comms =====#
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")
        
        #===== Color Detection Main Loop =====#
        ret1, frame1 = cap.read()
        ret2, frame2 = cap2.read()
        if not ret1 or not ret2:
            break
        
        frame1 = cv2.resize(frame1, (640, 480))
        frame2 = cv2.resize(frame2, (640, 480))

        hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv1, lower_range1, upper_range1) #Blocks
        mask2 = cv2.inRange(hsv1, lower_range2, upper_range2)

        mask5 = cv2.inRange(hsv1, lower_range_block, upper_range_block) #Border

        mask3 = cv2.inRange(hsv2, lower_range1L, upper_range1L) #Lines
        mask4 = cv2.inRange(hsv2, lower_range2L, upper_range2L)

        # NEW: Pink mask on Camera 2
        mask_pink = cv2.inRange(hsv2, lower_range_pink, upper_range_pink)

        #Color Comms Variable
        color_detected = None
        border_detected = False
        
        #===== Block Detection =====#
        cnts1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts1:
            if cv2.contourArea(c) > 600:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(frame1, "COLOR 1 DETECTED", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                color_detected = 1
                
        cnts2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts2:
            if cv2.contourArea(c) > 600:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame1, "COLOR 2 DETECTED", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                color_detected = 2

        #===== Failsafe Wall detection =====#
        cnts5, _ = cv2.findContours(mask5, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts5:
            if cv2.contourArea(c) > 600:
                cv2.putText(frame2, "BORDER DETECTED", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0,0), 2)
                border_detected = True

        
        #===== Line Detection with Lockout & Sequence ======
        current_time = time.time()
        current_line_color = None

        # Orange
        cnts3, _ = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts3:
            if cv2.contourArea(c) > 600:
                current_line_color = "orange"
                cv2.putText(frame2, "ORANGE DETECTED", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,140,255), 2)

        # Blue
        cnts4, _ = cv2.findContours(mask4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts4:
            if cv2.contourArea(c) > 600:
                current_line_color = "blue"
                cv2.putText(frame2, "BLUE DETECTED", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

        # Process if new line color detected & lockout expired
        if current_line_color and (current_time - last_detection_time >= lockout_time):

            if current_line_color == "orange":
                ser.write("O\n".encode('utf-8'))
                print("Sent: O (Orange detected)")
            elif current_line_color == "blue":
                ser.write("BL\n".encode('utf-8'))
                print("Sent: BL (Blue detected)")

            if last_line_color is None:
                # First detection in sequence
                last_line_color = current_line_color
                last_detection_time = current_time
                print(f"First line detected: {last_line_color}")
            else:
                # Second detection in sequence
                if last_line_color == "orange" and current_line_color == "blue":
                    ser.write("C\n".encode('utf-8'))
                    print("Sent: C (Orange -> Blue)")
                    line_count += 1
                elif last_line_color == "blue" and current_line_color == "orange":
                    ser.write("A\n".encode('utf-8'))
                    print("Sent: A (Blue -> Orange)")
                    line_count += 1
                else:
                    print(f"Failsafe: Same color {last_line_color} -> {current_line_color}, waiting again")

                # Parking trigger
                if line_count == 12:
                    ser.write("P\n".encode('utf-8'))
                    print("Sent: P")

                # Reset sequence
                last_line_color = None
                last_detection_time = current_time

        #===== Pink Detection Left / Right =====#
        cnts_pink, _ = cv2.findContours(mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        left_count, right_count = 0, 0
        for c in cnts_pink:
            if cv2.contourArea(c) > 500:
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                if cx < frame2.shape[1] // 2:
                    left_count += 1
                else:
                    right_count += 1
                cv2.rectangle(frame2, (x, y), (x+w, y+h), (255, 0, 255), 2)

        if left_count > right_count and (left_count > 0):
            ser.write("PL\n".encode('utf-8'))
            print("Sent: PL (Pink Left)")
        elif right_count > left_count and (right_count > 0):
            ser.write("PR\n".encode('utf-8'))
            print("Sent: PR (Pink Right)")
     
        #====== Color & Ultrasonic Comms ======#
        if dist <= 15:
            if color_detected == 1:
                ser.write("R\n".encode('utf-8'))
                print("Sent: R")
            
            elif color_detected == 2:
                ser.write("L\n".encode('utf-8'))
                print("Sent: L")
        
        if dist <= 5:
            if border_detected == True:
                ser.write("B\n".encode('utf-8'))
                print("Sent: B")
            
        #====== FPS Counter ======#
        frame_times.append(time.time())
        if len(frame_times) >= 2:
            fps = len(frame_times) / (frame_times[-1] - frame_times[0])
            cv2.putText(frame1, f"FPS: {fps:.2f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame2, f"FPS: {fps:.2f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        
        cv2.imshow("FRAME", frame1)
        cv2.imshow("FRAME2", frame2)
        if cv2.waitKey(1) & 0xFF == 27:
            break
        time.sleep(0.033)  #Small delay to target 30 FPS

        
except KeyboardInterrupt:
    print("Cleaning up...")
finally:
    GPIO.cleanup()
    ser.close()
    cap.release()
    cap2.release()
    cv2.destroyAllWindows()

#Made with love bro
