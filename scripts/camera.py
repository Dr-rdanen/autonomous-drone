import cv2
import threading
from pymavlink import mavutil
import time, math

# === CAMERA THREAD ===
def camera_stream():
    gst_pipeline = (
        "udpsrc port=5600 caps=application/x-rtp,encoding-name=H264 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
    )
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open camera stream")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No frame received")
            break

        cv2.imshow("Iris Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Start camera in parallel
threading.Thread(target=camera_stream, daemon=True).start()

# === DRONE CONTROL CODE ===
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(f"Heartbeat from system ({master.target_system} component {master.target_component})")

start_time = time.time()

def set_guided():
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        master.set_mode('GUIDED')
        if heartbeat and heartbeat.custom_mode == 4:
            print("Mode set to GUIDED!")
            break
        time.sleep(0.5)

def arm():
    print("waiting for pre-arm and arming drone")
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        master.arducopter_arm()
        if heartbeat and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed!")
            break
        time.sleep(0.5)

def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print("Taking off...")

def send_position(x, y, z, duration, yaw=0):
    end_time = time.time() + duration
    v_x = x / duration
    v_y = y / duration
    v_z = z / duration
    while time.time() < end_time:
        master.mav.set_position_target_local_ned_send(
            int((time.time() - start_time) * 1000),
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            x, y, z,
            v_x, v_y, v_z,
            0, 0, 0,
            yaw, 0
        )

def turn_yaw(angle, speed, direction, relative=True):
    print("Turning")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, angle, speed, direction,
        1 if relative else 0,
        0, 0, 0
    )

def land():
    print("Landing")
    master.set_mode('LAND')

# === Mission ===
set_guided()
time.sleep(2)
arm()
master.motors_armed_wait()
takeoff(5)
time.sleep(5)
print("Moving 5m")
send_position(5, 0, 0, 5)
time.sleep(6)
print("Turning right")
turn_yaw(90, 30, 0)
time.sleep(5)
print("Moving 4m")
send_position(4, 0, 0, 2)
time.sleep(6)
land()
master.motors_disarmed_wait()
print("Landed and disarmed")
