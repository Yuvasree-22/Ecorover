"""EcoRover: YOLO waste detection, target alignment, ultrasonic approach and arm pickup."""
from pathlib import Path
import time
import re
import cv2
import serial
from ultralytics import YOLO

# ---------------- USER CONFIGURATION ----------------
MODEL_PATH = "best.pt"                 # Trained YOLO weights
CAMERA_INDEX = 0                       # USB camera index
SERIAL_PORT = "/dev/ttyACM0"           # Arduino USB serial port
BAUD_RATE = 115200
CONFIDENCE = 0.55
WASTE_CLASSES = {"plastic_bottle", "plastic", "paper", "can", "metal", "waste"}

APPROACH_SPEED = 45                    # Arduino speed range: -100 to 100
PICKUP_DISTANCE_CM = 25.0
TARGET_CENTER_TOLERANCE = 0.18         # Fraction of image width
LOST_TARGET_STOP_FRAMES = 2
PICKUP_COOLDOWN_SECONDS = 4.0
LOOP_DELAY = 0.04
# ----------------------------------------------------


def send(board, command):
    board.write((command.strip() + "\n").encode("ascii"))
    board.flush()


def read_arduino(board):
    distance = None
    while board.in_waiting:
        line = board.readline().decode("utf-8", errors="ignore").strip()
        match = re.search(r"DISTANCE_CM=([0-9]+(?:\.[0-9]+)?)", line)
        if match:
            distance = float(match.group(1))
        elif "DISTANCE_CM=INVALID" in line:
            distance = None
        elif line:
            print("Arduino:", line)
    return distance


def find_target(result, model):
    if result.boxes is None:
        return None
    width = result.orig_shape[1]
    options = []
    for box in result.boxes:
        class_id = int(box.cls[0].item())
        label = str(model.names[class_id]).lower().strip().replace(" ", "_")
        score = float(box.conf[0].item())
        if label not in WASTE_CLASSES:
            continue
        x1, _, x2, _ = box.xyxy[0].tolist()
        center = ((x1 + x2) / 2.0) / width
        options.append((score, label, center))
    if not options:
        return None
    score, label, center = max(options, key=lambda item: item[0])
    return label, center, score


def main():
    if not Path(MODEL_PATH).is_file():
        raise FileNotFoundError(f"YOLO weights not found: {MODEL_PATH}")

    model = YOLO(MODEL_PATH)
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("USB camera could not be opened")

    board = None
    lost_frames = 0
    last_pickup = 0.0
    last_distance = None
    try:
        board = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(2.0)  # Arduino may reset when serial opens
        send(board, "STOP")
        print("EcoRover autonomous mode started. Press q to stop.")

        while True:
            ok, frame = camera.read()
            if not ok:
                send(board, "STOP")
                raise RuntimeError("Camera frame unavailable; motors stopped")

            result = model.predict(frame, conf=CONFIDENCE, verbose=False)[0]
            distance_reading = read_arduino(board)
            if distance_reading is not None:
                last_distance = distance_reading

            target = find_target(result, model)
            annotated = result.plot()
            status = "Searching for waste"

            if target is None:
                lost_frames += 1
                if lost_frames >= LOST_TARGET_STOP_FRAMES:
                    send(board, "STOP")
                status = "No waste detected - stopped"
            else:
                lost_frames = 0
                label, center_x, score = target
                status = f"{label} ({score:.2f})"

                # Require a recent valid ultrasonic reading before moving.
                if last_distance is None or last_distance <= 0:
                    send(board, "STOP")
                    status += " | waiting for distance"
                elif last_distance <= PICKUP_DISTANCE_CM:
                    send(board, "STOP")
                    status += f" | stopped at {last_distance:.1f} cm"
                    if time.monotonic() - last_pickup >= PICKUP_COOLDOWN_SECONDS:
                        send(board, "PICK")
                        last_pickup = time.monotonic()
                        status += " | pickup command sent"
                elif abs(center_x - 0.5) > TARGET_CENTER_TOLERANCE:
                    if center_x < 0.5:
                        send(board, f"MOVE {-APPROACH_SPEED} {APPROACH_SPEED}")
                        status += " | aligning left"
                    else:
                        send(board, f"MOVE {APPROACH_SPEED} {-APPROACH_SPEED}")
                        status += " | aligning right"
                else:
                    send(board, f"MOVE {APPROACH_SPEED} {APPROACH_SPEED}")
                    status += f" | approaching ({last_distance:.1f} cm)"

            cv2.putText(annotated, status[:100], (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("EcoRover - Autonomous Waste Detection", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(LOOP_DELAY)

    finally:
        if board is not None and board.is_open:
            try:
                send(board, "STOP")
                board.close()
            except serial.SerialException:
                pass
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
