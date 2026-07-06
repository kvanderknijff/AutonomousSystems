"""Overhead camera: ArUco tracking, LED linking, MQTT position publish."""
import cv2
import json
import math
import sys
import numpy as np
from paho.mqtt import client as mqtt_client

# USB webcam (Logitech HD 1080p). Set explicitly if auto-detect picks the wrong device.
# For IP Webcam on a phone, set USE_USB_WEBCAM = False and cameraSource to e.g.
# "http://<phone-ip>:8880/video"
USE_USB_WEBCAM = True
CAMERA_DEVICE_INDEX = 1  # None = auto-detect 1080p USB camera; or set 0, 1, ...
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
cameraSource = "http://145.137.61.30:8880/video"
debugType = "linking"
"""
debugType: which video do you want to see
    - "arucos": Show the detection of ArUco markers along with its information, orientation and area for LED linking
    - "leds": Show the detection of leds
    - "linking": Show which ArUco markers are linked to which leds
"""

FirstChariotMarkerID = 1
LastChariotMarkerID = 4
FirstCornerMarkerID = 5
LastCornerMarkerID = 8

# Printed robot markers use DICT_6X6_50 (not 4x4). Set to cv2.aruco.DICT_4X4_50 if needed.
ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50
# Phone IP webcam used 90° rotation; USB overhead mount usually needs none.
FRAME_ROTATION = None  # e.g. cv2.ROTATE_90_CLOCKWISE

dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
arucoParams = cv2.aruco.DetectorParameters()
arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(dictionary, arucoParams)

maxAllowedDistanceMarkerToLed = 100 #65
ledSearchAngle = 120

def on_connect(client, userdata, flags, rc) -> None:
    if rc == 0:
        client.subscribe(mqttTopic)
        print("Subscribed to mqtt topic: " + mqttTopic)
    else:
        print("Couldn't connect to mqtt topic")

mqttClientId = "Camera"
mqttPort = 8883
mqttBroker = "145.24.237.88"
mqttTopic = "Robots/Data/Positions/Physical"
client = mqtt_client.Client(client_id=mqttClientId)
client.username_pw_set(username="myuser", password="FormingFormsAS")
client.on_connect = on_connect

def arUcoDetection(frame: np.ndarray) -> tuple[list, list, np.ndarray]:
    display = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, markerIDs, _rejected = detector.detectMarkers(gray)
    if markerIDs is not None:
        cv2.aruco.drawDetectedMarkers(display, corners, markerIDs)

    chariotArucoInformation = []
    cornerArucoInformation = []

    if markerIDs is not None:
        for corner, markerID in zip(corners, markerIDs):
            marker_id = int(markerID[0])
            corner = corner.reshape(4, 2).astype(int)

            topLeft, topRight, bottomRight, bottomLeft = corner

            cv2.putText(display, f"id: {marker_id}", tuple(topLeft), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 0, 255), 2)

            centerX = (topLeft[0] + bottomRight[0]) // 2
            centerY = (topLeft[1] + bottomRight[1]) // 2

            center = (centerX, centerY)
            
            if FirstChariotMarkerID <= marker_id <= LastChariotMarkerID:
                cv2.circle(display, center, 5, (0, 0, 255), -1)

                topMiddleX = (topLeft[0] + topRight[0]) // 2
                topMiddleY = (topLeft[1] + topRight[1]) // 2

                direction = math.degrees(math.atan2(topMiddleY - centerY, topMiddleX - centerX))
                cv2.putText(display, f"dir: {direction:.0f}", tuple(topRight), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 0, 255), 2)            
                
                chariotArucoInformation.append([marker_id, center, direction])
            elif FirstCornerMarkerID <= marker_id <= LastCornerMarkerID:
                cornerArucoInformation.append([marker_id, center])
                cv2.line(display, topLeft, bottomLeft, (0, 255, 0), 2)
                cv2.line(display, bottomLeft, bottomRight, (0, 255, 0), 2)
                cv2.line(display, bottomRight, topRight, (0, 255, 0), 2)
                cv2.line(display, topLeft, topRight, (0, 255, 0), 2)

    for marker in chariotArucoInformation:
        cv2.circle(display, marker[1], maxAllowedDistanceMarkerToLed, (0, 0, 0), 2)

    return chariotArucoInformation, cornerArucoInformation, display

def detect_pix(frame: np.ndarray, frameRGB: np.ndarray, colorCode: tuple, method: int, minimumLedArea: int) -> tuple[list, np.ndarray]:
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam for course TINLAS03-2025-VT-JAAR
    """
    ledPositions = []

    contours, hierarchy = cv2.findContours(frame, method, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > minimumLedArea:
            x, y, w, h = cv2.boundingRect(contour)
            ledPositions.append([x + 0.5 * w, y + 0.5 * h])
            cv2.rectangle(frameRGB, (x, y), (x + w, y + h), colorCode, 2)

    if not ledPositions:
        ledPositions.append([-1,-1])

    return ledPositions, frameRGB

def ledDetection(frameBGR: np.ndarray) -> tuple[list, np.ndarray]:
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam for course TINLAS03-2025-VT-JAAR
    """
    ledPositions = []

    frameRGB = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)
    frameHSV = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)

    frameR, frameG, frameB = cv2.split(frameRGB)
    frameBG = cv2.subtract(frameB, frameG)

    ret, frameBG = cv2.threshold(frameBG, 40, 255, cv2.THRESH_BINARY)
    blueLedPositions, frameRGB = detect_pix(frameBG, frameRGB, (0, 0, 255), cv2.RETR_TREE, 150)
    ledPositions.append(blueLedPositions)

    lowerGreen = np.array([35, 40, 40])
    upperGreen = np.array([95, 255, 255])
    maskG = cv2.inRange(frameHSV, lowerGreen, upperGreen)
    greenLedPositions, frameRGB = detect_pix(maskG, frameRGB, (0, 255, 0), cv2.RETR_EXTERNAL, 40)
    ledPositions.append(greenLedPositions)

    outputFrame = cv2.cvtColor(frameRGB, cv2.COLOR_RGB2BGR)
    return ledPositions, outputFrame

def linkLedToChariot(arUcoInformation: list, ledPositions: list, frame: np.ndarray) -> tuple[list, np.ndarray]:
    """
    Assuming two ArUco's are not directly next to eachother
    """
    chariotInformation = []
    textHeight = 40

    for chariot in arUcoInformation:
        status = "Off"
        bestDistance = maxAllowedDistanceMarkerToLed + 1
        textHeight += 40
        
        chariotDirection = ((chariot[2] - 90 + 180) % 360) - 180

        x = int(chariot[1][0] + maxAllowedDistanceMarkerToLed * math.cos(math.radians(chariotDirection + ledSearchAngle)))
        y = int(chariot[1][1] + maxAllowedDistanceMarkerToLed * math.sin(math.radians(chariotDirection + ledSearchAngle)))
        cv2.line(frame, chariot[1], (x, y), (0, 255, 255), 2)
        x = int(chariot[1][0] + maxAllowedDistanceMarkerToLed * math.cos(math.radians(chariotDirection - ledSearchAngle)))
        y = int(chariot[1][1] + maxAllowedDistanceMarkerToLed * math.sin(math.radians(chariotDirection - ledSearchAngle)))
        cv2.line(frame, chariot[1], (x, y), (0, 255, 255), 2)
        cv2.circle(frame, chariot[1], maxAllowedDistanceMarkerToLed, (0, 255, 255), 2)

        for colorIndex, ledColor in enumerate(ledPositions):
            for led in ledColor:
                if led == (-1,-1):
                    continue

                distance = math.dist(chariot[1], led)
                angle = math.degrees(math.atan2(led[1] - chariot[1][1], led[0] - chariot[1][0]))
                angleDifference = abs((angle - chariotDirection + 180) % 360 - 180)

                if distance < bestDistance and angleDifference >= ledSearchAngle:
                    if colorIndex == 0: # Blue LED
                        status = "Connecting"
                    elif colorIndex == 1: # Green LED
                        status = "Connected"
                    bestDistance = distance

        chariotInformation.append([chariot[0], chariot[1], chariot[2], status])
        cv2.putText(frame, f"Chariot {chariot[0]}: {status}", (40, textHeight), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    return chariotInformation, frame

def sendChariotInformation(chariotInformation: list) -> None:
    for chariot in chariotInformation:
        message = {
            "ArUco_ID": int(chariot[0]), 
            "x_position": int(chariot[1][0]),
            "y_position": int(chariot[1][1]),
            "orientation": float(chariot[2]),
            "led_status": str(chariot[3])
        }
        message_json = json.dumps(message)
        client.publish(mqttTopic, message_json)

def sendCornerInformation(cornerInformation: list) -> None:
    for corner in cornerInformation:
        message = {
            "ArUco_ID": int(corner[0]),
            "x_position": int(corner[1][0]),
            "y_position": int(corner[1][1]),
            "marker_type": "corner",
        }
        message_json = json.dumps(message)
        client.publish(mqttTopic, message_json)

def find_webcam_device(max_devices: int = 6) -> int | None:
    """Pick the camera that reaches the target resolution (USB 1080p over built-in)."""
    api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    best_index: int | None = None
    best_pixels = 0

    for index in range(max_devices):
        capture = cv2.VideoCapture(index, api)
        if not capture.isOpened():
            continue

        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        pixels = width * height

        if pixels > best_pixels:
            best_pixels = pixels
            best_index = index

        capture.release()

    return best_index


def resolve_camera_source() -> str | int:
    if not USE_USB_WEBCAM:
        return cameraSource

    if CAMERA_DEVICE_INDEX is not None:
        return CAMERA_DEVICE_INDEX

    detected = find_webcam_device()
    if detected is None:
        raise RuntimeError("No USB webcam found. Check the connection or set CAMERA_DEVICE_INDEX manually.")

    print(f"Auto-selected webcam device {detected} ({CAMERA_WIDTH}x{CAMERA_HEIGHT} target)")
    return detected


def open_capture(source: str | int) -> cv2.VideoCapture:
    if isinstance(source, int):
        api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
        capture = cv2.VideoCapture(source, api)
        # MJPEG is required on many Logitech webcams to reach 1080p over USB.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, 30)
    else:
        capture = cv2.VideoCapture(source)

    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def videoProcessing(source: str | int, record: bool) -> None:
    capture = open_capture(source)

    if not capture.isOpened():
        print("Error: Could not open video source:", source)
        return

    if isinstance(source, int):
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Webcam opened: device {source}, resolution {width}x{height}")

    if record:
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if FRAME_ROTATION in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
            width, height = height, width

        fps = capture.get(cv2.CAP_PROP_FPS)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))

    while True:
        try:
            ret, frame = capture.read()

            if not ret:
                print("End of video")
                break

            if FRAME_ROTATION is not None:
                frame = cv2.rotate(frame, FRAME_ROTATION)

            arUcoInformation, cornerInformation, arUcoFrame = arUcoDetection(frame)
            ledPositions, ledFrame = ledDetection(frame)
            chariotInformation, linkingFrame = linkLedToChariot(arUcoInformation, ledPositions, frame)
            
            if chariotInformation:
                sendChariotInformation(chariotInformation)
            else:
                print("No chariot information to send")
            if cornerInformation:
                sendCornerInformation(cornerInformation)
            else:
                print("No corner information to send")

            if debugType == "arucos":
                cv2.imshow("Frames", arUcoFrame)
                if record:
                    out.write(arUcoFrame)
            elif debugType == "leds":
                cv2.imshow("Frames", ledFrame)
                if record:
                    out.write(ledFrame)
            elif debugType == "linking":
                cv2.imshow("Frames", linkingFrame)
                if record:
                    out.write(linkingFrame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print("Program ran into an exception: ", e)

    capture.release()
    if record:
        out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    client.connect(mqttBroker, mqttPort)
    client.loop_start()
    """
    videoProcessing() parameters:
    - record: do you want to save the shown frames to output.mp4
        - Yes: True
        - No: False
    """
    source = resolve_camera_source()
    videoProcessing(source, record=True)