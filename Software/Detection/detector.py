import cv2
import math
import numpy as np
from paho.mqtt import client as mqtt_client
import json

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(dictionary)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(mqttTopic)
        print("Subscribed to mqtt topic: " + mqttTopic)
    else:
        print("Couldn't connect to mqtt topic")

mqttClientId = "Camera"
mqttPort = 8883
mqttBroker = "145.24.237.88"
mqttTopic = "Robots/Data/Positions"
client = mqtt_client.Client(client_id=mqttClientId)
client.username_pw_set(username="myuser", password="FormingFormsAS")
client.on_connect = on_connect

def arUcoDetection(frame: np.ndarray):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, markerIDs, rejected = detector.detectMarkers(frame)
    
    information = []

    if markerIDs is not None:
        for corner, markerID in zip(corners, markerIDs):
            corner = corner.reshape(4, 2)
            corner = corner.astype(int)

            topLeft, topRight, bottomRight, bottomLeft = corner

            cv2.putText(frame, f"id: {markerID[0]}", tuple(topLeft), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 0, 255), 2)

            centerX = (topLeft[0] + bottomRight[0]) // 2
            centerY = (topLeft[1] + bottomRight[1]) // 2

            center = (centerX, centerY)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            topMiddleX = (topLeft[0] + topRight[0]) // 2
            topMiddleY = (topLeft[1] + topRight[1]) // 2
            
            direction = math.degrees(math.atan2(topMiddleY - centerY, topMiddleX - centerX))
            cv2.putText(frame, f"dir: {direction}", tuple(topRight), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 0, 255), 2)            
            information.append([int(markerID[0]), center, direction])

    return information, frame

def detect_pix(frame: np.ndarray, frameRGB: np.ndarray, colorCode: tuple, method: int):
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam
    """
    ledPositions = []

    Contours, hierarchy = cv2.findContours(frame, method, cv2.CHAIN_APPROX_SIMPLE)

    for contour in Contours:
        area = cv2.contourArea(contour)
        if area > 400:
            x, y, w, h = cv2.boundingRect(contour)
            ledPositions.append([x + 0.5 * w, y + 0.5* h])
            cv2.rectangle(frameRGB, (x, y), (x + w, y + h), colorCode, 2)

    if not ledPositions:
        ledPositions.append([-1,-1])

    return ledPositions, frameRGB

def ledDetection(frameBGR: np.ndarray):
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam
    """
    ledPositions = []

    frameRGB = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)
    frameHSV = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)

    frameR, frameG, frameB = cv2.split(frameRGB)

    frameBG = cv2.subtract(frameB, frameG)
    frameRB = cv2.subtract(frameR, frameB)

    ret, frameBG = cv2.threshold(frameBG, 37, 255, cv2.THRESH_BINARY)
    blueLedPositions, frameRGB = detect_pix(frameBG, frameRGB, (0, 0, 255), cv2.RETR_TREE)
    ledPositions.append(blueLedPositions)
    
    ret, frameRB = cv2.threshold(frameRB, 60, 255, cv2.THRESH_BINARY)
    redLedPositions, frameRGB = detect_pix(frameRB, frameRGB, (255, 0, 0), cv2.RETR_TREE)
    ledPositions.append(redLedPositions)

    lowerGreen = np.array([35, 40, 40])
    upperGreen = np.array([95, 255, 255])
    maskG = cv2.inRange(frameHSV, lowerGreen, upperGreen)
    greenLedPositions, frameRGB = detect_pix(maskG, frameRGB, (0, 255, 0), cv2.RETR_EXTERNAL)
    ledPositions.append(greenLedPositions)

    outputFrame = cv2.cvtColor(frameRGB, cv2.COLOR_RGB2BGR)
    return ledPositions, outputFrame

def linkLedToChariot(arUcoInformation: list, ledPositions: list, frame: np.ndarray):
    """
    Assuming two ArUco's are not directly next to eachother

    Could cause a problem if two chariots are directly next to eachother 
    and the led of one chariot is assigned to both
    """
    maxAllowedDistance = 25
    chariotInformation = []

    for chariot in arUcoInformation:
        status = "Off"
        bestDistance = maxAllowedDistance + 1
        for colorIndex, ledColor in enumerate(ledPositions):
            for led in ledColor:
                if led == (-1,-1):
                    continue
                distance = math.dist(chariot[1], led)
                if distance < bestDistance:
                    if colorIndex == 0: # Blue LED
                        status = "Available"
                    elif colorIndex == 1: # Red LED
                        status = "Connecting"
                    elif colorIndex == 2: # Green LED
                        status = "Connected"
                    bestDistance = distance

                    cv2.line(frame, chariot[1], led, (255, 0, 0), 3)

        chariotInformation.append([chariot[0], chariot[1], chariot[2], status])

    return chariotInformation, frame

def sendChariotInformation(chariotInformation: list):
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

def videoProcessing(file: str, record: bool, camera: bool, type: str) -> None:
    capture = cv2.VideoCapture(file)

    if not capture.isOpened():
        print("Error: Could not open video")
        return

    if record:
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if camera:
            width, height = height, width

        fps = capture.get(cv2.CAP_PROP_FPS)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        if type == "aruco":
            out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height), isColor=False)
        else:
            out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))


    while True:
        try:
            ret, frame = capture.read()

            if not ret:
                print("End of video")
                break

            if camera:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            arUcoInformation, arUcoFrame = arUcoDetection(frame)
            ledPositions, ledFrame = ledDetection(frame)
            chariotInformation, linkingFrame = linkLedToChariot(arUcoInformation, ledPositions, frame)
            if chariotInformation:
                sendChariotInformation(chariotInformation)

            if type == "aruco":
                cv2.imshow("Frames", arUcoFrame)
                if record:
                    out.write(arUcoFrame)
            elif type == "leds":
                cv2.imshow("Frames", ledFrame)
                if record:
                    out.write(ledFrame)
            elif type == "linking":
                cv2.imshow("Frames", linkingFrame)
                if record:
                    out.write(linkingFrame)
            else:
                cv2.imshow("Frames", frame)
                if record:
                    out.write(frame)

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

    #videoProcessing("http://192.168.1.108:8080/video", record=True, camera=True, type="linking")
    #videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/aruco test/arucoturntest.mp4", record=False, camera=False, type="aruco")
    #videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/leds test/led_lightson_openwindow.mp4", record=False, camera=False, type="leds")