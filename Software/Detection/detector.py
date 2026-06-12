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
    else:
        print("Couldn't connect to mqtt topic")

mqttClientId = "Camera"
mqttPort = 1883
mqttBroker = "145.24.237.88" # verbergen?
mqttTopic = "Robots/Data/Positions"
client = mqtt_client.Client(client_id=mqttClientId)
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

def detect_pix (frame: np.ndarray, frameRGB: np.ndarray, color: str, colorCode: tuple, method: int):
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam
    """
    ledPositions = []

    Contours, hierarchy = cv2.findContours(frame, method, cv2.CHAIN_APPROX_SIMPLE)

    for contour in Contours:
        area = cv2.contourArea(contour)
        if area > 400:
            x, y, w, h = cv2.boundingRect(contour)
            ledPositions.append((x + 0.5 * w, y + 0.5* h))

            cv2.rectangle(frameRGB, (x, y), (x + w, y + h), colorCode, 2)
            print(color, np.round(x + (w / 2)),np.round(y + (h / 2)))

    if not ledPositions:
        ledPositions.append((-1,-1))

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

    # BLUE
    ret, frameBG = cv2.threshold(frameBG, 37, 255, cv2.THRESH_BINARY)
    blueLedPositions, blueLedframeRGB = detect_pix(frameBG, frameRGB, "Blue", (0, 0, 255), cv2.RETR_TREE)
    ledPositions.append(blueLedPositions)
    
    # RED
    ret, frameRB = cv2.threshold(frameRB, 60, 255, cv2.THRESH_BINARY)
    redLedPositions, frameRGB = detect_pix(frameRB, frameRGB, "Red", (255, 0, 0), cv2.RETR_TREE)
    ledPositions.append(redLedPositions)

    # GREEN
    lowerGreen = np.array([35, 40, 40])
    upperGreen = np.array([95, 255, 255])
    maskG = cv2.inRange(frameHSV, lowerGreen, upperGreen)
    greenLedPositions, frameRGB = detect_pix(maskG, frameRGB, "Green", (0, 255, 0), cv2.RETR_EXTERNAL)
    ledPositions.append(greenLedPositions)

    outputFrame = cv2.cvtColor(frameRGB, cv2.COLOR_RGB2BGR)
    return ledPositions, outputFrame

def linkLedToChariot(arUcoInformation, ledPositions, maxAllowedDistance):
    """
    Assuming two ArUco's are not directly next to eachother

    Could cause a problem if two chariots are directly next to eachother 
    and the led of one chariot is assigned to both
    """
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
        chariotInformation.append([chariot[0], chariot[1], chariot[2], status])

    return chariotInformation

def sendChariotInformation(chariotInformation):
    for chariot in chariotInformation:
        message = {
            "ArUco_ID": chariot[0], 
            "x_position": chariot[1][0],
            "y_position": chariot[1][1],
            "orientation": chariot[2],
            "led_status": chariot[3]
        }
        message_json = json.dumps(message)
        client.publish(mqttTopic, message_json)

def videoProcessing(file: str, record: bool, camera: bool):
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
        out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height)) #, isColor=False)

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
            chariotInformation = linkLedToChariot(arUcoInformation, ledPositions, 25)
            sendChariotInformation(chariotInformation)

            cv2.imshow("frame", frame)
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

    videoProcessing("http://145.137.58.237:8080/video", record=True, camera=True)
    #videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/aruco test/arucoturntest.mp4", record=False, camera=False)
    #videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/leds test/led_lightson_openwindow.mp4", record=False, camera=False)