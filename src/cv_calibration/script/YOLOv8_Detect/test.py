from ultralytics import YOLO
import cv2

# Load a model
model = YOLO('./best.pt')  # pretrained YOLOv8n model

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS,30)

while True:
    _ , frame = cap.read()
    result = model(frame)
    cv2.imshow("detect",result[0].plot())
    if cv2.waitKey(1)==ord('q'):
        break