import cv2
import numpy as np
from ultralytics import YOLO 
from threads.detector import Detector

name = ['car', 'closed-road-stand', 'crosswalk-sign', 'highway-entry-sign', 'highway-exit-sign', 'no-entry-road-sign', 'one-way-road-sign', 'parking-sign', 'parking-spot', 'pedestrian', 'priority-sign', 'round-about-sign', 'stop-line', 'stop-sign', 'traffic-light']

def draw(frame, box):
    x1, y1, x2, y2 = np.int32(box.xyxy[0].cpu())
    classID = int(box.cls.cpu())
    
    # Extract the confidence score and round it to 2 decimal places
    confidence = round(float(box.conf.cpu()[0]), 2)
    
    # Draw the bounding box on the frame
    cv2.rectangle(img = frame, pt1 = (x1, y1), pt2 = (x2, y2), color = (0, 0, 255), thickness = 2)
    
    # Put the confidence text on the overlay
    cv2.putText(frame, f"{name[classID]}: {confidence}", (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_DUPLEX, .5, (0, 0, 255), 1, cv2.LINE_AA)

if __name__ == "__main__":
    cap = cv2.VideoCapture("./Videos/carrun.avi")
    model = YOLO("./SignDetection/Cook_Station_1/weights/best.engine") 
    
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("", frame)

            output = model(frame, verbose = False, nms = True, conf = 0.1)[0]
            boxes = output.boxes.to("cpu")
            for box in boxes:
                draw(frame, box)
                
            cv2.imshow("", frame)
        
        key = cv2.waitKey(60) & 0xFF
        if key == ord("q") or not ret: 
            break