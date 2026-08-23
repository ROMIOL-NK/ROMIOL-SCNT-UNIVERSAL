import numpy as np
from ultralytics import YOLO
import cv2
import os
import torch


colors = [(0, 200, 128), (200, 100, 0), (255, 0, 255), (100,100,100), (100, 100, 200), (150,150,150), (0,0,200), (200,200,200), (100,100,100)]
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(device)
yolomodel = YOLO('F:/SCNT-detect.pt')
print("----------------------------------Model load completed----------------------------------")
img_path = r"F:\Micro-Dataset\吐核图片/"
def test_img_folder(img_path):
    for img_name in os.listdir(img_path):
        if img_name.endswith('.jpg'):
            print(img_name)
            image = cv2.imread(img_path+img_name)
            showImg = image.copy()
            cv2.imshow('img', image)
            results = yolomodel.predict(image, conf=0.5)
            names = results[0].names
            for box in results[0].boxes.data:
                box = box.cpu().tolist()
                x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                classId = int(box[-1])
                name = names[classId]
                cv2.rectangle(showImg, (x1, y1), (x2, y2), colors[classId])
                cv2.putText(showImg, name, (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colors[classId])
            cv2.imshow('detect', showImg)
            cv2.waitKey(0)
def test_video(video_path):
    cap = cv2.VideoCapture(video_path)
    _, image = cap.read()
    while _:
        showImg = image.copy()
        results = yolomodel.predict(image, conf=0.5)
        names = results[0].names
        for box in results[0].boxes.data:
            box = box.cpu().tolist()
            x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
            classId = int(box[-1])
            name = names[classId]
            cv2.rectangle(showImg, (x1, y1), (x2, y2), colors[classId])
            cv2.putText(showImg, name, (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colors[classId])
        cv2.imshow('img', image)
        cv2.imshow('detect', showImg)
        cv2.waitKey(0)
        _, image = cap.read()
    cap.release()

path = r'F:\0Exp/'
# test_img_folder(path)
for fname in os.listdir(path):
    if fname.endswith('.avi'):
        print(fname)
        test_video(path+fname)