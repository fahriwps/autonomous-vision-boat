import cv2
import numpy as np
import serial
import time

cap = cv2.VideoCapture('http://0.0.0.0:5000/video_viewer')
kernel = np.ones((5,5),np.uint8)

#arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.3)
#time.sleep(3)

def arduino_serial(kirimX,radius):
    if (20<radius<70):
        arduino.write(str(kirimX).encode())
    else:
        arduino.write(str('100').encode())

    time.sleep(0.3)

def nothing(x):
    pass
def map1(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def loadHSV(nama):
    data = [255,255,255,255,255,255]
    file = open(nama,'r')
    i=0
    while(i<6):
        data[i] = file.readline()
        i+=1
    file.close()
    return data[0],data[1],data[2],data[3],data[4],data[5]


def saveHSV(nama,data):
    file = open(nama,'w')
    i=0
    while(i<6):
        file.write(str(data[i])+'\n')
        i+=1
    file.close()

def taskbar():
    cv2.namedWindow("Merah")
    cv2.createTrackbar("H min","Merah",0,255,nothing)
    cv2.createTrackbar("S min","Merah",100,255,nothing)
    cv2.createTrackbar("V min","Merah",100,255,nothing)
    cv2.createTrackbar("H max","Merah",10,255,nothing)
    cv2.createTrackbar("S max","Merah",255,255,nothing)
    cv2.createTrackbar("V max","Merah",255,255,nothing)
    cv2.createTrackbar("Save","Merah",0,1,nothing)

def func_1(hsv_,lower_,upper_,gray_):
    mask_ = cv2.inRange(hsv_,lower_,upper_)
    res_ = cv2.bitwise_and(gray_,gray_, mask=mask_)
    im1_ = cv2.GaussianBlur(res_,(5,5),0)
    im2_ = cv2.bilateralFilter(im1_,9,75,75)
    im_ = cv2.dilate(im2_,kernel,iterations = 2)
    im_ = cv2.erode(im_,kernel,iterations = 1)
    im_ = cv2.morphologyEx(im_, cv2.MORPH_OPEN, kernel)
    return im2_

def func_2(mask_,warna):
    color = np.array([])
    if warna == "Merah":
        color = (0,0,255)
    elif warna == "Hijau":
        color = (0,255,0)
    x = 0
    y = 0
    radius = 0
    jarak = 0
    cnts = cv2.findContours(mask_.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        for cnt in cnts:
            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius),color, 2)
                cv2.circle(frame, center, 5, color, -1)
                jarak = round(((10*547.64)/radius),2)
    return int(x),int(y),int(radius),jarak


taskbar()
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    lower_merah = np.array([cv2.getTrackbarPos("H min","Merah"), cv2.getTrackbarPos("S min","Merah"), cv2.getTrackbarPos("V min","Merah")], dtype="uint8")
    upper_merah = np.array([cv2.getTrackbarPos("H max","Merah"), cv2.getTrackbarPos("S max","Merah"), cv2.getTrackbarPos("V max","Merah")], dtype="uint8")

    #lower_hijau = np.array([cv2.getTrackbarPos("H min","Hijau"), cv2.getTrackbarPos("S min","Hijau"), cv2.getTrackbarPos("V min","Hijau")], dtype="uint8")
    #upper_hijau = np.array([cv2.getTrackbarPos("H max","Hijau"), cv2.getTrackbarPos("S max","Hijau"), cv2.getTrackbarPos("V max","Hijau")], dtype="uint8")

    im2 = func_1(hsv,lower_merah,upper_merah,gray)

    x,y,radius,jarak = func_2(im2,"Merah")
    kirimX= map1(x, 0, cap.get(3), 0, 20)
    #arduino_serial(kirimX=kirimX,radius=radius)
    print "X:",x," Y:",y," Radius:",radius," kirmX:",kirimX
    cv2.imshow("output", frame)
    cv2.imshow("im_all", im2)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cv2.waitKey(0)
cv2.destroyAllWindows()
