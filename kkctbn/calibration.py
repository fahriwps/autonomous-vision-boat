import cv2
import os
import serial
import numpy as np

kernel = np.ones((5,5),np.uint8)

port = ''
baudrate =  115200
timeout = 1

#ser = serial.Serial(port,baudrate,timeout = None)
#time.sleep(2)

def nothing(x):
    pass

def loadHSV(nama):
    data = [255,255,255,255,255,255]
    fol = os.getcwd() + "/calibration_file/" + nama
    file = open(fol,'r')
    i=0
    while(i<6):
        data[i] = file.readline()
        i+=1
    file.close()
    return int(data[0]),int(data[1]),int(data[2]),int(data[3]),int(data[4]),int(data[5])

def map1(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def saveHSV(nama,data):
    fol = os.getcwd() + "/calibration_file/" + nama
    file = open(fol,'w')
    i=0
    while(i<6):
        file.write(str(data[i])+'\n')
        i+=1
    file.close()

def send(command):
    ser.write("E"+str(command))

def createBar():
    h1,s1,v1,h1_,s1_,v1_ = loadHSV("hsv_merah")
    cv2.namedWindow("Merah")
    cv2.createTrackbar("H min","Merah",h1,255,nothing)
    cv2.createTrackbar("S min","Merah",s1,255,nothing)
    cv2.createTrackbar("V min","Merah",v1,255,nothing)
    cv2.createTrackbar("H max","Merah",h1_,255,nothing)
    cv2.createTrackbar("S max","Merah",s1_,255,nothing)
    cv2.createTrackbar("V max","Merah",v1_,255,nothing)
    cv2.createTrackbar("Save","Merah",0,1,nothing)

    h2,s2,v2,h2_,s2_,v2_ = loadHSV("hsv_hijau")
    cv2.namedWindow("Hijau")
    cv2.createTrackbar("H min","Hijau",h2,255,nothing)
    cv2.createTrackbar("S min","Hijau",s2,255,nothing)
    cv2.createTrackbar("V min","Hijau",v2,255,nothing)
    cv2.createTrackbar("H max","Hijau",h2_,255,nothing)
    cv2.createTrackbar("S max","Hijau",s2_,255,nothing)
    cv2.createTrackbar("V max","Hijau",v2_,255,nothing)
    cv2.createTrackbar("Save","Hijau",0,1,nothing)

def getBar(item,win):
    return cv2.getTrackbarPos(item,win)

def imageCorrection(hsv_):
    gamma = 1.0
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    hsv_ =  cv2.LUT(hsv_, table)
    return hsv_

def enhance(hsv_,lower_,upper_,gray_):
    mask_ = cv2.inRange(hsv_,lower_,upper_)
    res_ = cv2.bitwise_and(gray_,gray_, mask=mask_)
    im1_ = cv2.GaussianBlur(res_,(5,5),0)
    im2_ = cv2.bilateralFilter(im1_,9,75,75)
    im_ = cv2.dilate(im2_,kernel,iterations = 2)
    im_ = cv2.erode(im_,kernel,iterations = 1)
    im_ = cv2.morphologyEx(im_, cv2.MORPH_OPEN, kernel)
    return im2_

def crop(image, side):
    if(side == "right"):
        return image[120:480, 490:640]
    else:
        return image[120:480, 0:150]

def findContour(mask_):
    x = 0
    y = 0
    radius = 0
    cnts = cv2.findContours(mask_.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
        for cnt in cnts:
            c = max(cnts, key=cv2.contourArea)
            ((x_, y_), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius > 50:
                x = x_
                y = y_
                #temp.append([int(x),int(y),int(radius)])
                #cv2.circle(frame, (int(x), int(y)), int(radius),color, 2)
                #cv2.circle(frame, center, 5, color, -1)
    #print radius
    return int(x), int(y), int(radius)

cap = cv2.VideoCapture(1)
createBar()

while True:
    ret, frame = cap.read()
    frame_left = crop(frame,"left")
    frame_right = crop(frame,"right")

    hsv_left = cv2.cvtColor(frame_left,cv2.COLOR_RGB2HSV)
    gray_left = cv2.cvtColor(frame_left,cv2.COLOR_RGB2GRAY)

    hsv_right = cv2.cvtColor(frame_right,cv2.COLOR_RGB2HSV)
    gray_right = cv2.cvtColor(frame_right,cv2.COLOR_RGB2GRAY)


    lower_merah = np.array([getBar("H min","Merah"), getBar("S min","Merah"), getBar("V min","Merah")], dtype="uint8")
    upper_merah = np.array([getBar("H max","Merah"), getBar("S max","Merah"), getBar("V max","Merah")], dtype="uint8")
    lower_hijau = np.array([getBar("H min","Hijau"), getBar("S min","Hijau"), getBar("V min","Hijau")], dtype="uint8")
    upper_hijau = np.array([getBar("H max","Hijau"), getBar("S max","Hijau"), getBar("V max","Hijau")], dtype="uint8")

    im_merah = enhance(hsv_left,lower_merah,upper_merah,gray_left)
    im_hijau = enhance(hsv_right,lower_hijau,upper_hijau,gray_right)

    x_merah,y_merah,radius_merah = findContour(im_merah)
    x_hijau,y_hijau,radius_hijau = findContour(im_hijau)

    if x_merah != 0 and x_merah != 0:
        cv2.circle(im_merah, (int(x_merah), int(y_merah)), int(radius_merah),(255,255,255), 2)
    if x_hijau != 0 and x_hijau != 0:
        cv2.circle(im_hijau, (int(x_hijau), int(y_hijau)), int(radius_hijau),(255,255,255), 2)

    error_kiri = x_merah + (radius_merah/2)

    if(x_hijau == 0):
        error_kanan = 0
    else:
        error_kanan = (150 - x_hijau) + (radius_hijau/2)

    if(error_kanan > 150):
        error_kanan = 150
    if(error_kiri > 150):
        error_kiri = 150

    error_kiri = map1(error_kiri,0,150,0,10)
    error_kanan = map1(error_kanan,0,150,0,10)

    error_total = error_kiri - error_kanan #kalau error total positif berarti kapal belok ke kanan dan sebaliknya
    #send(error_total)

    if error_total == 0:
        direct = "lurus"
    elif error_total < 0:
        direct = "kiri"
    else:
        direct = "kanan"

    print "Error kanan:",error_kanan," | Error kiri:",error_kiri," | Error total:",error_total, " | ",direct


    cv2.imshow("Hasil frame", frame)
    cv2.imshow("Hasil Merah", im_merah)
    cv2.imshow("Hasil Hijau", im_hijau)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        if(cv2.getTrackbarPos("Save","Merah") == 1):
            dataHSV = np.concatenate((lower_merah,upper_merah))
            saveHSV("hsv_merah",dataHSV)
            print("Red HSV Saved")
        if(cv2.getTrackbarPos("Save","Hijau") == 1):
            dataHSV = np.concatenate((lower_hijau,upper_hijau))
            saveHSV("hsv_hijau",dataHSV)
            print("Hijau HSV Saved")
        break
cv2.waitKey(0)
cv2.destroyAllWindows()
