#Main program for KKTCB 2018
#Program ini mencakup modul opencv(citra), log data, serial-comunication dan self-learning(machine learning based on deeplearning) logic
#Semua program (speedtest dan manuver) membutuhkan program ini

#terimakasih sudah berjalan dengan baik, sayang :)
#selamat menjadi pemenang :)
#madura 17-20 November 2018

import cv2 #image-processing module
import time
import deap #deep-learning library
import os
import numpy as np

dic = os.getcwd()
#dic = os.getcwd() + "/kktcbn"

kernel = np.ones((5,5),np.uint8)

class Log:
    name = ""
    text = ""
    type = {1:"Image Processing",2:"Serial Communication",3:"System"}

    def setName(self,name):
        self.name = name
    def add(self, text,tipe=None):
        self.text = text
        tipe = self.type[tipe]
        fol = dic + "/log_file/" + self.name
        time_now  = time.asctime(time.localtime(time.time()))
        try:
            with open(fol,"a") as f:
                if tipe == None:
                    tipe = 3
                f.write("["+time_now+"] "+tipe +" > "+text+"\n")
        except:
            print "Error to open file"
        f.close


class ImageProcessing:
    gray = []
    hsv = []
    frame = []

    def setFrame(self,frame_):
        self.frame = frame_
        self.hsv = cv2.cvtColor(frame_,cv2.COLOR_BGR2HSV)
        self.gray = cv2.cvtColor(frame_,cv2.COLOR_BGR2GRAY)

    def imageCorrection(self):
        gamma = 1.0
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")
        self.hsv =  cv2.LUT(self.hsv, table)

    def colorFilter(self,upper_,lower_):
        gray_ = self.gray
        hsv_ = self.hsv
        mask_ = cv2.inRange(hsv_,lower_,upper_)
        res_ = cv2.bitwise_and(gray_,gray_, mask=mask_)
        im1_ = cv2.GaussianBlur(res_,(5,5),0)
        im2_ = cv2.bilateralFilter(im1_,9,75,75)
        im_ = cv2.dilate(im2_,kernel,iterations = 2)
        im_ = cv2.erode(im_,kernel,iterations = 1)
        im_ = cv2.morphologyEx(im_, cv2.MORPH_OPEN, kernel)
        return im_

    def enhanceImage(self):
        pass

    def findContour(self,mask_):
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


def crop(image, side,size):
    if(side == "right"):
        return image[120:480, 640-size:640]
    else:
        return image[120:480, 0:size]
        #return image

def loadHSV(nama):
    data = [255,255,255,255,255,255]
    fol = dic + "/calibration_file/" + nama
    file = open(fol,'r')
    i=0
    while(i<6):
        data[i] = file.readline()
        i+=1
    file.close()
    return int(data[0]),int(data[1]),int(data[2]),int(data[3]),int(data[4]),int(data[5])

def saveHSV(nama,data):
    fol = dic + "/calibration_file/" + nama
    file = open(fol,'w')
    i=0
    while(i<6):
        file.write(str(data[i])+'\n')
        i+=1
    file.close()

def nothing(x):
    pass

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
