import cv2
import numpy as np
import serial
import time
import threading

cap = cv2.VideoCapture(0)
kernel = np.ones((5,5),np.uint8)
global arduino

class Arduino(threading.Thread):
    def __init__(self,kirimX,radius,warna):
        threading.Thread.__init__(self)
        self.isRunning = True
        self.warna = warna
        self.kirimX = kirimX
        self.radius = radius


    def run(self):
        #kapal berada di deramaga / di dekat posisi finish
        if (20<self.radius<290):
            #kapal follow bola biru
            arduino.write(str(self.kirimX).encode())
        elif (self.radius <= 720):
            #kapal lurus karena bola masih jauh dan berada di luar jangkauan
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
    return int(data[0]),int(data[1]),int(data[2]),int(data[3]),int(data[4]),int(data[5])

def saveHSV(nama,data):
    file = open(nama,'w')
    i=0
    while(i<6):
        file.write(str(data[i])+'\n')
        i+=1
    file.close()

def createBar():

    h2,s2,v2,h2_,s2_,v2_ = loadHSV('hsv_biru')
    cv2.namedWindow("Biru")
    cv2.createTrackbar("H min","Biru",h2,255,nothing)
    cv2.createTrackbar("S min","Biru",s2,255,nothing)
    cv2.createTrackbar("V min","Biru",v2,255,nothing)
    cv2.createTrackbar("H max","Biru",h2_,255,nothing)
    cv2.createTrackbar("S max","Biru",s2_,255,nothing)
    cv2.createTrackbar("V max","Biru",v2_,255,nothing)
    cv2.createTrackbar("Save","Biru",0,1,nothing)

def getBar(item,win):
    return cv2.getTrackbarPos(item,win)

def enhance(hsv_,lower_,upper_,gray_):
    mask_ = cv2.inRange(hsv_,lower_,upper_)
    res_ = cv2.bitwise_and(gray_,gray_, mask=mask_)
    im1_ = cv2.GaussianBlur(res_,(5,5),0)
    im2_ = cv2.bilateralFilter(im1_,9,75,75)
    im_ = cv2.dilate(im2_,kernel,iterations = 2)
    im_ = cv2.erode(im_,kernel,iterations = 1)
    im_ = cv2.morphologyEx(im_, cv2.MORPH_OPEN, kernel)
    return im2_

def findContour(mask_,warna=None):
    color = np.array([])
    if warna == "Merah":
        color = (0,0,255)
    elif warna == "Biru":
        color = (255,0,0)
    else:
        color = (0,0,0)
    x = 0
    y = 0
    radius = 0
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
    return int(x),int(y),int(radius)


def main():
    global frame
    #Main Function
    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        hsv_ = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        #Gamma Correction
        gamma = 1.0
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")
        hsv =  cv2.LUT(hsv_, table)

        #Get HSV Trackbar Position
        lower_biru = np.array([getBar("H min","Biru"), getBar("S min","Biru"), getBar("V min","Biru")], dtype="uint8")
        upper_biru = np.array([getBar("H max","Biru"), getBar("S max","Biru"), getBar("V max","BIu")], dtype="uint8")

        #Enhance Image
        im_biru = enhance(hsv,lower_biru,upper_biru,gray)

        #findContour for getting x , y and radius
        x_biru,y_biru,radius_biru = findContour(im_biru,"Biru")

        #Re-map value 0-20 scala
        kirimX_biru= map1(x_biru, 0, cap.get(3), 0, 20)

        #Kirim sinyal ke arduino sesuai dengan besar radius merah/ biru
        thread_serial = Arduino(kirimX=kirimX_biru,radius=radius_biru,warna="biru")
        thread_serial.start()
        thread_serial.join()
        print "Biru : X:",x_biru," Y:",y_biru," Radius:",radius_biru," kirmX:",kirimX_biru
        cv2.imshow("output", frame)
        #cv2.imshow("im_merah", im_merah)
        cv2.imshow("im_biru", im_biru)
        #cv2.imshow("im_kuning", im_kuning)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            if(cv2.getTrackbarPos("Save","Biru") == 1):
                dataHSV = np.concatenate((lower_biru,upper_biru))
                saveHSV("hsv_biru",dataHSV)
                print("Blue HSV Saved")
            break
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.3)
        time.sleep(1)
        print("Arduino Serial & Camera Connected")
    except:
        print("Error")
    createBar()
    main()
