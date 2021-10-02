import cv2
import numpy as np
import serial
import time
import threading

cap = cv2.VideoCapture(1)
kernel = np.ones((5,5),np.uint8)

class Arduino(threading.Thread):
    def __init__(self, xval, radius, colorval):
        threading.Thread.__init__(self)
        self.isRunning = True
        self.colorval = colorval
        self.xval = xval
        self.radius = radius


    def run(self):
        if(self.colorval == "Red"):
            # boat near start/finish line
            if (20<self.radius<70):
                # boat following red ball
                arduino.write(str(self.xval).encode())
            elif (self.radius <= 20):
                # boat keep going straight forward
                arduino.write(str('100').encode())
            elif (self.radius >= 70):
                # turn left/right mapping
                arduino.write('10'.encode())
        else:
            # boat near star/finish line
            if (20<self.radius<70):
                # boat following blue ball
                arduino.write(str(self.xval).encode())
            elif (self.radius <= 720):
                # boat keep going straight forward
                arduino.write(str('100').encode())
            elif (self.radius >= 70):
                # turn left/right mapping
                arduino.write('10'.encode())
        time.sleep(0.1)

def nothing(x):
    pass

def mapping(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def loadHSV(filename):
    data = [255,255,255,255,255,255]
    file = open(filename,'r')
    i=0
    while(i<6):
        data[i] = file.readline()
        i+=1
    file.close()
    return int(data[0]), int(data[1]), int(data[2]), int(data[3]), int(data[4]), int(data[5])

def saveHSV(filename, data):
    file = open('hsv_value/' + filename,'w')
    i=0
    while(i<6):
        file.write(str(data[i])+'\n')
        i+=1
    file.close()

def createBar():
    hue1, saturation1, value1, hue1_, saturation1_, value1_ = loadHSV('hsv_value/rvalue')
    cv2.namedWindow("Red Value")
    cv2.createTrackbar("H min","Red", hue1, 255, nothing)
    cv2.createTrackbar("S min","Red", saturation1, 255, nothing)
    cv2.createTrackbar("V min","Red", value1, 255, nothing)
    cv2.createTrackbar("H max","Red", hue1_, 255, nothing)
    cv2.createTrackbar("S max","Red", saturation1_, 255, nothing)
    cv2.createTrackbar("V max","Red", value1_, 255, nothing)
    cv2.createTrackbar("Save","Red", 0,1, nothing)


    hue2, saturation2, value2, hue2_, saturation2_, value2_ = loadHSV('hsv_value/bvalue')
    cv2.namedWindow("Blue Value")
    cv2.createTrackbar("H min","Blue", hue2, 255, nothing)
    cv2.createTrackbar("S min","Blue", saturation2, 255, nothing)
    cv2.createTrackbar("V min","Blue", value2, 255, nothing)
    cv2.createTrackbar("H max","Blue", hue2_, 255, nothing)
    cv2.createTrackbar("S max","Blue", saturation2_, 255, nothing)
    cv2.createTrackbar("V max","Blue", v2_, 255, nothing)
    cv2.createTrackbar("Save","Blue", 0,1, nothing)

    hue3, saturation3, value3, hue3_, saturation3_, value3_ = loadHSV('hsv_value/yvalue')
    cv2.namedWindow("Yellow")
    cv2.createTrackbar("H min","Yellow", hue3, 255, nothing)
    cv2.createTrackbar("S min","Yellow", saturation3, 255, nothing)
    cv2.createTrackbar("V min","Yellow", value3, 255, nothing)
    cv2.createTrackbar("H max","Yellow", hue3_, 255, nothing)
    cv2.createTrackbar("S max","Yellow", saturation3_, 255, nothing)
    cv2.createTrackbar("V max","Yellow", value3_, 255,  nothing)
    cv2.createTrackbar("Save","Yellow", 0,1, nothing)

def getBar(item, win):
    return cv2.getTrackbarPos(item, win)

def enhance(hsv_,lower_,upper_,gray_):
    mask_ = cv2.inRange(hsv_,lower_,upper_)
    res_ = cv2.bitwise_and(gray_,gray_, mask=mask_)
    im1_ = cv2.GaussianBlur(res_,(5,5),0)
    im2_ = cv2.bilateralFilter(im1_,9,75,75)
    im_ = cv2.dilate(im2_,kernel,iterations = 2)
    im_ = cv2.erode(im_,kernel,iterations = 1)
    im_ = cv2.morphologyEx(im_, cv2.MORPH_OPEN, kernel)
    return im2_

def findContour(mask_, color=None):
    color = np.array([])
    if color == "Red":
        color = (0,0,255)
    elif color == "Blue":
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
        lower_Red = np.array([getBar("H min","Red"), getBar("S min","Red"), getBar("V min","Red")], dtype="uint8")
        upper_Red = np.array([getBar("H max","Red"), getBar("S max","Red"), getBar("V max","Red")], dtype="uint8")
        lower_Blue = np.array([getBar("H min","Blue"), getBar("S min","Blue"), getBar("V min","Blue")], dtype="uint8")
        upper_Blue = np.array([getBar("H max","Blue"), getBar("S max","Blue"), getBar("V max","BIu")], dtype="uint8")
        lower_Yellow = np.array([getBar("H min","Yellow"), getBar("S min","Yellow"), getBar("V min","Yellow")], dtype="uint8")
        upper_Yellow = np.array([getBar("H max","Yellow"), getBar("S max","Yellow"), getBar("V max","Yellow")], dtype="uint8")

        #Enhance Image
        im_Red = enhance(hsv,lower_Red,upper_Red,gray)
        im_Blue = enhance(hsv,lower_Blue,upper_Blue,gray)
        im_Yellow = enhance(hsv,lower_Yellow,upper_Yellow,gray)

        #findContour for getting x , y and radius
        x_Red,y_Red,radius_Red = findContour(im_Red,"Red")
        x_Blue,y_Blue,radius_Blue = findContour(im_Blue,"Blue")
        x_Yellow,y_Yellow,radius_Yellow = findContour(im_Yellow,"Blue") # Boat have same reaction between blue and yellow balls

        #Re-map value 0-20 scala
        xval_Red= mapping(x_Red, 0, cap.get(3), 0, 20)
        xval_Blue= mapping(x_Blue, 0, cap.get(3), 0, 20)
        xval_Yellow= mapping(x_Yellow, 0, cap.get(3), 0, 20)

        # Sending signal to Arduino according to the value of Red/Blue radius
        if(radius_Red >= 10):
            thread_serial = Arduino(xval=xval_Red,radius=radius_Red,color="Red")
            print("Red : X:",x_Red," Y:",y_Red," Radius:",radius_Red," sendX:",xval_Red)
        elif(radius_Blue>radius_Yellow):
            thread_serial = Arduino(xval=xval_Blue,radius=radius_Blue,color="Blue")
            print("Blue : X:",x_Blue," Y:",y_Blue," Radius:",radius_Blue," sendX:",xval_Blue)
        else:
            thread_serial = Arduino(xval=xval_Yellow,radius=radius_Yellow,color="Yellow")
            print("Yellow : X:",x_Yellow," Y:",y_Yellow," Radius:",radius_Yellow," sendX:",xval_Yellow)

        thread_serial.start()

        cv2.imshow("output", frame)
        #cv2.imshow("im_Red", im_Red)
        #cv2.imshow("im_Blue", im_Blue)
        #cv2.imshow("im_Yellow", im_Yellow)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            if(cv2.getTrackbarPos("Save","Red") == 1):
                HSVdata = np.concatenate((lower_Red,upper_Red))
                saveHSV("rvalue",HSVdata)
                print("Red HSV saved")
            if(cv2.getTrackbarPos("Save","Blue") == 1):
                HSVdata = np.concatenate((lower_Blue,upper_Blue))
                saveHSV("bvalue",HSVdata)
                print("Blue HSV Saved")
            if(cv2.getTrackbarPos("Save","Yellow") == 1):
                HSVdata = np.concatenate((lower_Yellow,upper_Yellow))
                saveHSV("yvalue",HSVdata)
                print("Yellow HSV Saved")
            break
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        arduino = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,timeout=0.1)
        time.sleep(1)
        print("Arduino Serial & Camera Connected")
    except:
        print("Error")
    createBar()
    main()
