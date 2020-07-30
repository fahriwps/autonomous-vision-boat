import main
import serial
import time

main.createBar()

time_now  = time.asctime(time.localtime(time.time()))

log_data = main.Log()
log_data.setName(time_now)

cap = main.cv2.VideoCapture(0)
log_data.add("Opening camera device", 1)


fourcc = main.cv2.cv.FOURCC(*'DIVX')
out = main.cv2.VideoWriter(main.dic+"/video_file/"+time_now+".avi",fourcc,20.0,(640,480))
log_data.add("Writing camera video device", 1)

ip_left = main.ImageProcessing()
ip_right = main.ImageProcessing()

error_kanan = 0
error_kiri = 0
error_total = 0
direct = ""

crop_size = 250

def drawFrame(frame,x_red,y_red,r_red,x_green,y_green,r_green):
    main.cv2.rectangle(frame,(0,120),(crop_size,480),(255,255,255),2)
    main.cv2.rectangle(frame,(640-crop_size,120),(640,480),(255,255,255),2)
    main.cv2.putText(frame,"LEFT-MERAH",(120,110),main.cv2.cv.CV_FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),2)
    main.cv2.putText(frame,"RIGHT-GREEN",(640-crop_size,110),main.cv2.cv.CV_FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),2)
    main.cv2.circle(frame, (int(x_red), int(y_red)+120), int(r_red),(255,255,255), 2)
    main.cv2.circle(frame, (int(x_green)+(640-crop_size), int(y_green)+120), int(r_green),(255,255,255), 2)
    return frame
def send(command):
    ser.write("E"+str(command))

def map1(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def main_():
    iteration = 1
    while True:
        log_data.add("========================= | ITERATION ("+str(iteration)+ ") | =========================",3)
        ret,frame = cap.read()
        ip_left.setFrame(main.crop(frame,"left",crop_size))
        ip_right.setFrame(main.crop(frame,"right",crop_size))

        ip_left.imageCorrection()
        ip_right.imageCorrection()

        lower_merah = main.np.array([main.getBar("H min","Merah"), main.getBar("S min","Merah"), main.getBar("V min","Merah")], dtype="uint8")
        upper_merah = main.np.array([main.getBar("H max","Merah"), main.getBar("S max","Merah"), main.getBar("V max","Merah")], dtype="uint8")
        lower_hijau = main.np.array([main.getBar("H min","Hijau"), main.getBar("S min","Hijau"), main.getBar("V min","Hijau")], dtype="uint8")
        upper_hijau = main.np.array([main.getBar("H max","Hijau"), main.getBar("S max","Hijau"), main.getBar("V max","Hijau")], dtype="uint8")

        filter_merah = ip_left.colorFilter(upper_merah,lower_merah)
        filter_hijau = ip_right.colorFilter(upper_hijau,lower_hijau)

        x_merah,y_merah,radius_merah = ip_left.findContour(filter_merah)
        x_hijau,y_hijau,radius_hijau = ip_right.findContour(filter_hijau)

        if x_merah != 0 and x_merah != 0:
            log_data.add("Object Contouring | Red Ball Found | Radius : "+str(radius_merah)+", X: "+str(x_merah), 1)
            main.cv2.circle(filter_merah, (int(x_merah), int(y_merah)), int(radius_merah),(255,255,255), 2)
        else:
            log_data.add("Object Contouring | Red Ball Not Found",1)
        if x_hijau != 0 and x_hijau != 0:
            log_data.add("Object Contouring | Green Ball Found | Radius : "+str(radius_hijau)+", X: "+str(x_hijau), 1)
            main.cv2.circle(filter_hijau, (int(x_hijau), int(y_hijau)), int(radius_hijau),(255,255,255), 2)
        else:
            log_data.add("Object Contouring | Green Ball Not Found",1)

        error_kiri = x_merah + (radius_merah/2)
        log_data.add("Error | Red Left : "+str(error_kiri),1)

        if(x_hijau == 0):
            error_kanan = 0
        else:
            error_kanan = (crop_size - x_hijau) + (radius_hijau/2)
        log_data.add("Error | Green Right : "+str(error_kanan),1)

        if(error_kanan > crop_size):
            error_kanan = crop_size
            log_data.add("Error | Green Right Correction",1)
        if(error_kiri > crop_size):
            error_kiri = crop_size
            log_data.add("Error | Red Left Correction",1)

        error_kiri = map1(error_kiri,0,crop_size,0,10)
        log_data.add("Error | Red Left Mapping : "+str(error_kiri),1)
        error_kanan = map1(error_kanan,0,crop_size,0,10)
        log_data.add("Error | Green Right Mapping : "+str(error_kanan),1)

        error_total = error_kiri - error_kanan #kalau error total positif berarti kapal belok ke kanan dan sebaliknya
        log_data.add("Error | Calculate Total : "+str(error_total),1)

        #send(error_total)
        #log_data.add("Sending data to Microcontroller | Data : "+error_total,2)

        if error_total == 0:
            direct = "lurus"
            log_data.add("Manuver | Direction : Center with Speed : ",1)
        elif error_total < 0:
            direct = "kiri"
            log_data.add("Manuver | Direction : Left with Speed : ",1)
        else:
            direct = "kanan"
            log_data.add("Manuver | Direction : Right with Speed : ",1)

        #print "Error kanan:",error_kanan," | Error kiri:",error_kiri," | Error total:",error_total, " | ",direct

        frame = drawFrame(frame,x_merah,y_merah,radius_merah,x_hijau,y_hijau,radius_hijau)
        out.write(frame)

        main.cv2.imshow("frame", frame)
        main.cv2.imshow("left", filter_merah)
        main.cv2.imshow("right", filter_hijau)
        #main.cv2.imshow("dsd", main.cv2.cvtColor(frame,main.cv2.COLOR_BGR2HSV))

        iteration = iteration + 1

        if main.cv2.waitKey(1) & 0xFF == ord("q"):
            if(main.cv2.getTrackbarPos("Save","Merah") == 1):
                dataHSV = main.np.concatenate((lower_merah,upper_merah))
                main.saveHSV("hsv_merah",dataHSV)
                print("Red HSV Saved")
                log_data.add("Calibration | Red HSV Saved",1)
            if(main.cv2.getTrackbarPos("Save","Hijau") == 1):
                dataHSV = main.np.concatenate((lower_hijau,upper_hijau))
                main.saveHSV("hsv_hijau",dataHSV)
                print("Hijau HSV Saved")
                log_data.add("Calibration | Green HSV Saved",1)

            main.cv2.waitKey(0)
            log_data.add("Interupting Program...",3)
            break

    cap.release()
    log_data.add("Releasing camera device", 1)
    out.release()
    log_data.add("Saving video camera", 1)
    main.cv2.destroyAllWindows()
    log_data.add("Destroying All Windows", 1)
    log_data.add("Shutdown Program...",3)

port = ''
baudrate =  115200
timeout = 1

#ser = serial.Serial(port,baudrate,timeout = None)
#log_data("Opening Serial Communication at Port : "+port+" with Baudrate : "+baudrate,2)
#time.sleep(2)
main_()
