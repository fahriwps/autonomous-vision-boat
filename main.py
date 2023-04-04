import cv2
import numpy as np

from arduino_thread import ArduinoThread


def mapping(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def load_hsv(filename):
    data = [255, 255, 255, 255, 255, 255]
    file = open(filename, "r")
    i = 0
    while i < 6:
        data[i] = file.readline()
        i += 1
    file.close()
    return (
        int(data[0]),
        int(data[1]),
        int(data[2]),
        int(data[3]),
        int(data[4]),
        int(data[5]),
    )


def save_hsv(filename, data):
    file = open("hsv_value/" + filename, "w")
    i = 0
    while i < 6:
        file.write(str(data[i]) + "\n")
        i += 1
    file.close()


def create_bar():
    hue1, saturation1, value1, hue1_, saturation1_, value1_ = load_hsv(
        "hsv_value/rvalue"
    )
    cv2.namedWindow("Red Value")
    cv2.createTrackbar("H min", "Red", hue1, 255, nothing)
    cv2.createTrackbar("S min", "Red", saturation1, 255, nothing)
    cv2.createTrackbar("V min", "Red", value1, 255, nothing)
    cv2.createTrackbar("H max", "Red", hue1_, 255, nothing)
    cv2.createTrackbar("S max", "Red", saturation1_, 255, nothing)
    cv2.createTrackbar("V max", "Red", value1_, 255, nothing)
    cv2.createTrackbar("Save", "Red", 0, 1, nothing)

    hue2, saturation2, value2, hue2_, saturation2_, value2_ = load_hsv(
        "hsv_value/bvalue"
    )
    cv2.namedWindow("Blue Value")
    cv2.createTrackbar("H min", "Blue", hue2, 255, nothing)
    cv2.createTrackbar("S min", "Blue", saturation2, 255, nothing)
    cv2.createTrackbar("V min", "Blue", value2, 255, nothing)
    cv2.createTrackbar("H max", "Blue", hue2_, 255, nothing)
    cv2.createTrackbar("S max", "Blue", saturation2_, 255, nothing)
    cv2.createTrackbar("V max", "Blue", value2_, 255, nothing)
    cv2.createTrackbar("Save", "Blue", 0, 1, nothing)

    hue3, saturation3, value3, hue3_, saturation3_, value3_ = load_hsv(
        "hsv_value/yvalue"
    )
    cv2.namedWindow("Yellow")
    cv2.createTrackbar("H min", "Yellow", hue3, 255, nothing)
    cv2.createTrackbar("S min", "Yellow", saturation3, 255, nothing)
    cv2.createTrackbar("V min", "Yellow", value3, 255, nothing)
    cv2.createTrackbar("H max", "Yellow", hue3_, 255, nothing)
    cv2.createTrackbar("S max", "Yellow", saturation3_, 255, nothing)
    cv2.createTrackbar("V max", "Yellow", value3_, 255, nothing)
    cv2.createTrackbar("Save", "Yellow", 0, 1, nothing)


def get_bar(item, win):
    return cv2.getTrackbarPos(item, win)


def enhance(hsv_, lower_, upper_, gray_, kernel):
    mask_ = cv2.inRange(hsv_, lower_, upper_)
    res_ = cv2.bitwise_and(gray_, gray_, mask=mask_)
    im1_ = cv2.GaussianBlur(res_, (5, 5), 0)
    im2_ = cv2.bilateralFilter(im1_, 9, 75, 75)
    im_ = cv2.dilate(im2_, kernel, iterations=2)
    im_ = cv2.erode(im_, kernel, iterations=1)
    im_ = cv2.morphologyEx(im_, cv2.MORPH_OPEN, kernel)
    return im2_


def find_contour(mask_, color=None):
    color = np.array([])
    if color == "Red":
        color = (0, 0, 255)
    elif color == "Blue":
        color = (255, 0, 0)
    else:
        color = (0, 0, 0)
    x = 0
    y = 0
    radius = 0
    cnts = cv2.findContours(mask_.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[
        -2
    ]
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        for cnt in cnts:
            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), color, 2)
                cv2.circle(frame, center, 5, color, -1)
    return int(x), int(y), int(radius)


def nothing():
    pass


def main():
    global frame
    kernel = np.ones((5, 5), np.uint8)
    cap = cv2.VideoCapture(0)
    if cap is None or not cap.isOpened():
        raise ValueError("UNABLE TO OPEN CAM")

    while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv_ = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        gamma = 1.0
        invGamma = 1.0 / gamma
        table = np.array(
            [((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]
        ).astype("uint8")
        hsv = cv2.LUT(hsv_, table)

        lower_Red = np.array(
            [get_bar("H min", "Red"), get_bar("S min", "Red"), get_bar("V min", "Red")],
            dtype="uint8",
        )
        upper_Red = np.array(
            [get_bar("H max", "Red"), get_bar("S max", "Red"), get_bar("V max", "Red")],
            dtype="uint8",
        )
        lower_Blue = np.array(
            [get_bar("H min", "Blue"), get_bar("S min", "Blue"), get_bar("V min", "Blue")],
            dtype="uint8",
        )
        upper_Blue = np.array(
            [get_bar("H max", "Blue"), get_bar("S max", "Blue"), get_bar("V max", "BIu")],
            dtype="uint8",
        )
        lower_Yellow = np.array(
            [
                get_bar("H min", "Yellow"),
                get_bar("S min", "Yellow"),
                get_bar("V min", "Yellow"),
            ],
            dtype="uint8",
        )
        upper_Yellow = np.array(
            [
                get_bar("H max", "Yellow"),
                get_bar("S max", "Yellow"),
                get_bar("V max", "Yellow"),
            ],
            dtype="uint8",
        )

        im_Red = enhance(hsv, lower_Red, upper_Red, gray, kernel)
        im_Blue = enhance(hsv, lower_Blue, upper_Blue, gray, kernel)
        im_Yellow = enhance(hsv, lower_Yellow, upper_Yellow, gray, kernel)

        x_Red, y_Red, radius_Red = find_contour(im_Red, "Red")
        x_Blue, y_Blue, radius_Blue = find_contour(im_Blue, "Blue")
        x_Yellow, y_Yellow, radius_Yellow = find_contour(
            im_Yellow, "Blue"
        )  # Boat have same reaction between blue and yellow balls

        xval_Red = mapping(x_Red, 0, cap.get(3), 0, 20)
        xval_Blue = mapping(x_Blue, 0, cap.get(3), 0, 20)
        xval_Yellow = mapping(x_Yellow, 0, cap.get(3), 0, 20)

        if radius_Red >= 10:
            thread_serial = ArduinoThread(xval=xval_Red, radius=radius_Red, color="Red")
            print(
                "Red : X:",
                x_Red,
                " Y:",
                y_Red,
                " Radius:",
                radius_Red,
                " sendX:",
                xval_Red,
            )
        elif radius_Blue > radius_Yellow:
            thread_serial = ArduinoThread(
                xval=xval_Blue, radius=radius_Blue, color="Blue"
            )
            print(
                "Blue : X:",
                x_Blue,
                " Y:",
                y_Blue,
                " Radius:",
                radius_Blue,
                " sendX:",
                xval_Blue,
            )
        else:
            thread_serial = ArduinoThread(
                xval=xval_Yellow, radius=radius_Yellow, color="Yellow"
            )
            print(
                "Yellow : X:",
                x_Yellow,
                " Y:",
                y_Yellow,
                " Radius:",
                radius_Yellow,
                " sendX:",
                xval_Yellow,
            )

        thread_serial.start()

        cv2.imshow("output", frame)
        # cv2.imshow("im_Red", im_Red)
        # cv2.imshow("im_Blue", im_Blue)
        # cv2.imshow("im_Yellow", im_Yellow)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            if cv2.getTrackbarPos("Save", "Red") == 1:
                HSVdata = np.concatenate((lower_Red, upper_Red))
                save_hsv("rvalue", HSVdata)
                print("Red HSV saved")
            if cv2.getTrackbarPos("Save", "Blue") == 1:
                HSVdata = np.concatenate((lower_Blue, upper_Blue))
                save_hsv("bvalue", HSVdata)
                print("Blue HSV Saved")
            if cv2.getTrackbarPos("Save", "Yellow") == 1:
                HSVdata = np.concatenate((lower_Yellow, upper_Yellow))
                save_hsv("yvalue", HSVdata)
                print("Yellow HSV Saved")
            break
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    create_bar()
    main()
