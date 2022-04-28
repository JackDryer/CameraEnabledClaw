import cv2
import numpy as np

CENTERCIRCLECOLOUR = (0,255,255)
CONTOURCOLOUR=(0, 255, 0)
MAXCAMERAS = 10

def maskImage(frame,colour,tollerance=10,closings=1,openings=1,dilateions=0):
    frame = cv2.resize(frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of any color in HSV (hue, saturation, value)((what colour it is, how intence it is , and how dark it is))
    # old colour values
    #lower = np.array([155,25,0])
    #upper = np.array([179,255,255])
    #newer colour values
    ##lower = np.array([0, 200, 120])
    ##upper = np.array([179, 255, 255])
    lower,upper = pad(np.array(colour),tollerance)
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)
    # post prosseing
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,iterations=closings)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,iterations=openings)
    mask = cv2.dilate(mask, None, iterations=dilateions)
    # gray = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
    return (frame, mask)


def pad(array: np.array, tollerance):
    return array-tollerance, array + tollerance


def findMasks(frame, *colours, tolllerance=10):
    frame = cv2.resize(frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of any color in HSV (hue, saturation, value)
    masks = []
    kernal = np.ones((40,40),np.uint8)
    for x, i in enumerate(colours):
        if not(isinstance(i, np.ndarray)):
            i = np.array(list(i))
        masks.append(cv2.inRange(hsv, *pad(i, tolllerance)))
        #masks[x] = cv2.erode(masks[x], None, iterations=1)
        masks[x] = cv2.dilate(masks[x], kernal, iterations=1)
        #masks[x] = cv2.morphologyEx(masks[x], cv2.MORPH_CLOSE,kernal)
    return frame, masks


def findCentre(frame, mask,minradius=100):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    center = None
    scale = 1
    centerPoint = None
    for k in range(np.shape(contours)[0]):
        c = contours[k][:]
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"]!=0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
          center = (int(x),int(y))
        if radius > minradius:
            cv2.circle(frame, center, int(60*scale), CENTERCIRCLECOLOUR, 10*scale)
            cv2.circle(frame, center, int(10*scale), CENTERCIRCLECOLOUR, -1)
            centerPoint = [center[0], center[1]]
    cv2.drawContours(frame, contours, -1, CONTOURCOLOUR, 3)
    return(frame, centerPoint)
def findCentreLargest(frame,mask):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        # draw in blue the contours that were founded
        cv2.drawContours(frame, contours, -1, 255, 3)

        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)

        # draw the biggest contour (c) in green
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        center = (int(x+(w/2)),int(y+w/2))
        cv2.circle(frame, center, int(10), (0, 255, 255), -1)
    else:
        center = None
    return frame, center

def findCentreOfCirle(blackandwhite,mask =None):
    if not mask is None:
        output = cv2.bitwise_and(blackandwhite, blackandwhite, mask=mask)
        cv2.imshow("and",output)
    blackandwhite =  cv2.resize(blackandwhite,None,fx = 0.25,fy = 0.25,interpolation = cv2.INTER_AREA)
    blackandwhite = cv2.medianBlur(blackandwhite,5)
    colour = cv2.cvtColor(blackandwhite,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(blackandwhite,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=30)
    if not circles is None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(colour,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(colour,(i[0],i[1]),2,(0,0,255),3)
    colour =  cv2.resize(colour,None,fx = 4,fy = 4,interpolation = cv2.INTER_CUBIC)
    return colour ,(0,0)


def findRects(frame, mask):
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        cnt = contours[0]
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        for i in contours:
            rect = cv2.minAreaRect(i)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            frame = cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
    return frame
def distanceBetween(point1,point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(np.linalg.norm(point1-point2))

def scalePoint(point:np.array,scalefactor,origin=(0,0)):
    return (scalefactor*(point-origin))+origin

def getCameraIndexes():
    # checks the first 10 indexes.
    arr = []
    for i in range(MAXCAMERAS):
        cap = cv2.VideoCapture(i,cv2.CAP_DSHOW)# suppresses warings
        if cap.isOpened():
            arr.append(i)
        cap.release()
    return arr

# based off https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
def rotatePoints(points, angle=0, origin=(0, 0), anlgleunit="Radians"):
    if anlgleunit == "Degrees":
        angle = np.deg2rad(angle)
    rotationmatrix = np.array([[np.cos(angle), -np.sin(angle)],
                               [np.sin(angle),  np.cos(angle)]])
    origin = np.atleast_2d(origin)
    points = np.atleast_2d(points)
    return np.squeeze((rotationmatrix @ (points.T-origin.T) + origin.T).T)

def rotatedRectWithMaxArea(w, h, angle): #from https://stackoverflow.com/questions/16702966/rotate-image-and-crop-out-black-borders/16778797#16778797
    """
    Given a rectangle of size wxh that has been rotated by 'angle' (in
    radians), computes the width and height of the largest possible
    axis-aligned rectangle (maximal area) within the rotated rectangle.
    """
    if w <= 0 or h <= 0:
        return 0,0
    width_is_longer = w >= h
    side_long, side_short = (w,h) if width_is_longer else (h,w)

    # since the solutions for angle, -angle and 180-angle are all the same,
    # if suffices to look at the first quadrant and the absolute values of sin,cos:
    sin_a, cos_a = abs(np.sin(angle)), abs(np.cos(angle))
    if side_short <= 2.*sin_a*cos_a*side_long or abs(sin_a-cos_a) < 1e-10:
        # half constrained case: two crop corners touch the longer side,
        #   the other two corners are on the mid-line parallel to the longer line
        x = 0.5*side_short
        wr,hr = (x/sin_a,x/cos_a) if width_is_longer else (x/cos_a,x/sin_a)
    else:
        # fully constrained case: crop touches all 4 sides
        cos_2a = cos_a*cos_a - sin_a*sin_a
        wr,hr = (w*cos_a - h*sin_a)/cos_2a, (h*cos_a - w*sin_a)/cos_2a

    return wr,hr

# stuff i tried to get rotation before just using 3 masks of circles


def findCorners(frame):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = np.float32(grey)
    dst = cv2.cornerHarris(grey, 2, 3, 0.04)
    # result is dilated for marking the corners, not important
    dst = cv2.dilate(dst, None)
    # Threshold for an optimal value, it may vary depending on the image.
    frame[dst > 0.01*dst.max()] = [0, 0, 255]
    return frame


def findGoodFeatures(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
    corners = np.int0(corners)

    for i in corners:
        x, y = i.ravel()
        cv2.circle(frame, (x, y), 3, 255, -1)
    return frame


def findORB(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create()
    kp = orb.detect(gray, None)
    kp, des = orb.compute(gray, kp)
    frame = cv2.drawKeypoints(gray, kp, None, color=(0, 255, 0), flags=0)
    return frame


def findImage(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img, None)
    kp2, des2 = orb.detectAndCompute(gray, None)
    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1, des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)

    # Draw first 10 matches.
    img3 = cv2.drawMatches(
        img, kp1, gray, kp2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return frame, img3


def findImageByMatch(frame):
    frame2 = frame.copy()
    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(frame2, img, cv2.TM_CCOEFF)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = min_loc
    w = 0
    h = 0
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(frame, top_left, bottom_right, 255, 2)
    return frame


def findLines(frame):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grey, 40, 160, apertureSize=3)
    return frame, edges
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name


img = cv2.imread('Orieenter.png', 0)
# cv2.imshow('image',img)


if __name__ == "__main__":
    cap = cv2.VideoCapture('BW_test.mp4')
    # Check if camera opened successfully
    if (cap.isOpened() == False):
        print("Error opening video stream or file")

    img = cv2.imread('Orieenter.png', 0)
    # Read until video is completed
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # print(frame.size.as_integer_ratio())
        if ret == True:
            frame, mask = maskImage(frame)
            frame, centerpoint = findCentre(frame, mask)
            # Display the resulting frame
            #frame,orbs = findImage(frame)
            #frame = findRects(frame,mask)
            frame = cv2.resize(frame, None, fx=0.5, fy=0.5,
                               interpolation=cv2.INTER_CUBIC)
            mask = cv2.resize(mask, None, fx=0.25, fy=0.25,
                              interpolation=cv2.INTER_CUBIC)
            #orbs = cv2.resize(orbs,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
            cv2.imshow('Frame', frame)
            cv2.imshow('Mask', mask)
            print(centerpoint)
            # cv2.imshow("Orbs",orbs)
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Break the loop
        else:
            break

    # When everything done, release the video capture object
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()
