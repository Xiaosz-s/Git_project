#整个视觉代码仅用来识别圆和物料颜色
#注意：未识别色环对应颜色，色环颜色是固定的，由于视觉总会受光照影响所以仅可能将视觉承受压力缩小



import cv2
import numpy as np
from sklearn.cluster import KMeans
import serial

ser = serial.Serial("/dev/ttyS0",115200)

def Is_within_range(constant,min_val,max_val):
    return min_val <= constant <= max_val
    
def nothing(x):
    pass

def ColorDetector(img, lower=np.array([20, 50, 46]), upper=np.array([60, 255, 255])):
    Hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(Hsv, lower, upper)
    res = cv2.bitwise_and(img, img, mask=mask_green)
    return res

def get_dominant_color(image, k=3):
    image = image.reshape((image.shape[0] * image.shape[1], 3))
    kmeans = KMeans(n_clusters=k)
    kmeans.fit(image)
    dominant_colors = kmeans.cluster_centers_
    return dominant_colors


cap = cv2.VideoCapture(0)

desired_width = 176
desired_height = 144

cap.set(cv2.CAP_PROP_FRAME_WIDTH,desired_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,desired_height)



cv2.createTrackbar('H_Low', 'image', 0, 255, nothing)
cv2.createTrackbar('H_High', 'image', 0, 255, nothing)

cv2.createTrackbar('S_Low', 'image', 0, 255, nothing)
cv2.createTrackbar('S_High', 'image', 0, 255, nothing)

cv2.createTrackbar('V_Low', 'image', 0, 255, nothing)
cv2.createTrackbar('V_High', 'image', 0, 255, nothing)


lower_green = np.array([44, 99, 108])
upper_green = np.array([85, 220, 255])

lower_red_1 = np.array([0, 50, 50])
upper_red_1 = np.array([10, 255, 255])

lower_red_2 = np.array([160, 50, 50])
upper_red_2 = np.array([179, 255, 255])

lower_blue = np.array([75, 100, 100])
upper_blue = np.array([155, 255, 255])

lower_greenc = np.array([33, 46, 46])
upper_greenc = np.array([85, 255, 255])

lower_redc_1 = np.array([0, 50, 50])
upper_redc_1 = np.array([10, 255, 255])

lower_redc_2 = np.array([160, 50, 50])
upper_redc_2 = np.array([179, 255, 255])

lower_bluec = np.array([100, 46, 46])
upper_bluec = np.array([155, 255, 255])

kernel = np.ones((3,3),np.uint8)
kernel1= np.ones((15,15),np.uint8)

min_contour_area = 600
center_x=0
center_y=0

greenS=0
redS=0
blueS=0

greenX=0
greenY=0

redX=0
redY=0

blueX=0
blueY=0

Send_color=0
sendX=0
sendY=0

while True:
    Send_color=0
    sendX=0
    sendX=0
    ret, frame = cap.read()
    
    if not ret:
        continue
    
    eroded_frame=frame
    dilate_frame = cv2.dilate(eroded_frame, kernel1)
    #eroded_frame = cv2.erode(eroded_frame, kernel)
    #eroded_frame = cv2.erode(eroded_frame, kernel)
    #eroded_frame = cv2.erode(eroded_frame, kernel)
    
    img = cv2.cvtColor(eroded_frame, cv2.COLOR_BGR2GRAY)

    hsv = cv2.cvtColor(dilate_frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blueS=0
    greenS=0
    redS=0
    for contour in contours:
    
        area = cv2.contourArea(contour)
        
        if area > min_contour_area:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 255), 2)
            greenS=w*h
            greenX=int(x+w/2)
            greenY=int(y+h/2)
        
    mask = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask1 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask,mask1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
    
        area = cv2.contourArea(contour)
        
        if area > min_contour_area:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 255), 2)
            redS=w*h
            redX=int(x+w/2)
            redY=int(y+h/2)
            
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
    
        area = cv2.contourArea(contour)
        
        if area > min_contour_area:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 255), 2)
            blueS=w*h
            blueX=int(x+w/2)
            blueY=int(y+h/2)
            
    if(blueS>=redS and blueS>=greenS and blueS>=600):
        Send_color=3
        sendX=blueX
        sendY=blueY
    elif(redS>=blueS and redS>=greenS and redS>=600):
        Send_color=1
        sendX=redX
        sendY=redY
    elif(greenS>=redS and greenS>=blueS and greenS>=600):
        Send_color=2
        sendX=greenX
        sendY=greenY
    
    maskc = cv2.inRange(hsv, lower_redc_1, upper_redc_1)
    mask1c = cv2.inRange(hsv, lower_redc_2, upper_redc_2)
    Redc_mask = cv2.bitwise_or(maskc,mask1c)
    
    Greenc_mask = cv2.inRange(hsv, lower_greenc, upper_greenc)
    
    Bluec_mask = cv2.inRange(hsv, lower_bluec, upper_bluec)
    
    dilated_frame = cv2.dilate(Redc_mask, kernel)
    dilated_frame = cv2.dilate(dilated_frame, kernel)
    dilated_frame = cv2.dilate(dilated_frame, kernel)
    Red_img = dilated_frame
    
    dilated_frame = cv2.dilate(Greenc_mask, kernel)
    dilated_frame = cv2.dilate(dilated_frame, kernel)
    dilated_frame = cv2.dilate(dilated_frame, kernel)
    Green_img = dilated_frame
    
    dilated_frame = cv2.dilate(Bluec_mask, kernel)
    dilated_frame = cv2.dilate(dilated_frame, kernel)
    dilated_frame = cv2.dilate(dilated_frame, kernel)
    Blue_img = dilated_frame
     #eroded_frame = cv2.erode(Redc_mask, kernel)
    eroded_frame = cv2.erode(frame, kernel)
    eroded_frame = cv2.erode(eroded_frame, kernel)
    img=eroded_frame
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=50, minRadius=10, maxRadius=40)# x y r
    
    Circle_color=0
    
    if circles is not None:
        for i in circles[0, : ]:
            cv2.circle(img, (i[0], i[1]), 2, (255, 255, 255), 2)
            
            i[0]=int(i[0])
            i[1]=int(i[1])
            i[2]=int(i[2])
            diff=int(i[2]*1.1)
            center_x = int(i[0])
            center_y = int(i[1])
            if(Is_within_range(center_y+diff,0,143) and Is_within_range(center_x+diff,0,175)):
                Red_value=Red_img[center_y+diff,center_x+diff]
                Green_value=Green_img[center_y+diff,center_x+diff]
                Blue_value=Blue_img[center_y+diff,center_x+diff]
            else:
                Red_value=Red_img[center_y-diff,center_x-diff]
                Green_value=Green_img[center_y-diff,center_x-diff]
                Blue_value=Blue_img[center_y-diff,center_x-diff]
                
            Circle_color=4
            #print(Red_value,Green_value,Blue_value)
            #print(Red_img[center_y,center_x],Green_img[center_y,center_x],Blue_img[center_y,center_x])
            if(Red_value==255):
                Circle_color=1
            elif(Green_value==255):
                Circle_color=2
            elif(Blue_value==255):
                Circle_color=3
                    
            print(Circle_color)
            
    #cv2.imshow('Redc_mask', Redc_mask)
    #cv2.imshow('Greenc_mask', Greenc_mask)
    #cv2.imshow('Bluec_mask', Bluec_mask)
    #cv2.imshow('Red_img', Red_img) 
    #cv2.imshow('Blue_img', Blue_img)
    #cv2.imshow('Green_img', Green_img)
    #cv2.imshow('img', img)
    mystr=bytearray([0xff,Circle_color,center_x,center_y,Send_color,sendY,sendX])
    print(Circle_color,center_x,center_y,Send_color,sendY,sendX)
    ser.write(mystr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


