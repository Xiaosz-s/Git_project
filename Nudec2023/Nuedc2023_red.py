from machine import Pin
import sensor, image, time,utime
from machine import UART
import seekfree, pyb
uart = UART(1, baudrate=115200)
#uart.write("666\r\n")
# 初始化TFT180屏幕
#lcd = seekfree.LCD180(3)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 设置图像色彩格式为RGB565格式
sensor.set_framesize(sensor.QQVGA)  # 设置图像大小为160*120
sensor.set_auto_whitebal(False)      # 设置自动白平衡
sensor.set_auto_gain(False) #关闭自动增益
sensor.set_auto_exposure(False)
#sensor.set_colorbar(True)
#sensor.set_quality(0)     #质量
#sensor.set_saturation(-3) #饱和度
#sensor.set_contrast(-3)   #对比度

sensor.set_brightness(400)         # 设置亮度为3000
sensor.skip_frames(time = 20)       # 跳过帧

clock = time.clock()

myrect_x=0
myrect_y=0
myrect_w=0
myrect_h=0
mypoint_x=0
mypoint_y=0
rectpoint1_x=0
rectpoint1_y=0
rectpoint2_x=0
rectpoint2_y=0
rectpoint3_x=0
rectpoint3_y=0
rectpoint4_x=0
#rectpoint4_y=0

ROI=(42,8,110,110)
#(89, 100, -128, -10, -128, 127)
while(True):
    clock.tick()
    img = sensor.snapshot()
    statistics=img.get_statistics(roi=ROI)
    L_max=statistics.l_max()
    #green_td=[( 89, 100, -128, -10, -128, 127)]
    #print(L_max)
    if L_max==100:
        red_td = [(100, 100, -128, 127, -128, 127)]
    else:
        red_td = [(49, 100, 21, 127, -128, 127)]

    #for b in img.find_blobs(green_td,pixels_threshold=1, area_threshold=1, merge=True,invert = 0):
        ## 在屏幕上画出色块
        #img.draw_rectangle(b.rect(), color = (0, 255, 0), scale = 2, thickness = 2)
    for b in img.find_blobs(red_td,pixels_threshold=1, area_threshold=1, merge=True,invert = 0):
        # 在屏幕上画出色块
        img.draw_rectangle(b.rect(), color = (255, 0, 0), scale = 2, thickness = 2)
        #print(b.rect())
        mypoint_x=b.x() + b.w()/2
        mypoint_y=b.y() + b.h()/2
        mypoint_x=int(mypoint_x)
        mypoint_y=int(mypoint_y)
        break
    # -----矩形框部分-----
    # 在图像中寻找矩形
    for r in img.find_rects(ROI,threshold = 10000):
        # 判断矩形边长是否符合要求
        #if r.w() < 50 and r.h() < 50 and r.h() > 10 and r.w() > 10:

        if r.h() > 25 and r.w() > 25 and r.w() < 150 and r.h() < 150 and r.magnitude()>25000:
            print(r.magnitude())
            # 在屏幕上框出矩形
            img.draw_rectangle(r.rect(), color = (255, 0, 0), scale = 4)
            # 获取矩形角点位置
            corner = r.corners()
            # 在屏幕上圈出矩形角点
            img.draw_circle(corner[0][0], corner[0][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            img.draw_circle(corner[1][0], corner[1][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            img.draw_circle(corner[2][0], corner[2][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            img.draw_circle(corner[3][0], corner[3][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            Rect_cx=(corner[0][0]+corner[1][0]+corner[2][0]+corner[3][0])/4
            Rect_cy=(corner[0][1]+corner[1][1]+corner[2][1]+corner[3][1])/4
            rectpoint1_x=int(corner[0][0]+(Rect_cx-corner[0][0])/24)
            rectpoint1_y=int(corner[0][1]+(Rect_cy-corner[0][1])/12)
            rectpoint2_x=int(corner[1][0]+(Rect_cx-corner[1][0])/24)
            rectpoint2_y=int(corner[1][1]+(Rect_cy-corner[1][1])/12)
            rectpoint3_x=int(corner[2][0]+(Rect_cx-corner[2][0])/24)
            #rectpoint3_y=int(corner[2][1]+(Rect_cy-corner[2][1])/12)3\
            rectpoint4_x=int(corner[3][0]+(Rect_cx-corner[3][0])/24)
            rectpoint4_y=int(corner[3][1]+(Rect_cy-corner[3][1])/12)
            #print(Rect_cx,Rect_cy)

    #utime.sleep_ms()
    str_r=bytearray([0x11,0x12, rectpoint1_x,rectpoint1_y,rectpoint2_x,
    rectpoint2_y,rectpoint3_x,rectpoint3_y,rectpoint4_x,
    rectpoint4_y,mypoint_x,mypoint_y])
    uart.write(str_r)
    #print(mypoint_x,mypoint_y)

    #print(rectpoint1_x,rectpoint1_y,rectpoint2_x,rectpoint2_y,rectpoint3_x,rectpoint3_y,rectpoint4_x,rectpoint4_y)
