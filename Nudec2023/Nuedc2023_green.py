from machine import Pin
import sensor, image, time,utime
from machine import UART
import seekfree, pyb
uart = UART(2, baudrate=115200)
#uart.write("666\r\n")
# 初始化TFT180屏幕
#lcd = seekfree.LCD180(3)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 设置图像色彩格式为RGB565格式
sensor.set_framesize(sensor.QQVGA)  # 设置图像大小为160*120
sensor.set_auto_whitebal(True)      # 设置自动白平衡
sensor.set_auto_gain(True) #关闭自动增益
sensor.set_auto_exposure(True)
#sensor.set_colorbar(True)
#sensor.set_quality(0)     #质量
#sensor.set_saturation(-3) #饱和度
#sensor.set_contrast(-3)   #对比度

sensor.set_brightness(140)         # 设置亮度为3000
sensor.skip_frames(time = 20)       # 跳过帧

clock = time.clock()

myrect_x=0
myrect_y=0
myrect_w=0
myrect_h=0
mypoint_x=0
mypoint_y=0
mygpoint_x=0
mygpoint_y=0

ROI=(0,0,160,120)
#(89, 100, -128, -10, -128, 127)
while(True):
    clock.tick()
    img = sensor.snapshot()
    statistics=img.get_statistics(roi=ROI)
    L_max=statistics.l_max()
    green_td=[(90, 100, -128, -20, -128, 127)]
    #print(L_max)
    #if L_max==100:
    red_td = [(75, 95, 18, 127, -34, 34)]
    #else:
        #red_td = [(49, 100, 21, 127, -128, 127)]

    #for g in img.find_blobs(green_td,pixels_threshold=1, area_threshold=1, merge=True,invert = 0):
        ## 在屏幕上画出色块
        #img.draw_rectangle(g.rect(), color = (0, 0, 0), scale = 2, thickness = 2)
        #mygpoint_x=int(g.cx())
        #mygpoint_y=int(g.cy())
        #break
    for b in img.find_blobs(red_td,pixels_threshold=1, area_threshold=1, merge=True,invert = 0):
        # 在屏幕上画出色块
        img.draw_rectangle(b.rect(), color = (0, 0, 0), scale = 2, thickness = 2)
        #print(b.rect())
        mypoint_x=b.x() + b.w()/2
        mypoint_y=b.y() + b.h()/2
        mypoint_x=int(mypoint_x)
        mypoint_y=int(mypoint_y)
        break
    str_r=bytearray([0x11,0x12,mypoint_x,mypoint_y])
    uart.write(str_r)
    print(mypoint_x,mypoint_y,mygpoint_x,mygpoint_y)

    #print(rectpoint1_x,rectpoint1_y,rectpoint2_x,rectpoint2_y,rectpoint3_x,rectpoint3_y,rectpoint4_x,rectpoint4_y,mypoint_x,mypoint_y)
