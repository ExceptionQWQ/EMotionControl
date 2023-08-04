# Untitled - By: zhou - 周四 8月 3 2023

#import sensor, image, time

#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.skip_frames(time = 2000)

#clock = time.clock()

#while(True):
    #clock.tick()
    #img = sensor.snapshot()
    #print(clock.fps())
import sensor, image, time

red_threshold = (8, 100, 8, 127, -128, 127) # 红色激光笔的阈值范围
green_threshold = (24, 100, -128, -10, -128, 127) # 绿色激光笔的阈值范围

sensor.reset() # 重置OpenMV相机
sensor.set_pixformat(sensor.RGB565) # 设置像素格式为RGB565
sensor.set_framesize(sensor.QVGA) # 设置帧大小为QVGA (320x240)
sensor.set_auto_exposure(False, 5000)
sensor.skip_frames(time = 2000) # 跳过一些帧以使摄像头稳定
#listx=[]
#listy=[]
while(True):
    img = sensor.snapshot() # 获取一帧图像
    img.lens_corr(strength = 1.8, zoom = 1.0)
    red_blobs = img.find_blobs([red_threshold]) # 在图像中查找红色区域
    green_blobs = img.find_blobs([green_threshold]) # 在图像中查找绿色区域

    if red_blobs:
        for blob in red_blobs:
            img.draw_rectangle(blob.rect()) # 在图像上绘制红色区域的矩形框
            img.draw_cross(blob.cx(), blob.cy()) # 在图像上绘制红色区域的中心点
            #if(len(listx)<20 and len(listy)<20):
                    #listx.append(blob.cx())
                    #listy.append(blob.cy())
            #else:
                    #del(listx[0])
                    #del(listy[0])
                    #listx.append(blob.cx())
                    #listy.append(blob.cy())
            #new_label = []
            #n = 50000
            #for i in range(n):
              #count_dict = {}
            #for i in listx:
              #if i in count_dict:
                  #count_dict[i] += 1
              #else:
                  #count_dict[i] = 1
            #dictSortList = sorted(count_dict.items(),key = lambda x:x[1], reverse = True)
            #new_label.append(dictSortList[0][0])
    if green_blobs:
         for blob in green_blobs:
            img.draw_rectangle(blob.rect()) # 在图像上绘制绿色区域的矩形框
            img.draw_cross(blob.cx(), blob.cy()) # 在图像上绘制绿色区域的中心点

 #   time.sleep(10) # 等待一段时间
