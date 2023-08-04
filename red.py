import sensor, image, time
from pyb import UART

uart = UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)

red_threshold = (8, 100, 8, 127, -128, 127) # 红色激光笔的阈值范围
green_threshold = (24, 100, -128, -10, -128, 127) # 绿色激光笔的阈值范围

cmdStatus = 0x00 #Stop

def SendPair(first, second):
    byteData = bytearray([0x12, 0x34, 4, 1, (first >> 8) & 0xff, first & 0xff, (second >> 8) & 0xff, second & 0xff])
    uart.write(byteData)

def SendRect(ax, ay, bx, by, cx, cy, dx, dy):
    byteData = bytearray([0x12, 0x34, 16, 2,
    (ax >> 8) & 0xff, ax & 0xff,
    (ay >> 8) & 0xff, ay & 0xff,
    (bx >> 8) & 0xff, bx & 0xff,
    (by >> 8) & 0xff, by & 0xff,
    (cx >> 8) & 0xff, cx & 0xff,
    (cy >> 8) & 0xff, cy & 0xff,
    (dx >> 8) & 0xff, dx & 0xff,
    (dy >> 8) & 0xff, dy & 0xff,])
    uart.write(byteData)

def UartReadCmd():
    len = uart.any()
    if len == 0:
        return None
    cmd = 0x00
    while len > 0:
        cmd = uart.readchar()
        len -= 1
    print("cmd:", cmd)
    return cmd

def ResetForDefault():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((280, 120, 280, 310))
    sensor.skip_frames(time = 200)

def DefaultTick():
    img = sensor.snapshot()
    #img.lens_corr(strength = 1.5, zoom = 1.0)

def ResetForDetectRedLaser():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((280, 120, 280, 310))
    sensor.set_auto_exposure(False, 3000)
    sensor.skip_frames(time = 200)

def DetectRedLaser():
    img = sensor.snapshot()
    #img.lens_corr(strength = 1.8, zoom = 1.0)
    red_blobs = img.find_blobs([red_threshold]) # 在图像中查找红色区域
    if red_blobs:
        for blob in red_blobs:
            if blob.area() > 80:
                continue
            img.draw_rectangle(blob.rect()) # 在图像上绘制红色区域的矩形框
            img.draw_cross(blob.cx(), blob.cy()) # 在图像上绘制红色区域的中心点
            #print("x:" + str(blob.cx()) + " y:" + str(blob.cy()))
            return [blob.cy(), blob.cx()]
    return None

def ResetForDetectRect():
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((280, 120, 280, 310))
    sensor.skip_frames(time = 200)

def DetectRect():
    img = sensor.snapshot()
    #img.lens_corr(strength = 1.8, zoom = 1.0)

    lst = []
    for r in img.find_rects(threshold = 29000):
        img.draw_rectangle(r.rect(), color = (255, 0, 0))
        for p in r.corners():
            img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))
            lst.append(p[1])
            lst.append(p[0])
    return lst

def UpdateStatus(cmd):
    global cmdStatus
    if cmdStatus == cmd:
        return None
    cmdStatus = cmd
    if cmd == 0x00:
        ResetForDefault()
    elif cmd == 0x01:
        ResetForDetectRedLaser()
    elif cmd == 0x02:
        ResetForDetectRect()


ResetForDefault()

while(True):
    cmd = UartReadCmd()
    if cmd != None:
        UpdateStatus(cmd)
    if cmdStatus == 0x00:
        ret = DefaultTick()
    elif cmdStatus == 0x01:
        ret = DetectRedLaser()
        if ret:
            SendPair(ret[0], ret[1])
    elif cmdStatus == 0x02:
        ret = DetectRect()
        if ret != None and len(ret) != 0:
            SendRect(ret[0], ret[1], ret[2], ret[3], ret[4], ret[5], ret[6], ret[7])
