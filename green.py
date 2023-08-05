import sensor, image, time
from pyb import UART

uart = UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)

red_threshold = (7, 100, 19, 127, -128, 127) # 红色激光笔的阈值范围
green_threshold = (24, 100, -128, -10, -128, 127) # 绿色激光笔的阈值范围
roiRect = (270, 100, 350, 350)

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

def SendTwoLaser(ax, ay, bx, by):
    byteData = bytearray([0x12, 0x34, 8, 2,
    (ax >> 8) & 0xff, ax & 0xff,
    (ay >> 8) & 0xff, ay & 0xff,
    (bx >> 8) & 0xff, bx & 0xff,
    (by >> 8) & 0xff, by & 0xff])
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
    sensor.skip_frames(time = 200)

def DefaultTick():
    img = sensor.snapshot()
    img.lens_corr(strength = 1.8, zoom = 1.0)
    img.draw_rectangle(roiRect, color = (0, 255, 0))

def ResetForDetectTwoLaser():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_auto_exposure(False, 3000)
    sensor.skip_frames(time = 200)

def DetectTwoLaser():
    img = sensor.snapshot() # 获取一帧图像
    img.lens_corr(strength = 1.8, zoom = 1.0)
    img.draw_rectangle(roiRect, color = (255, 255, 255))
    red_blobs = img.find_blobs([red_threshold], roi = roiRect, x_stride = 2, y_stride = 2) # 在图像中查找红色区域
    green_blobs = img.find_blobs([green_threshold], roi = roiRect, x_stride = 2, y_stride = 2) # 在图像中查找绿色区域

    redX = 0
    redY = 0
    greenX = 0
    greenY = 0

    if red_blobs:
        for blob in red_blobs:
            img.draw_rectangle(blob.rect()) # 在图像上绘制红色区域的矩形框
            img.draw_cross(blob.cx(), blob.cy()) # 在图像上绘制红色区域的中心点
            redX = blob.cy()
            redY = blob.cx()
            print("red " + str(redX) + " " + str(redY))
    if green_blobs:
         for blob in green_blobs:
            img.draw_rectangle(blob.rect()) # 在图像上绘制绿色区域的矩形框
            img.draw_cross(blob.cx(), blob.cy()) # 在图像上绘制绿色区域的中心点
            greenX = blob.cy()
            greenY = blob.cx()
            print("green " + str(greenX) + " " + str(greenY))
    return [redX, redY, greenX, greenY]

def UpdateStatus(cmd):
    global cmdStatus
    if cmdStatus == cmd:
        return None
    cmdStatus = cmd
    if cmd == 0x00:
        ResetForDefault()
    elif cmd == 0x03:
        ResetForDetectTwoLaser()


ResetForDefault()

while(True):
    cmd = UartReadCmd()
    if cmd != None:
        UpdateStatus(cmd)
    if cmdStatus == 0x00:
        ret = DefaultTick()
    elif cmdStatus == 0x03:
        ret = DetectTwoLaser()
        if ret != None and len(ret) != 0:
            SendTwoLaser(ret[0], ret[1], ret[2], ret[3])
