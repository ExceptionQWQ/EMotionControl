import sensor
import image
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

def get_rect_corners(rect):
    """
    接收一个矩形的坐标 (x, y, w, h)，
    返回矩形的四个角点坐标顺序为左上、右上、右下、左下
    """
    x, y, w, h = rect
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]

while True:
    img = sensor.snapshot()  # 捕获图像

    # 使用颜色阈值来识别黑色矩形边框
    thresholds = [(0, 23)]  # 设置颜色阈值（黑色）
    blobs = img.find_blobs(thresholds,roi=(96,78,117,121))

    if blobs:
        max_area = 0
        max_blob = None

        for b in blobs:
            if b.area() > max_area:
                max_area = b.area()
                max_blob = b

        if max_blob:
            # 绘制矩形边框
            img.draw_rectangle(max_blob.rect(), color=(255, 0, 0))
            # 获取矩形边框的四个角点坐标
            corners = get_rect_corners(max_blob.rect())

            # 输出顶点的坐标
            for corner in corners:
                print(corner)
