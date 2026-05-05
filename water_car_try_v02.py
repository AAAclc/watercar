from paddleocr import PaddleOCR
import cv2
import serial
import time

# ========== 配置 ==========
SERIAL_PORT = 'COM7'  # 根据实际修改
BAUDRATE = 115200
OCR_INTERVAL = 2.0
SEND_TIMEOUT = 2.0
MAX_RETRIES = 2
SCALE = 1000
COORD_REGION_1 = (0.8, 0.0)
COORD_REGION_2 = (0.8, 0.8)
# =========================

print("加载OCR模型...")
ocr = PaddleOCR(lang="ch")

# 初始化串口
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.5)
    print(f"视觉端已打开 {SERIAL_PORT}")
except Exception as e:
    print(f"无法打开 {SERIAL_PORT}: {e}")
    exit()

# 初始化摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

def parse_target(ocr_result):
    try:
        res_dict = ocr_result[0]
        full_text = "".join(res_dict.get('rec_texts', []))
        print(f"识别文字: {full_text}")
        if any(kw in full_text for kw in ['二', '2', '次']):
            return 2
        if any(kw in full_text for kw in ['一', '1', '首']):
            return 1
        return None
    except:
        return None

def send_coordinates(x, y):
    x_int = int(round(x * SCALE))
    y_int = int(round(y * SCALE))
    x_int = max(0, min(65535, x_int))
    y_int = max(0, min(65535, y_int))
    # 协议: AA 55 XH XL YH YL (大端)
    frame = bytes([0xAA,
                   0x55,
                   (x_int >> 8) & 0xFF, x_int & 0xFF,
                   (y_int >> 8) & 0xFF, y_int & 0xFF])
    ser.write(frame)
    print(f"发送位置: ({x},{y})m -> X={x_int}mm Y={y_int}mm")

def send_yaw(theta_rad):
    """发送航向角指令: AA 56 TH TL, theta 单位 rad"""
    theta_mrad = int(round(theta_rad * 1000))
    theta_mrad = max(-32768, min(32767, theta_mrad))
    frame = bytes([0xAA,
                   0x56,
                   (theta_mrad >> 8) & 0xFF, theta_mrad & 0xFF])
    ser.write(frame)
    print(f"发送航向: {theta_rad:.3f}rad ({theta_mrad}mrad)")

def send_command_with_feedback(target):
    if target == 1:
        x, y = COORD_REGION_1
    else:
        x, y = COORD_REGION_2

    for retry in range(MAX_RETRIES):
        send_coordinates(x, y)
        # 等待电控确认 'K'
        start = time.time()
        while time.time() - start < SEND_TIMEOUT:
            if ser.in_waiting:
                resp = ser.read().decode(errors='ignore')
                if resp == 'K':
                    print("收到确认 K — 电控已收到指令")
                    return True
                elif resp == 'D':
                    print("收到 D — 已到达目标点")
                    return True
        print("确认超时，重试")
    return False

print("s:OCR识别  y:设航向  q:退出")
last_ocr = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.putText(frame, "s:Scan y:Yaw q:Quit", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Vision", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('y'):
        try:
            deg = float(input("输入航向角(度, 正值=左转): "))
            theta_rad = deg * 3.14159265 / 180.0
            send_yaw(theta_rad)
            # 等待确认
            start = time.time()
            while time.time() - start < SEND_TIMEOUT:
                if ser.in_waiting:
                    resp = ser.read().decode(errors='ignore')
                    if resp == 'K':
                        print("航向指令已确认")
                        break
        except ValueError:
            print("输入无效")

    if key == ord('s'):
        if time.time() - last_ocr < OCR_INTERVAL:
            print("请稍后再试")
            continue
        last_ocr = time.time()
        print("识别中...")
        cv2.imwrite("_temp.jpg", frame)
        try:
            result = ocr.predict("_temp.jpg")
            target = parse_target(result)
            if target:
                print(f"目标区域: {target}")
                success = send_command_with_feedback(target)
                print("任务完成" if success else "任务失败")
            else:
                print("未识别到有效指令")
        except Exception as e:
            print(f"错误: {e}")

cap.release()
cv2.destroyAllWindows()
ser.close()