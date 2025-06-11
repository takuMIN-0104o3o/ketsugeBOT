import pygame
import bettercam
import cv2
import numpy as np
import torch
import pathlib
import threading
import win32api
import hid
import time
from ultralytics import YOLO

# --- 初期設定 ---

pathlib.PosixPath = pathlib.WindowsPath
model_yolo = YOLO('v/scripts/best.pt')
print("✅ YOLOv8 モデルロード完了")

VID = 0x046d
PID = 0xc539
PING_CODE = 0xf9
RELEASE = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

fps = 240
smoothing_factor = 8
fov_radius = 50

width_x, height_y = 416, 416
center_screen = (width_x // 2, height_y // 2)
fov_x_min = center_screen[0] - fov_radius
fov_x_max = center_screen[0] + fov_radius
fov_y_min = center_screen[1] - fov_radius
fov_y_max = center_screen[1] + fov_radius

# --- カメラ ---

camera = bettercam.create(output_idx=0, output_color="BGR")
left = (1920 - width_x) // 2
top = (1080 - height_y) // 2
camera.start(region=(left, top, left + width_x, top + height_y), target_fps=fps)

# --- Pygame 初期化 ---

pygame.init()
screen = pygame.display.set_mode((width_x, height_y))
pygame.display.set_caption("ketsugeBOTv1 Created by takuMIN0104o3o")
fps_font = pygame.font.SysFont("Arial", 18)
clock = pygame.time.Clock()

# --- 状態変数 ---

latest_frame = None
processed_frame = None
closest_target = None
lock = threading.Lock()
running = True
mouse = None
residual = [0, 0]


# --- マウス認識 ---

def find_mouse():
    for info in hid.enumerate(VID, PID):
        dev = hid.device()
        dev.open_path(info['path'])
        dev.write([0, PING_CODE])
        try:
            resp = dev.read(1, 10)
            if resp and resp[0] == PING_CODE:
                print("✅ マウス接続確認済み")
                return dev
        except Exception:
            pass
        dev.close()
    return None


mouse = find_mouse()


# --- マウス移動 ---

def move_mouse(dev, dx, dy, smooth):
    global residual
    move_x = dx / smooth + residual[0]
    move_y = dy / smooth + residual[1]
    dx_i, dy_i = round(move_x), round(move_y)

    residual[0], residual[1] = move_x - dx_i, move_y - dy_i

    dev.write([
        0x01, 0x00,
        dx_i & 0xFF, (dx_i >> 8) & 0xFF,
        dy_i & 0xFF, (dy_i >> 8) & 0xFF,
        0x00
    ])


# --- マウス制御スレッド ---

def mouse_thread():
    global closest_target
    while running:
        dx = 0
        dy = 0
        if win32api.GetAsyncKeyState(0x05) & 0x8000 and closest_target:
            if fov_x_min <= closest_target[0] <= fov_x_max and fov_y_min <= closest_target[1] <= fov_y_max:
                dx = closest_target[0] - center_screen[0]
                dy = closest_target[1] - center_screen[1]
        move_mouse(mouse, dx, dy, smoothing_factor)


# --- 検出スレッド ---

def detection_thread():
    global latest_frame, processed_frame, closest_target
    while running:
        with lock:
            if latest_frame is None:
                continue
            original_frame = latest_frame.copy()

        frame = original_frame.copy()

        results = model_yolo.predict(original_frame, imgsz=416, device=0, verbose=False)[0]

        centers = []
        for box in results.boxes:
            xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy().astype(int)
            label_idx = int(box.cls[0].cpu().numpy())
            label = results.names[label_idx]

            if label == "enemy":
                target_x = (xmin + xmax) // 2
                target_y = ymin + int((ymax - ymin) * 0.145)
            else:
                target_x = (xmin + xmax) // 2
                target_y = ymin + int((ymax - ymin) * 0.9)

            centers.append((target_x, target_y))
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 1)
            cv2.circle(frame, (target_x, target_y), 2, (0, 255, 0), -1)

        if centers:
            closest_target = min(centers,
                                 key=lambda pt: (pt[0] - center_screen[0]) ** 2 + (pt[1] - center_screen[1]) ** 2)
            cv2.line(frame, center_screen, closest_target, (0, 255, 255), 1)
        else:
            closest_target = None

        # --- FOVの円描画 ---
        cv2.circle(frame, center_screen, fov_radius, (255, 255, 255), 1)

        with lock:
            processed_frame = frame


# --- スレッド起動 ---

threading.Thread(target=detection_thread, daemon=True).start()
threading.Thread(target=mouse_thread, daemon=True).start()

# --- メインループ ---

try:
    while running:
        frame = camera.get_latest_frame()
        if frame is None:
            continue
        with lock:
            latest_frame = frame.copy()
            draw_frame = processed_frame.copy() if processed_frame is not None else frame

        frame_rgb = cv2.cvtColor(draw_frame, cv2.COLOR_BGR2RGB)
        surface = pygame.image.frombuffer(frame_rgb.tobytes(), frame_rgb.shape[1::-1], "RGB")
        screen.blit(surface, (0, 0))

        fps_text = fps_font.render(f"FPS: {int(clock.get_fps())}", True, (255, 255, 255))
        screen.blit(fps_text, (10, 10))

        pygame.display.update()
        clock.tick(fps)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

finally:
    running = False
    camera.stop()
    pygame.quit()
