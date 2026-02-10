#!/usr/bin/env python3

import os
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from kuka_ros_node import *
import time

client = kuka_iiwa_ros_node()  # Making a connection object.
model_weights = os.path.expanduser("~/flymov/catkin_ws/src/kuka_controller/best2.pt")

# Função para converter ângulos de Euler (ZYX) em matriz de rotação 3x3
def euler_to_rotation_matrix(a, b, c):
    a, b, c = np.radians([a, b, c])
    R_x = np.array([[1, 0, 0],
                     [0, np.cos(a), -np.sin(a)],
                     [0, np.sin(a), np.cos(a)]])
    R_y = np.array([[np.cos(b), 0, np.sin(b)],
                     [0, 1, 0],
                     [-np.sin(b), 0, np.cos(b)]])
    R_z = np.array([[np.cos(c), -np.sin(c), 0],
                     [np.sin(c), np.cos(c), 0],
                     [0, 0, 1]])
    return R_z @ R_y @ R_x

# Função para calcular matriz de rotação a partir de dois pontos

def compute_rotation_matrix(p_center, p_top):
    z_axis = np.array([p_top[0] - p_center[0], p_top[1] - p_center[1], p_top[2] - p_center[2]])
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.array([1, 0, 0])  # Assume-se que o eixo x seja fixo
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    R = np.column_stack((x_axis, y_axis, z_axis))
    return R

# CONFIGURAÇÃO DA CÂMERA REALSENSE
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# MATRIZ FIXA DA CÂMERA EM RELAÇÃO À FLANGE
T_cam_flan = np.array([
    [0, -1,  0, 0.03428],
    [1,  0,  0, 0.09858],
    [0,  0,  1, 0.0338],
    [0,  0,  0, 1]
])

# CONFIGURAÇÃO DO YOLO
model = YOLO(model_weights)

print("Iniciando detecção em tempo real")

try:
    profile = pipeline.start(config)
   
    # Criar janela OpenCV
    cv2.namedWindow("Detecção em Tempo Real", cv2.WINDOW_NORMAL)

    while True:
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        fx, fy, cx, cy = intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy

        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        results = model.predict(frame, verbose=False)

        # Atualiza a matriz de transformação do bagageiro para a câmera em tempo real
        tool_position = client.ToolPosition[0]
        tool_position[:3] = [x / 1000 for x in tool_position[:3]]
        flange_pose = list(tool_position[:3])
        T_world_flan = np.eye(4)
        T_world_flan[:3, 3] = flange_pose

        for result in results:
            for box in result.boxes:
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = box.conf[0]
                class_name = model.names[class_id]

                if class_name == "OverheadBin":
                    x_center = int((x_min + x_max) / 2)
                    y_center = int((y_min + y_max) / 2)
                    x_top = int((x_min + x_max) / 2)
                    y_top = y_min
                   
                    distance_center = depth_frame.get_distance(x_center, y_center)
                    distance_top = depth_frame.get_distance(x_top, y_top)
                   
                    p_center = np.array([(x_center - cx) * distance_center / fx,
                                         (y_center - cy) * distance_center / fy,
                                         distance_center])
                   
                    p_top = np.array([(x_top - cx) * distance_top / fx,
                                      (y_top - cy) * distance_top / fy,
                                      distance_top])
                   
                    R_bag_cam = compute_rotation_matrix(p_center, p_top)
                   
                    T_bag_cam = np.eye(4)
                    T_bag_cam[:3, :3] = R_bag_cam
                    T_bag_cam[:3, 3] = p_center
                   
                    T_bag_flan = np.dot(T_bag_cam, T_cam_flan)
                    T_bag_world = np.dot(T_world_flan, T_bag_flan)
                   
                    print("Matriz de transformação do bagageiro para a câmera atualizada:")
                    print(T_bag_cam)

        cv2.imshow("Detecção em Tempo Real", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("Execução interrompida pelo usuário.")
except rs.error as e:
    print(f"Erro RealSense: {e}")
except Exception as e:
    print(f"Erro geral: {e}")
finally:
    pipeline.stop()
    cv2.destroyAllWindows()