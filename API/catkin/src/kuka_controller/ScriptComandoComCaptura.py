#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import time

# =========================
# INICIALIZAÇÃO DO NÓ ROS
# =========================
rospy.init_node('script_comando_com_captura', anonymous=True)
pub_joints = rospy.Publisher('/kuka_cmd', Float64MultiArray, queue_size=10)
pub_capture = rospy.Publisher('/capture_realsense', Bool, queue_size=10)
rospy.sleep(1.0)  # pequena folga para conectar nos tópicos

# =========================
# ENTRADA DE UMA ÚNICA POSE
# =========================
joint_names = [
    'LBR_iiwa_14_R820_joint1',
    'LBR_iiwa_14_R820_joint2',
    'LBR_iiwa_14_R820_joint3',
    'LBR_iiwa_14_R820_joint4',
    'LBR_iiwa_14_R820_joint5',
    'LBR_iiwa_14_R820_joint6',
    'LBR_iiwa_14_R820_joint7'
]

print("\nInforme UMA pose (ângulos em graus):")
angles_deg = []
for jn in joint_names:
    ang = float(input(f" - Ângulo para {jn} [graus]: "))
    angles_deg.append(ang)

pose_rad = [np.radians(a) for a in angles_deg]

# =========================
# PUBLICA A POSE E CAPTURA
# =========================
msg = Float64MultiArray(data=pose_rad)
print("\nPublicando pose...")
pub_joints.publish(msg)

print("Capturando imagem (Realsense)...")
pub_capture.publish(Bool(data=True))

# aguarda gravação (ajuste se precisar)
time.sleep(4)
rospy.sleep(2.0)

print("Captura concluída.")
print("Fim.")

