#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import time

# =========================================
# INICIALIZAÇÃO DO NÓ ROS
# =========================================
rospy.init_node('script_comando_com_captura', anonymous=True)

pub_joints = rospy.Publisher('/kuka_cmd', Float64MultiArray, queue_size=10)
pub_capture = rospy.Publisher('/capture_realsense', Bool, queue_size=10)

rospy.sleep(1.0)  # Espera conexão com tópicos

# =========================================
# CONFIGURAÇÃO DE MOVIMENTO
# =========================================
joint_names = [
    'LBR_iiwa_14_R820_joint1',
    'LBR_iiwa_14_R820_joint2',
    'LBR_iiwa_14_R820_joint3',
    'LBR_iiwa_14_R820_joint4',
    'LBR_iiwa_14_R820_joint5',
    'LBR_iiwa_14_R820_joint6',
    'LBR_iiwa_14_R820_joint7'
]

num_poses = int(input("Quantas poses você deseja passar ao robô? "))
time_per_pose = float(input("Quanto tempo o robô deve permanecer em cada pose (em segundos)? "))

poses = []
for i in range(num_poses):
    print(f"\n🔢 Pose {i + 1}:")
    angles = []
    for j in range(len(joint_names)):
        angle = float(input(f" - Ângulo para {joint_names[j]} (em graus): "))
        angles.append(np.radians(angle))  # Converte para radianos
    poses.append(angles)

# =========================================
# ENVIO DAS POSES E SOLICITAÇÃO DE CAPTURA
# =========================================
for pose_idx, pose in enumerate(poses):
    print(f"\n➡️ Publicando pose {pose_idx + 1} via ROS...")
    msg = Float64MultiArray()
    msg.data = pose
    pub_joints.publish(msg)
    rospy.sleep(time_per_pose)
    print(f"✅ Pose {pose_idx + 1} concluída.")

    resposta = input("📸 Deseja capturar imagem da Realsense nesta pose? (s/n): ").strip().lower()

    if resposta == 's':
        pub_capture.publish(Bool(data=True))
        print("🟢 Sinal de captura publicado no tópico /capture_realsense.")
        time.sleep(4)
        rospy.sleep(2.0)  # Aguarda tempo suficiente para o Coppelia salvar
    else:
        print("❌ Nenhuma imagem encontrada.")
else:
    print(" Execução de nó Python finalizada")

print("\n🏁 Sequência finalizada.")
