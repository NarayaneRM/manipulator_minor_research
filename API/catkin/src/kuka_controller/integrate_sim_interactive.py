#!/usr/bin/env python3
from kuka_ros_node import *
from settings import max_t1_speed, max_t2_speed, max_auto_speed, medium_auto_speed, slow_auto_speed
from time import sleep
import sys
import rospy
from std_msgs.msg import Int32, Float64MultiArray, Bool
import numpy as np


sim_motion_done = False
sim_ready = False

def execute_pose(idx, angles_deg, pub_joints, client):
    global last_sim_ack

    pos_str  = " ".join(str(a) for a in angles_deg)
    pose_rad = [np.radians(a) for a in angles_deg]

    print(f"\n=== EXECUTANDO POSE {idx} ===")

    # 1) SIM PRIMEIRO
    pose_msg = Float64MultiArray()
    pose_msg.data = [idx] + pose_rad

    wait_sim_ready()
    print("Enviando pose para SIM...")
    last_sim_ack = -1
    pub_joints.publish(pose_msg)
    wait_sim_motion(idx)

    rospy.sleep(0.5)

    # 2) ROBÔ REAL
    print("Executando pose no robô real...")
    move_robot_joint(client, pos_str)

    print("Pose concluída (sim + real)")

SIM_SETTLE_SEC = 1.0   # tempo opcional p/ sim estabilizar antes da captura
SAVE_WAIT_SEC  = 4.0   # tempo para salvar imagem após comando de captura

last_sim_ack = -1

def sim_motion_callback(msg):
    global last_sim_ack
    last_sim_ack = int(msg.data)
    print(f"ACK da sim recebido para pose {last_sim_ack}")

def sim_ready_callback(msg):
    global sim_ready
    sim_ready = msg.data
    print("SIM pronta para próxima pose")


def move_robot_joint(client, position_str):
    """Envia uma posição de juntas (string com 7 ângulos em graus) e espera concluir."""
    client.send_command(f'setPosition {position_str}')
    for _ in range(120):
        if client.isFinished and client.isFinished[0]:
            return
        sleep(1)
    print("Tempo máximo de espera atingido.")
    sys.exit(1)

def wait_sim_motion(expected_id):
    global last_sim_ack
    print(f"Esperando ACK da sim para pose {expected_id}...")
    rate = rospy.Rate(50)
    while last_sim_ack != expected_id and not rospy.is_shutdown():
        rate.sleep()
def wait_sim_ready():
    global sim_ready
    print("Esperando sim ficar pronta...")
    rate = rospy.Rate(50)
    while not sim_ready and not rospy.is_shutdown():
        rate.sleep()
    sim_ready = False



def main():
    # Cria o cliente (init_node acontece dentro de kuka_ros_node.py)
    client = kuka_iiwa_ros_node()

    # Aguarda conexão
    while not client.isReady:
        print("Aguardando conexão com o robô...")
        sleep(1)
    print(f"Conectado. Modo de operação: {client.OperationMode[0]}")

    # Ajusta limites de velocidade conforme o modo
    mode = client.OperationMode[0]
    if mode == 'T1':
        max_t1_speed(client)
    elif mode == 'T2':
        max_t2_speed(client)
    elif mode == 'AUT':
        slow_auto_speed(client)

    # Publishers para o Coppelia — criar após o cliente para evitar init_node duplo
    pub_joints = rospy.Publisher('/kuka_cmd', Float64MultiArray, queue_size=1)
    rospy.sleep(1.0)   # garante conexão real publisher-subscriber
    rospy.Subscriber('/sim_motion_done', Int32, sim_motion_callback)
    rospy.sleep(1.0)
    rospy.Subscriber('/sim_ready', Bool, sim_ready_callback)
    rospy.sleep(1.0)
    

    print("Esperando subscribers da sim conectarem...")
    while pub_joints.get_num_connections() == 0:
        rospy.sleep(0.1)

    print("Sim conectada!")

    print("\n===== TELEOP DE JUNTAS =====")
    print("Digite 7 ângulos das juntas em graus")
    print("Exemplo: 0 -30 0 60 0 0 0")
    print("Digite 'sair' para encerrar")

    pose_id = 1

    while not rospy.is_shutdown():
        try:
            cmd = input("\nComando: ")

            if cmd.lower() in ["sair", "exit", "quit"]:
                print("Encerrando teleop.")
                break

            parts = cmd.split()

            if len(parts) == 7:
                try:
                    angles = [float(x) for x in parts]
                    execute_pose(pose_id, angles, pub_joints, client)
                    pose_id += 1
                except:
                    print("Digite apenas números.")
            else:
                print("Comando inválido.")
                print("Digite 7 ângulos separados por espaço.")
                print("Ex: 0 -30 0 60 0 0 0")

        except Exception as e:
            print("Erro:", e)

if __name__ == "__main__":
    main()

