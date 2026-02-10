#!/usr/bin/env python3

from kuka_ros_node import *
import rospy
from time import sleep

client = kuka_iiwa_ros_node()  # Making a connection object.

def move_robot_joint(client):
    position = input("Digite a posição das juntas (ex: '0 -15 00 75 90 30 0'): ")
    client.send_command(f'setPosition {position}')
    print("Movendo para posição... Aguarde")
    sleep(3)
    while not client.isFinished[0]:
        print("Aguardando finalização...")
        sleep(1)
    print("Movimento concluído.")

def move_robot_xyzabc(client):
    position = input("Digite a posição XYZABC (ex: '-660 50 800 - - - ptp'): ")
    client.send_command(f'setPositionXYZABC {position}')
    print("Movendo para posição... Aguarde")
    sleep(3)
    while not client.isFinished[0]:
        print("Aguardando finalização...")
        sleep(1)
    print("Movimento concluído.")

def read_robot_data(client):
    print("1. Ler posição das juntas")
    print("2. Ler posição da ferramenta")
    print("3. Ler forças e torques")
    opcao = input("Escolha uma opção: ")
    
    if opcao == '1':
        print("Posição das juntas:", client.JointPosition)
    elif opcao == '2':
        print("Posição da ferramenta:", client.ToolPosition)
    elif opcao == '3':
        print("Forças na ferramenta:", client.ToolForce)
        print("Torques na ferramenta:", client.ToolTorque)
    else:
        print("Opção inválida!")

def read_safety_data(client):
    print("Leitura de Segurança:")
    print("- Compliance:", client.isCompliance)
    print("- Colisão detectada:", client.isCollision)
    print("- Masterizado:", client.isMastered)
    print("- Modo de operação:", client.OperationMode)

def menu(client):
    while not rospy.is_shutdown():
        print("\nMenu de controle do robô KUKA:")
        print("1. Mover no espaço de juntas")
        print("2. Mover no espaço cartesiano")
        print("3. Ler dados do robô")
        print("4. Leitura de Segurança")
        print("5. Sair")
        escolha = input("Escolha uma opção: ")

        if escolha == '1':
            move_robot_joint(client)
        elif escolha == '2':
            move_robot_xyzabc(client)
        elif escolha == '3':
            read_robot_data(client)
        elif escolha == '4':
            read_safety_data(client)
        elif escolha == '5':
            print("Saindo...")
            break
        else:
            print("Opção inválida! Escolha novamente.")

if __name__ == "__main__":
    print("Compliance:", client.isCompliance)
    client.send_command(f'setCompliance 200 200 100 100 100 100')
    print("Compliance:", client.isCompliance)
    #menu(client)
