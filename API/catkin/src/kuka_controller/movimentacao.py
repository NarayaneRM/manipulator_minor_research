#!/usr/bin/env python3

from kuka_ros_node import *
from settings import max_t1_speed, max_t2_speed, max_auto_speed, medium_auto_speed, slow_auto_speed
from time import sleep
import sys

def move_robot_joint(client, position):
    print(f"Sending position {position}")
    client.send_command(f'setPosition {position}')
    sleep(3)

    for i in range(100):
        print(f'Is Finished? {client.isFinished}')
        #print(client.JointVelocity) //Corrigir!
        sleep(1)
        if client.isFinished[0]:
            break
        if i == 99:
            print("Max tries achieved")
            sys.exit()
        print(f"Slept for {i} seconds")

    print('Finished')

def move_robot_xyzabc(client, position): #funcao para Exame
    print(f"Sending position {position}")
    client.send_command(f'setPositionXYZABC {position}')
    sleep(1)

    for i in range(100):
        print(f'Is Finished? {client.isFinished}')
        # print(client.JointVelocity) //Corrigir!
        sleep(1)
        if client.isFinished[0]:
            break
        if i == 99:
            print("Max tries achieved")
            sys.exit()
        print(f"Slept for {i} seconds")

    print('Finished')

def rotational_cycle(client, repetitions=10):
    for i in range(repetitions):
        move_robot_joint(client, '0 -15 00 75 90 30 0')
        move_robot_joint(client, '0 -15 00 75 90 -30 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def linear_cycle(client, repetitions=10):
    for i in range(repetitions):
        move_robot_xyzabc(client, '-660 50 800 - - - ptp')
        move_robot_xyzabc(client, '-660 50 750 - - - ptp')
        move_robot_xyzabc(client, '-660 0 750 - - - ptp')
        move_robot_xyzabc(client, '-660 0 800 - - - ptp')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def linear_cycle_y(client, repetitions=10):
    move_robot_joint(client, '0 -15 0 75 0 0 0')
    for i in range(repetitions):
        move_robot_xyzabc(client, '-660 0 750 - - - ptp')
        move_robot_xyzabc(client, '-560 0 750 - - - ptp')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def rotational_cycle_velocity(client, repetitions=10):
    max_auto_speed(client)

    for i in range(repetitions):
        move_robot_joint(client, '0 -15 -90 75 0 0 0')
        move_robot_joint(client, '0 -15 90 75 0 0 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

    sleep(10)
    medium_auto_speed(client)

    for i in range(repetitions):
        move_robot_joint(client, '0 -15 -90 75 0 0 0')
        move_robot_joint(client, '0 -15 90 75 0 0 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

    sleep(10)
    slow_auto_speed(client)

    for i in range(repetitions):
        move_robot_joint(client, '0 -15 -90 75 0 0 0')
        move_robot_joint(client, '0 -15 90 75 0 0 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

client = kuka_iiwa_ros_node()  # Making a connection object.

# Adicionando monitoramento de login e endereço
while not client.isReady:
    print("Waiting for the client to connect...")
    sleep(1)

print(f"Client connected: {client.isReady}")
#print(f"Server address: {client.server_address}")

if client.OperationMode[0] == 'T1':
    max_t1_speed(client)
elif client.OperationMode[0] == 'T2':
    max_t2_speed(client)
elif client.OperationMode[0] == 'AUT':
    slow_auto_speed(client)

print('The robot is in', client.OperationMode[0], 'mode.')

def print_tool_position(client):
    rate = rospy.Rate(1)  # 1 Hz (imprime a cada segundo)
    while not rospy.is_shutdown():
        if client.ToolPosition is not None:
            print("Último valor de ToolPosition:", client.ToolPosition)
        else:
            print("Aguardando dados de ToolPosition...")
        rate.sleep()

# Example command
move_robot_joint(client, '0 0 0 0 0 0 0')
move_robot_joint(client, '-90.01 0.06 -0.02 -89.70 -0.09 90.26 0')
#move_robot_joint(client, '0.06 -60.35 -0.02 -89.70 -45.19 90.26 0')
#move_robot_joint(client, '30.21 -30.61 30.57 -30.41 30.93 -29.64 0')
#linear_cycle(client, repetitions=10)
print_tool_position(client)
sleep(1)
