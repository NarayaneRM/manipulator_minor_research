#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox
from kuka_ros_node import *
import rospy
from time import sleep
import customtkinter as ctk
from tkinter import ttk

class KukaGUI:
    def __init__(self, root):
        self.client = kuka_iiwa_ros_node()
        self.root = root
        self.root.title("Controle do Robô KUKA")
        self.root.geometry("1000x700")  # Ajustado para mais largura e altura
        ctk.set_appearance_mode("Dark")
        
        # Frame principal
        main_frame = ctk.CTkFrame(root, corner_radius=10, fg_color="black")
        main_frame.pack(fill=tk.BOTH, padx=20, pady=20)
        
        # Título
        ctk.CTkLabel(main_frame, text="Controle do Robô KUKA", font=("Helvetica", 18, "bold"), text_color="blue").pack(pady=10)

        # Bloco de controle das juntas (sliders e entradas)
        joint_control_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=10, border_width=2, border_color="blue")
        joint_control_frame.pack(pady=10, padx=10, side=tk.LEFT, fill=tk.Y)

        self.joint_sliders = []
        self.joint_entries = []
        slider_frame = ctk.CTkFrame(joint_control_frame, fg_color="black")
        slider_frame.pack(pady=10, fill=tk.X)
        
        for i in range(7):
            ctk.CTkLabel(slider_frame, text=f"A{i+1}", font=("Helvetica", 12, "bold"), text_color="blue").grid(row=i, column=0, sticky='w', padx=5)
            var = tk.DoubleVar()
            slider = ctk.CTkSlider(slider_frame, from_=-170, to=170, variable=var, width=300, button_color="blue")
            slider.grid(row=i, column=1, padx=5, pady=2)
            self.joint_sliders.append(slider)
            
            # Caixa de entrada para cada junta
            entry = ctk.CTkEntry(slider_frame, width=50)
            entry.grid(row=i, column=2, padx=5, pady=2)
            entry.insert(0, str(round(slider.get(), 2)))  # Inserir valor inicial
            self.joint_entries.append(entry)

            # Vincular o evento de digitação para atualizar o slider
            entry.bind("<FocusOut>", lambda event, idx=i: self.update_slider_from_entry(int(idx)))

        
        # Botões para mover juntas
        button_frame = ctk.CTkFrame(joint_control_frame, fg_color="black")
        button_frame.pack(pady=10)
        
        self.move_button = ctk.CTkButton(button_frame, text="Mover Juntas", width=150, command=self.move_robot_joint, fg_color="blue")
        self.move_button.grid(row=0, column=0, padx=5, pady=2)
        
        # Bloco de leitura de dados
        data_reading_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=10, border_width=2, border_color="blue")
        data_reading_frame.pack(pady=10, padx=10, side=tk.LEFT, fill=tk.Y)
        
        # Tabelas de dados
        self.data_table_joints = ttk.Treeview(data_reading_frame, columns=("Junta", "Valor"), show="headings", style="Custom.Treeview")
        self.data_table_joints.heading("Junta", text="Junta")
        self.data_table_joints.heading("Valor", text="Valor")

        for i in range(7):
            self.data_table_joints.insert("", tk.END, values=(f"A{i+1}", "0.0"))

        self.data_table_joints.pack(fill=tk.X, padx=5, pady=2)

        # Tabela de Posições Cartesiana (XYZ)
        self.data_table_cartesian = ttk.Treeview(data_reading_frame, columns=("Posição", "Valor"), show="headings", style="Custom.Treeview")
        self.data_table_cartesian.heading("Posição", text="Posição")
        self.data_table_cartesian.heading("Valor", text="Valor")

        for pos in ["X", "Y", "Z", "A", "B", "C"]:
            self.data_table_cartesian.insert("", tk.END, values=(pos, "0.0"))

        self.data_table_cartesian.pack(fill=tk.X, padx=5, pady=2)

        # Tabela de Dados da Ferramenta
        self.data_table_tool = ttk.Treeview(data_reading_frame, columns=("Dado", "Valor"), show="headings", style="Custom.Treeview")
        self.data_table_tool.heading("Dado", text="Dado")
        self.data_table_tool.heading("Valor", text="Valor")

        for data in ["Torque X", "Torque Y", "Torque Z", "Força X", "Força Y", "Força Z"]:
            self.data_table_tool.insert("", tk.END, values=(data, "0.0"))

        self.data_table_tool.pack(fill=tk.X, padx=5, pady=2)

        # Tabela de Dados de Segurança
        self.data_table_safety = ttk.Treeview(data_reading_frame, columns=("Dado", "Valor"), show="headings", style="Custom.Treeview")
        self.data_table_safety.heading("Dado", text="Dado")
        self.data_table_safety.heading("Valor", text="Valor")

        for data in ["Compliance", "Colisão", "Masterizado", "Modo de Operação"]:
            self.data_table_safety.insert("", tk.END, values=(data, "Desconhecido"))

        self.data_table_safety.pack(fill=tk.X, padx=5, pady=2)

        # Bloco de outputs de status (como um terminal de logs)
        status_output_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=10, border_width=2, border_color="blue")
        status_output_frame.pack(pady=10, padx=10, side=tk.LEFT, fill=tk.BOTH, expand=True)  # Mudando para expandir o bloco

        ctk.CTkLabel(status_output_frame, text="Saída de Status", font=("Helvetica", 14, "bold"), text_color="blue").pack(pady=10)

        # Aumentando a altura e largura da área de texto
        self.status_text = ctk.CTkTextbox(status_output_frame, height=60, width=120, fg_color="black", text_color="blue", font=("Helvetica", 12), wrap="word")
        self.status_text.pack(pady=10, fill=tk.BOTH, expand=True)  # Expandir para preencher o bloco

        self.status_text.insert(tk.END, "Status: Aguardando conexão...\n")
     
        # Botão para sair
        ctk.CTkButton(main_frame, text="Sair", width=150, command=root.quit, fg_color="red").pack(pady=20)

        # Iniciar a atualização automática
        self.update_data()
        
        # Estilização da tabela
        self.style = ttk.Style()
        self.style.configure("Custom.Treeview", font=("Helvetica", 12), rowheight=25)
        self.style.configure("Custom.Treeview.Heading", font=("Helvetica", 14, "bold"))
        self.style.map("Custom.Treeview", background=[('selected', 'gray')])


        self.data_table_joints.config(height=7)
        self.data_table_cartesian.config(height=6)
        self.data_table_tool.config(height=6)
        self.data_table_safety.config(height=4)

        # Bloco de controle cartesiano (coordenadas XYZABC)
        cartesian_control_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=10, border_width=2, border_color="blue")
        cartesian_control_frame.pack(pady=10, padx=10, side=tk.LEFT, fill=tk.Y)

        ctk.CTkLabel(cartesian_control_frame, text="Controle Cartesiano", font=("Helvetica", 14, "bold"), text_color="blue").pack(pady=10)
        
        self.cartesian_entry = ctk.CTkEntry(cartesian_control_frame, width=200)
        self.cartesian_entry.pack(pady=5)
        self.cartesian_entry.insert(0, "Digite as coordenadas XYZABC aqui")
        
        ctk.CTkButton(cartesian_control_frame, text="Mover XYZABC", width=150, command=self.move_robot_xyzabc, fg_color="blue").pack(pady=10)

    def log_output(self, message):
        # Atualiza a saída de status como um terminal
        self.status_text.insert(tk.END, f"{message}\n")
        self.status_text.yview(tk.END)

    def update_status(self, message):
        # Atualiza a área de status com a mensagem
        self.log_output(message)
    

    def update_data(self):
        """ Atualiza todos os dados automaticamente a cada 500ms """
        self.read_robot_data()
        self.read_safety_data()
        self.root.after(500, self.update_data)  # Atualiza a cada 500ms

    def move_robot_joint(self):
        # Atualizar os sliders com os valores digitados
        for i, entry in enumerate(self.joint_entries):
            value = entry.get()
            try:
                value = float(value)
                self.joint_sliders[i].set(value)
            except ValueError:
                continue  # Caso o valor digitado não seja válido, não altera o slider

        # Enviar o comando
        position = " ".join(str(round(slider.get(), 2)) for slider in self.joint_sliders)
        self.client.send_command(f'setPosition {position}')
        self.update_status(f"Movendo para posição: {position}")
        self.wait_for_completion()
        
        # Alterar a cor do botão para normal após o envio
        self.move_button.configure(fg_color="blue")
    
    def move_robot_xyzabc(self):
        position = self.cartesian_entry.get()
        if position:
            self.client.send_command(f'setPositionXYZABC {position}')
            self.update_status(f"Movendo para posição XYZABC: {position}")
            self.wait_for_completion()

    # Defina o número de linhas de dados que você espera
    def create_tables(self):
        # Tabela para Juntas
        self.data_table_joints = ttk.Treeview(self.frame, columns=("Junta", "Valor"), show="headings")
        self.data_table_joints.heading("Junta", text="Junta")
        self.data_table_joints.heading("Valor", text="Valor")
        self.data_table_joints.pack(fill=tk.X, padx=5, pady=2)
        
        # Adicionar 7 linhas para as juntas
        for i in range(7):
            self.data_table_joints.insert("", "end", values=(f"A{i+1}", "0.00"))

        # Tabela para Posições Cartesiana
        self.data_table_cartesian = ttk.Treeview(self.frame, columns=("Coordenada", "Valor"), show="headings")
        self.data_table_cartesian.heading("Coordenada", text="Coordenada")
        self.data_table_cartesian.heading("Valor", text="Valor")
        self.data_table_cartesian.pack(fill=tk.X, padx=5, pady=2)
        
        # Adicionar 6 linhas para as posições cartesiana XYZABC
        for i in range(6):
            self.data_table_cartesian.insert("", "end", values=(f"XYZABC {i+1}", "0.00"))

        # Tabela para Dados da Ferramenta (Força)
        self.data_table_tool = ttk.Treeview(self.frame, columns=("Força", "Valor"), show="headings")
        self.data_table_tool.heading("Força", text="Força")
        self.data_table_tool.heading("Valor", text="Valor")
        self.data_table_tool.pack(fill=tk.X, padx=5, pady=2)
        
        # Adicionar 3 linhas para os dados de força (Fx, Fy, Fz)
        for i in range(3):
            self.data_table_tool.insert("", "end", values=(f"Tool Force {i+1}", "0.00"))

    
    def read_robot_data(self):
            # Ignorar o timestamp e pegar apenas a lista de valores (primeira parte da tupla)
        if self.client.JointPosition and self.client.JointPosition[0] is not None:
            joint_position_rounded = [round(value, 2) for value in self.client.JointPosition[0]]
        else:
            joint_position_rounded = [0.0] * 7  # Valores padrão caso os dados não estejam disponíveis

        tool_position_rounded = [round(value, 2) for value in self.client.ToolPosition[0]]  # Pegando a lista de valores
        tool_force_rounded = [round(value, 2) for value in self.client.ToolForce[0]]  # Pegando a lista de valores
        tool_torque_rounded = [round(value, 2) for value in self.client.ToolTorque[0]]  # Pegando a lista de valores
        
        # Atualizar a tabela de Juntas
        for i in range(7):
            self.data_table_joints.item(self.data_table_joints.get_children()[i], values=(f"A{i+1}", str(joint_position_rounded[i])))
        
        # Atualizar a tabela de Posições Cartesiana
        for i, pos in enumerate(tool_position_rounded):
            self.data_table_cartesian.item(self.data_table_cartesian.get_children()[i], values=(f"XYZABC {i+1}", str(pos)))
        
        # Atualizar a tabela de Dados da Ferramenta
        for i, data in enumerate(tool_force_rounded):
            self.data_table_tool.item(self.data_table_tool.get_children()[i], values=(f"Tool Force {i+1}", str(data)))
        
        for i, data in enumerate(tool_torque_rounded):
            self.data_table_tool.item(self.data_table_tool.get_children()[i+3], values=(["Torque X", "Torque Y", "Torque Z"][i], str(data)))


    def read_safety_data(self):
        """ Atualiza os dados de segurança automaticamente """

        try:
            safety_values = [
                "Ativado" if self.client.isCompliance else "Desativado",
                "Colisão Detectada" if self.client.isCollision else "Sem Colisão",
                "Sim" if self.client.isMastered else "Não",
                str(self.client.OperationMode[0])
            ]
        except AttributeError:
            safety_values = ["Desconhecido"] * 4  # Caso os valores não estejam disponíveis
        safety_labels = ["Compliance", "Colisão", "Masterizado", "Modo de Operação"]
        # Pegando os IDs das linhas da tabela
        children = self.data_table_safety.get_children()

        # Se a tabela estiver vazia, recria as linhas
        if not children:
            for label, value in zip(safety_labels, safety_values):
                self.data_table_safety.insert("", "end", values=(label, value))
        else:
            # Atualiza os valores das linhas já existentes
            for i, value in enumerate(safety_values):
                self.data_table_safety.item(children[i], values=(safety_labels[i], value))

    def get_input(self, prompt):
        input_window = ctk.CTkToplevel(self.root)
        input_window.title("Entrada de Dados")
        ctk.CTkLabel(input_window, text=prompt).pack(pady=10)
        entry = ctk.CTkEntry(input_window)
        entry.pack(pady=10)
        ctk.CTkButton(input_window, text="OK", command=lambda: input_window.destroy()).pack(pady=10)
        input_window.wait_window()

    def update_slider_from_entry(self, idx):
        try:
            value = float(self.joint_entries[idx].get())
            self.joint_sliders[idx].set(value)
        except ValueError:
            pass
    
    def wait_for_completion(self):
        self.move_button.configure(fg_color="gray")
        sleep(1)
        self.move_button.configure(fg_color="blue")


if __name__ == "__main__":
    root = tk.Tk()
    gui = KukaGUI(root)
    root.mainloop()