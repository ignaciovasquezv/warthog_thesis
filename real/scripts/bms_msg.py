#!/usr/bin/env python3

import rospy
from valence_bms_msgs.msg import SystemStatus
import tkinter as tk
from tkinter import ttk 

class BatteryMonitorGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Battery Monitor")
        
        self.state_of_charge_label = tk.Label(self.root, text="State of Charge:", font=("Helvetica", 24))
        self.state_of_charge_label.pack()
        
        self.state_of_charge_value = tk.StringVar()
        self.state_of_charge_display = tk.Label(self.root, textvariable=self.state_of_charge_value, font=("Helvetica", 24))
        self.state_of_charge_display.pack()

        # Agrega una barra de progreso con forma de batería
        self.canvas = tk.Canvas(self.root, width=200, height=100)
        self.canvas.pack()
        
        self.battery_outline = self.canvas.create_rectangle(5, 5, 200, 70, outline="black", width=2)
        self.battery_fill = self.canvas.create_rectangle(10, 10, 10, 65, outline="", fill="green")

        rospy.init_node('battery_monitor_node', anonymous=True)
        rospy.Subscriber('/bms/system_status', SystemStatus, self.callback)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def callback(self, data):
        state_of_charge = data.state_of_charge
        self.state_of_charge_value.set("{:.2f}%".format(state_of_charge))

        # Actualiza la forma de la batería según el estado de carga
        fill_width = int(185 * (state_of_charge / 100))  # Calcula el ancho de llenado proporcional
        self.canvas.coords(self.battery_fill, 10, 10, 10 + fill_width, 65)

        # Actualiza el color de la barra según el estado de carga
        if state_of_charge < 30:
            fill_color = "red"
        elif state_of_charge < 50:
            fill_color = "orange"
        else:
            fill_color = "green"
        self.canvas.itemconfig(self.battery_fill, fill=fill_color)

    def on_closing(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    gui = BatteryMonitorGUI()
    gui.run()

