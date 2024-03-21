#!/usr/bin/env python3

import psutil
import tkinter as tk
import threading
import time
import signal
import sys

def get_storage_space():
    partition = '/media/nacho/HDD/'  # Ruta de la partición a monitorear
    #partition = '/media/nacho/TOSHIBA EXT/'
    space = psutil.disk_usage(partition)
    return space.total, space.used

def update_interface():
    total, used = get_storage_space()
    used_percentage = (used / total) * 100

    percentage_label.config(text=f'Used Percentage: {used_percentage:.2f}%')
    update_progress_bar(used, total)

    root.after(1000, update_interface)

def update_progress_bar(used, total):
    percentage = (used / total) * 100
    bar_width = int((percentage / 100) * progress_bar_width)

    canvas.delete("bar")

    canvas.create_rectangle(0, 0, bar_width, progress_bar_height, fill="blue", tags="bar")

def quit_program():
    root.destroy()

def handle_interrupt(signal, frame):
    print("\nStorage monitor interrupted by user")
    sys.exit(0)

root = tk.Tk()
root.title("Storage Space Monitor")

percentage_label = tk.Label(root, text="Used Percentage: 0.00%")
percentage_label.pack()

progress_bar_width = 400
progress_bar_height = 20

canvas = tk.Canvas(root, width=progress_bar_width, height=progress_bar_height)
canvas.pack()

quit_button = tk.Button(root, text="Quit", command=quit_program)
quit_button.pack()

update_thread = threading.Thread(target=update_interface)
update_thread.daemon = True
update_thread.start()

signal.signal(signal.SIGINT, handle_interrupt)

root.mainloop()
