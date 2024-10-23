import tkinter as tk
from math import atan2, sqrt, cos, sin, pi
import numpy as np

def constrain_sin_cos(value):
    return max(-1, min(1, value))

def ik_2links(x, y, l1, l2):
    cos_theta2 = constrain_sin_cos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
    sin_theta2 = sqrt(1 - cos_theta2**2)
    theta2 = atan2(sin_theta2, cos_theta2)
    
    k1 = l1 + l2 * cos_theta2
    k2 = l2 * sin_theta2
    theta1 = atan2(y, x) - atan2(k2, k1)

    return [theta1, theta2]

def ik_3links(x, y, L1, L2, L3):
    # Calculate the position of the wrist (ignoring the third link)
    wrist_x = x - L3 * np.cos(np.arctan2(y, x))
    wrist_y = y - L3 * np.sin(np.arctan2(y, x))

    # Use the cosine rule to find the angles
    cos_theta2 = (wrist_x**2 + wrist_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))  # Clipping to avoid numerical errors

    theta1 = np.arctan2(wrist_y, wrist_x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

    theta3 = np.arctan2(y, x) - theta1 - theta2

    # Convert angles from radians to degrees
    theta1 = np.degrees(theta1)
    theta2 = np.degrees(theta2)
    theta3 = np.degrees(theta3)

    return theta1, theta2, theta3

def visualize():
    x = float(entry_x.get())
    y = float(entry_y.get())
    l1 = float(entry_l1.get())
    l2 = float(entry_l2.get())
    l3 = float(entry_l3.get())
    
    theta1, theta2, theta3 = ik_3links(x, y, l1, l2, l3)

    print(theta1, theta2, theta3)
    
    canvas.delete("all")
    
    x0, y0 = 250, 250
    x1 = x0 + l1 * cos(theta1)
    y1 = y0 - l1 * sin(theta1)
    x2 = x1 + l2 * cos(theta1 + theta2)
    y2 = y1 - l2 * sin(theta1 + theta2)
    x3 = x2 + l3 * cos(theta1 + theta2 + theta3)
    y3 = y2 - l3 * sin(theta1 + theta2 + theta3)
    
    canvas.create_line(x0, y0, x1, y1, fill="blue", width=2)
    canvas.create_line(x1, y1, x2, y2, fill="red", width=2)
    canvas.create_line(x2, y2, x3, y3, fill="green", width=2)
    canvas.create_oval(x0-5, y0-5, x0+5, y0+5, fill="black")
    canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill="black")
    canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill="black")
    canvas.create_oval(x3-5, y3-5, x3+5, y3+5, fill="black")

root = tk.Tk()
root.title("3-Link Inverse Kinematics Visualization")

tk.Label(root, text="x:").grid(row=0, column=0)
entry_x = tk.Entry(root)
entry_x.grid(row=0, column=1)

tk.Label(root, text="y:").grid(row=1, column=0)
entry_y = tk.Entry(root)
entry_y.grid(row=1, column=1)

tk.Label(root, text="l1:").grid(row=2, column=0)
entry_l1 = tk.Entry(root)
entry_l1.grid(row=2, column=1)

tk.Label(root, text="l2:").grid(row=3, column=0)
entry_l2 = tk.Entry(root)
entry_l2.grid(row=3, column=1)

tk.Label(root, text="l3:").grid(row=4, column=0)
entry_l3 = tk.Entry(root)
entry_l3.grid(row=4, column=1)

button = tk.Button(root, text="Visualize", command=visualize)
button.grid(row=4, columnspan=2)

canvas = tk.Canvas(root, width=500, height=500, bg="white")
canvas.grid(row=5, columnspan=2)

root.mainloop()