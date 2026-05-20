""" 
# Direct and Inverse Kinematics Model Simulation and Validation

## Objective:

Test and visualize direct and inverse kinematics model


## Requirements:

Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.

Input: position of the end effector.

Output: 
- Visual representation of robot joints and links in space.
- Error between desired end effector position and accomplished end effector position.
- Error due to changes in link longitude.

"""

import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


def rot_x(theta):
    return np.array([
        [1, 0, 0, 0 ],
        [0, np.cos(theta), -np.sin(theta), 0 ],
        [0, np.sin(theta), np.cos(theta), 0 ],
        [0, 0, 0, 1]
    ])

def rot_y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta), 0 ],
        [0, 1, 0, 0 ],
        [-np.sin(theta), 0, np.cos(theta), 0 ],
        [0, 0, 0, 1]
    ])

def rot_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0 ],
        [np.sin(theta), np.cos(theta), 0, 0 ],
        [0, 0, 1, 0 ],
        [0, 0, 0, 1]
    ])

def movement(x, y, z):
    return np.array([
        [1, 0, 0, x ],
        [0, 1, 0, y ],
        [0, 0, 1, z ],
        [0, 0, 0, 1]
    ])

def direct(pf, j, zero):
    A0=rot_z(j[0]) @ movement(0, 0, l0)
    A1=rot_y(j[1]) @ movement(0, 0, l1)
    A2=rot_y(j[2]) @ movement(0, 0, l2)

    pf[3]=A0@A1@A2@zero

    pf[2]=A0@A1@zero

    pf[1]=A0@zero

def inverse(pr, j):
    j[0] = np.atan2(pr[1],pr[0])

    ## Limitation on arccos input between -1 and 1
    D = (pr[0]**2 + pr[1]**2 + (pr[2]-l0)**2 - l1**2 - l2**2)/(2*l1*l2)
    D = np.clip(D, -1.0, 1.0)

    j[2] = np.arccos(D) 

    j[1] = np.atan2((pr[0]**2+pr[1]**2)**(0.5), pr[2]-l0) - np.atan2(l2*np.sin(j[2]), l1+l2*np.cos(j[2]))

##Joint angles

j0 = 0
j1 = 0
j2 = np.pi/2

j = [j0, j1, j2] 


##Link lenght

l0 = 1
l1 = 2
l2 = 2

l = [l0, l1, l2] 


##Each joint position in the world frame

zero = np.array([
    [0],
    [0],
    [0],
    [1]
])


p0 = np.array([
    [0],
    [0],
    [l0],
    [1]
])

p1 = np.array([
    [0],
    [0],
    [l0+l1],
    [1]
])

p2 = np.array([
    [0],
    [0],
    [l0+l1+l2],
    [1]
])

pf = [zero, p0, p1, p2]



# Main window
root = tk.Tk()
root.title("3D arm visualization")

# Global frame space definition
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1,1,1])

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

def vector_draw():
    # Read end effector position
    x = float(entry_x.get())
    y = float(entry_y.get())
    z = float(entry_z.get())
    pr = [x, y, z]

    # Based on desired end effector position, calculate each joint angle, stored in j
    inverse(pr, j)
    direct(pf, j, zero)

    # Clear global space
    ax.clear()

    # Variable storing each link lenght post calculations
    link_long=[]

    # Draw each link, represented by a vector
    for i in range(len(pf)-1):
        ax.quiver(pf[i][0], pf[i][1], pf[i][2], pf[i+1][0]-pf[i][0], pf[i+1][1]-pf[i][1], pf[i+1][2]-pf[i][2])
        link_long.append(((pf[i+1][0]-pf[i][0])**2 + (pf[i+1][1]-pf[i][1])**2 + (pf[i+1][2]-pf[i][2])**2)**0.5)

    # Draw each joint, represented by a point
    for p in pf:
        ax.scatter(p[0,0], p[1,0], p[2,0])
    
    # Write each result variable
    angle_text.set(
    f"j0={np.degrees(j[0]):.1f}°  "
    f"j1={np.degrees(j[1]):.1f}°  "
    f"j2={np.degrees(j[2]):.1f}°"
    )

    link_long_error.set(
    f"l0={abs(l0-link_long[0])}  "
    f"l1={abs(l1-link_long[1])}  "
    f"l2={abs(l2-link_long[2])}"
    )

    position_error.set(
    f"x={abs(x-pf[3][0])}  "
    f"y={abs(y-pf[3][1])}  "
    f"z={abs(z-pf[3][2])}"
    )

    # Global space limit
    limit= l0 + l1 + l2 + 1
    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([-limit, limit])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    canvas.draw()

# Entradas y salidas
frame = tk.Frame(root)
frame.pack()

angle_text = tk.StringVar()
angle_text.set("Ángulos: ")

label_angle_text = tk.Label(frame, textvariable=angle_text)
label_angle_text.grid(row=4, columnspan=2)


link_long_error = tk.StringVar()
link_long_error.set("Error links: ")

label_link_long_error = tk.Label(frame, textvariable=link_long_error)
label_link_long_error.grid(row=5, columnspan=2)


position_error = tk.StringVar()
position_error.set("Error posicion: ")

label_position_error = tk.Label(frame, textvariable=position_error)
label_position_error.grid(row=6, columnspan=2)


tk.Label(frame, text="X").grid(row=0, column=0)
entry_x = tk.Entry(frame)
entry_x.grid(row=0, column=1)
entry_x.insert(0, "1")

tk.Label(frame, text="Y").grid(row=1, column=0)
entry_y = tk.Entry(frame)
entry_y.grid(row=1, column=1)
entry_y.insert(0, "1")

tk.Label(frame, text="Z").grid(row=2, column=0)
entry_z = tk.Entry(frame)
entry_z.grid(row=2, column=1)
entry_z.insert(0, "1")

# Botón
boton = tk.Button(frame, text="Draw", command=vector_draw)
boton.grid(row=3, columnspan=2)

# Primer dibujo
vector_draw()

root.mainloop()



