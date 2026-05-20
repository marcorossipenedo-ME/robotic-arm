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

class Vector:
    def __init__(self, values):
        self.v = values

    def __len__(self):
        return len(self.v)

    def __getitem__(self, index):
        return self.v[index]

    def __add__(self, other):
        r = []

        for i in range(len(self)):
            r.append(self[i] + other[i])

        return Vector(r)

    def __sub__(self, other):
        r = []

        for i in range(len(self)):
            r.append(self[i] - other[i])

        return Vector(r)

    def __mul__(self, scalar):
        r = []

        for i in range(len(self)):
            r.append(self[i] * scalar)

        return Vector(r)

    def dot(self, other):
        d = 0

        for i in range(len(self)):
            d += self[i] * other[i]

        return d

    def __repr__(self):
        return f"Vector({self.v})"
    

class Matrix:
    def __init__(self, rows):
        self.m = rows


    def shape(self):
        return (len(self.m), len(self.m[0]))


    def __getitem__(self, index):
        return self.m[index]


    def __matmul__(self, other):
        rows_A = len(self.m)
        cols_A = len(self.m[0])

        rows_B = len(other.m)
        cols_B = len(other.m[0])

        if cols_A != rows_B:
            raise Exception("Invalid dimensions")

        C = []

        for i in range(rows_A):
            row = []

            for j in range(cols_B):

                d = 0

                for k in range(cols_A):
                    d += self.m[i][k] * other.m[k][j]

                row.append(d)

            C.append(row)

        return Matrix(C)


    def apply(self, v):

        r = []

        for i in range(len(self.m)):

            d = 0

            for k in range(len(self.m[0])):
                d += self.m[i][k] * v[k]

            r.append(d)

        return Vector(r)
    

    @staticmethod
    def identity(e):
        B = []
        for i in range(e):
            row = []
            for j in range(e):
                if(i==j):
                    row.append(1)
                else:
                    row.append(0)
            B.append(row)

        return(Matrix(B))
    

    def gauss_exchange(self,i,j):
        for k in range(len(self.m[0])):
            t=self.m[i][k]
            self.m[i][k]=self.m[j][k]
            self.m[j][k]=t

    def gauss_sum(self,i,j,e):
        for k in range(len(self.m[0])):
            self.m[i][k]=self.m[i][k]+e*self.m[j][k]

    def gauss_mult(self,i,j):
        for k in range(len(self.m[0])):
            self.m[i][k]=self.m[i][k]*j

    def copy(self):
        B = []
        for i in range(len(self.m)):
            row = []
            for j in range(len(self.m[0])):
                row.append(self.m[i][j])
            B.append(row)

        return Matrix(B)


    def inverse(self):

        A = self.copy()

        rows_A = len(A.m)
        cols_A = len(A.m[0])

        if rows_A != cols_A:
            raise Exception("Invalid dimensions")

        B = Matrix.identity(rows_A)

        eps = 1e-12

        for i in range(cols_A):

            # Searching for a non 0 number in column i
            pivot_row = i

            while pivot_row < rows_A and abs(A.m[pivot_row][i]) < eps:
                pivot_row += 1

            # If all the column is 0
            if pivot_row == rows_A:
                raise Exception("Singular matrix")

            # Exchange rows to locate pivot in matrix diagonal
            if pivot_row != i:
                A.gauss_exchange(i, pivot_row)
                B.gauss_exchange(i, pivot_row)

            # Turn pivot into 1
            pivot = A.m[i][i]

            A.gauss_mult(i, 1/pivot)
            B.gauss_mult(i, 1/pivot)

            # Turn all numbers under diagonal to 0
            for j in range(i+1, rows_A):

                if abs(A.m[j][i]) > eps:

                    factor = -A.m[j][i]

                    A.gauss_sum(j, i, factor)
                    B.gauss_sum(j, i, factor)

        # Using found pivots, turn numbers above diagonal into 0

        for i in range(cols_A-1, -1, -1):

            for j in range(i-1, -1, -1):

                if abs(A.m[j][i]) > eps:

                    factor = -A.m[j][i]

                    A.gauss_sum(j, i, factor)
                    B.gauss_sum(j, i, factor)

        return B


    def __repr__(self):
        return f"Matrix({self.m})"





def rot_x(theta):
    return Matrix([
        [1, 0, 0, 0 ],
        [0, np.cos(theta), -np.sin(theta), 0 ],
        [0, np.sin(theta), np.cos(theta), 0 ],
        [0, 0, 0, 1]
    ])

def rot_y(theta):
    return Matrix([
        [np.cos(theta), 0, np.sin(theta), 0 ],
        [0, 1, 0, 0 ],
        [-np.sin(theta), 0, np.cos(theta), 0 ],
        [0, 0, 0, 1]
    ])

def rot_z(theta):
    return Matrix([
        [np.cos(theta), -np.sin(theta), 0, 0 ],
        [np.sin(theta), np.cos(theta), 0, 0 ],
        [0, 0, 1, 0 ],
        [0, 0, 0, 1]
    ])

def movement(x, y, z):
    return Matrix([
        [1, 0, 0, x ],
        [0, 1, 0, y ],
        [0, 0, 1, z ],
        [0, 0, 0, 1]
    ])

def direct_k(j, zero, l):
    A0=rot_z(j[0]) @ movement(0, 0, l[0])
    A1=rot_y(j[1]) @ movement(0, 0, l[1])
    A2=rot_y(j[2]) @ movement(0, 0, l[2])

    p3=Matrix.apply(A0@A1@A2,zero)

    p2=Matrix.apply(A0@A1,zero)

    p1=Matrix.apply(A0,zero)

    p0=zero

    return([p0,p1,p2,p3])

def inverse_k(pr, l):
    
    j=[]
    j.append(np.atan2(pr[1],pr[0])) 

    ## Limitation on arccos input between -1 and 1
    D = (pr[0]**2 + pr[1]**2 + (pr[2]-l[0])**2 - l[1]**2 - l[2]**2)/(2*l[1]*l[2])
    D = np.clip(D, -1.0, 1.0)

    t = np.arccos(D) 

    j.append(np.atan2((pr[0]**2+pr[1]**2)**(0.5), pr[2]-l[0]) - np.atan2(l[2]*np.sin(t), l[1]+l[2]*np.cos(t)))
    j.append(t)
    return Vector(j)
    



##Joint angles

j = Vector([0, 0, np.pi/2])


##Link lenght

l = ([1, 2, 2]) 


##Each joint position in the world frame

zero = Vector([0,0,0,1])


p0 = Vector([0,0,l[0],1])

p1 = Vector([0,0,l[0]+l[1],1])

p2 = Vector([0,0,l[0]+l[1]+l[2],1])

pf = Matrix([zero, p0, p1, p2])



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
    j=inverse_k(pr, l)
    pf=direct_k(j, zero, l)

    # Clear global space
    ax.clear()

    # Variable storing each link lenght post calculations
    link_long=[]

    # Draw each link, represented by a vector
    for i in range(len(pf)-1):

        p0 = pf[i][:3]
        p1 = pf[i+1][:3]

        ax.quiver(
            p0[0], p0[1], p0[2],
            p1[0]-p0[0],
            p1[1]-p0[1],
            p1[2]-p0[2]
        )
        
        link_long.append(((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2)**0.5)

    
    # Write each result variable
    angle_text.set(
    f"j0={np.degrees(j[0]):.1f}°  "
    f"j1={np.degrees(j[1]):.1f}°  "
    f"j2={np.degrees(j[2]):.1f}°"
    )

    link_long_error.set(
    f"l0={abs(l[0]-link_long[0])}  "
    f"l1={abs(l[1]-link_long[1])}  "
    f"l2={abs(l[2]-link_long[2])}"
    )

    position_error.set(
    f"x={abs(x-pf[3][0])}  "
    f"y={abs(y-pf[3][1])}  "
    f"z={abs(z-pf[3][2])}"
    )

    # Global space limit
    limit= l[0] + l[1] + l[2] + 1
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



