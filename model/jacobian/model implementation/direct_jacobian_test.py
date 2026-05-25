""" 
# Direct and Inverse Kinematics Model Simulation and Validation

## Objective:

Test and visualize direct and inverse kinematics model


## Requirements:

Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.

Input: position of the end effector.

Output: 
- Visual representation of robot joints and links in space, with movement.
- Error between end effector position based on Jacobian calculations and kinematics.

Numpy will not be used for algebraic matematical operations, as it produced undesired behaviours and it's functions could not be fully controlled. 

Vectors are defined as row vectors, but represent column vectores in reality. This is due to the fact that they are easier to use and define in python.

Matrix are defined as follows:
A=[[row 0],
    [row 1],
    [...],
    [row n]]
"""

import time
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
    
    


def jacobian(j, l):
    return Matrix([
        [-np.sin(j[0])*(l[1]*np.cos(j[1])+l[2]*np.cos(j[1]+j[2])), -np.cos(j[0])*(l[1]*np.sin(j[1])+l[2]*np.sin(j[1]+j[2])), -l[2]*np.cos(j[0])*np.sin(j[1]+j[2])],
        [np.cos(j[0])*(l[1]*np.cos(j[1])+l[2]*np.cos(j[1]+j[2])), -np.sin(j[0])*(l[1]*np.sin(j[1])+l[2]*np.sin(j[1]+j[2])), -l[2]*np.sin(j[0])*np.sin(j[1]+j[2])],
        [0, l[1]*np.cos(j[1])+l[2]*np.cos(j[1]+j[2]), l[2]*np.cos(j[1]+j[2])]
    ])


"""
def inverse_jacobian_implementation(p_v, j, l):

    A=l[1]*np.cos(j[1])+l[2]*np.cos(j[1]+j[2])

    B=l[1]*np.sin(j[1])+l[2]*np.sin(j[1]+j[2])

    C=-l[2]*np.cos(j[0])*np.sin(j[1]+j[2])

    D=-l[2]*np.sin(j[0])*np.sin(j[1]+j[2])

    E=l[2]*np.cos(j[1]+j[2])

    j_v1=(E*A*p_v[2]*(C*np.cos(j[0])+D*np.sin(j[0]))-E*E*A*(p_v[0]*np.cos(j[0])+p_v[1]*np.sin(j[0])))/(B*E*E*A+A*A*C*E*(np.cos(j[0])+np.sin(j[0])))

    j_v2=(p_v[2]-A*j_v1)/E

    j_v0=(E*p_v[1]+j_v1*B*E*np.sin(j[0])-p_v[2]*D+j_v1*A*D)/(E*A*np.cos(j[0]))
    
    return ([j_v0, j_v1, j_v2])

"""



def end_effector_velocity(t):

    return Vector([0.5, 0, 0])




def arm_draw(pf, c):

    colors = np.array(['red', 'blue', 'green', 'black'])
    
    for i in range(len(pf)-1):

        p0 = pf[i][:3]
        p1 = pf[i+1][:3]

        ax.quiver(
            p0[0], p0[1], p0[2],
            p1[0]-p0[0],
            p1[1]-p0[1],
            p1[2]-p0[2], 
            color=colors[c]
        )

    limit = 6

    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([-limit, limit])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    canvas.draw()

zero = Vector([0,0,0,1])

##End effector initial position

p_end = Vector([1,0,0.5])

##Link lenght

l = Vector([1,2,2])

j=inverse_k(p_end, l)
p=direct_k(j, zero, l)

step=0.1

real_p=p_end

jaco_p=p_end

jaco_j=j

i = 0


def update():

    global i
    global real_p
    global jaco_j
    global jaco_p

    if i > 10:
        return

    p_vel = end_effector_velocity(i)
    
    j0=inverse_k(real_p, l)

    real_p = real_p + p_vel*step
    
    j1=inverse_k(real_p, l)
    
    j_vel=(j1-j0)*(1/step)

    p_jaco_vel = Matrix.apply((jacobian(j0, l)), j_vel)

    jaco_p = jaco_p+p_jaco_vel*step
    
    ax.clear()

    arm_draw(direct_k(inverse_k(jaco_p, l), zero, l), 0) ##red arm
    
    arm_draw(direct_k(inverse_k(real_p, l), zero, l), 1) ##blue arm

    error_x = abs(p_vel[0] - p_jaco_vel[0])
    error_y = abs(p_vel[1] - p_jaco_vel[1])
    error_z = abs(p_vel[2] - p_jaco_vel[2])

    position_error.set(
        f"x={error_x:.4f}  "
        f"y={error_y:.4f}  "
        f"z={error_z:.4f}"
    )

    i += step

    root.after(int(step * 1000), update)




# Main window
root = tk.Tk()
root.title("3D arm visualization")

# Global frame space definition
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1,1,1])

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Entradas y salidas
frame = tk.Frame(root)
frame.pack()

position_error = tk.StringVar()
position_error.set("Error posicion: ")

label_position_error = tk.Label(frame, textvariable=position_error)
label_position_error.grid(row=6, columnspan=2)

update()
root.mainloop()

