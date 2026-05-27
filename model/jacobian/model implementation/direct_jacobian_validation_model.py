""" 
# Direct Jacobian Model Simulation and Validation

## Objective:

Test and visualize direct Jacobian Model to verify correct definition.


## Requirements:

Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.

Input: position of the end effector.

Output: 
- Visual representation of robot joints and links in space, with movement.
- Error between end effector position based on direct Jacobian calculations and kinematics.

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
        I = []
        for i in range(e):
            row = []
            for j in range(e):
                if(i==j):
                    row.append(1)
                else:
                    row.append(0)
            I.append(row)

        return(Matrix(I))
    

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
        [-np.sin(j[0])*(l[1]*np.sin(j[1])+l[2]*np.sin(j[1]+j[2])), np.cos(j[0])*(l[1]*np.cos(j[1])+l[2]*np.cos(j[1]+j[2])), l[2]*np.cos(j[0])*np.cos(j[1]+j[2])],
        [np.cos(j[0])*(l[1]*np.sin(j[1])+l[2]*np.sin(j[1]+j[2])), np.sin(j[0])*(l[1]*np.cos(j[1])+l[2]*np.cos(j[1]+j[2])), l[2]*np.sin(j[0])*np.cos(j[1]+j[2])],
        [0, -(l[1]*np.sin(j[1])+l[2]*np.sin(j[1]+j[2])), -l[2]*np.sin(j[1]+j[2])]
    ])

def arm_draw(pf):

    ax.clear()

    for i in range(len(pf)-1):

        p0 = pf[i][:3]
        p1 = pf[i+1][:3]

        ax.quiver(
            p0[0], p0[1], p0[2],
            p1[0]-p0[0],
            p1[1]-p0[1],
            p1[2]-p0[2]
        )

    limit = 6

    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([-limit, limit])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    canvas.draw()

def joint_velocity(t):
    ## Desired angular joint velocity: [J0, J1, J2]
    return Vector([1, -1, -1])



# Main window
root = tk.Tk()
root.title("3D arm visualization")

# Global frame space definition
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1,1,1])

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()


frame = tk.Frame(root)
frame.pack()

##Velocity error display config.
vel_error = tk.StringVar()
vel_error.set("Angular error: ")

label_vel_error = tk.Label(frame, textvariable=vel_error)
label_vel_error.grid(row=6, columnspan=2)




##Zero vector
zero = Vector([0,0,0,1])

##End effector initial position
p_end = Vector([1,1,-2])

##Link lenght
l = Vector([1,2,2])

##Initial joint angles and cartesian positions
j=inverse_k(p_end, l)
p=direct_k(j, zero, l)

##Time steps
dt=0.001

##Initial time
t0 = time.perf_counter()

real_p_end=p_end

jaco_p_end=p_end


def step_physics(dt):
    global j, j_vel, jaco_p_end_vel, jaco_p_end, real_p_end_vel

    ##Actual time since initial time
    t = time.perf_counter() - t0

    ##Joint velocity update (if non constant velocity)
    j_vel = joint_velocity(t)

    ##End effector velocity calculated by comparing current joint angles DK position and updated joint angles using joint velocity DK position.
    real_p_end_vel = ((direct_k((j + j_vel*dt), zero, l)[3] - direct_k(j, zero, l)[3]) * (1 / dt))

    ##Jacobian calculated end effector velocity
    jaco_p_end_vel = jacobian(j, l).apply(j_vel)

    ##Update of end effector position using jacobian calculated end effector velocity
    jaco_p_end = jaco_p_end + jaco_p_end_vel * dt

    ##Update of joint angles using updated end effector position
    j = inverse_k(jaco_p_end, l)


def physics_loop():

    ## Update end effector position and cartesian velocity error every dt seconds.
    step_physics(dt)

    root.after(int(dt * 1000), physics_loop)


def render_loop():

    ## Update joint cartesian positions and draw updated arm at 30fps.
    p = direct_k(j, zero, l)

    arm_draw(p)

    root.after(int(1000 / 30), render_loop)

    ## Update and display cartesian velocity error.
    vel_error.set(
    f"j_dx={abs(jaco_p_end_vel[0]-real_p_end_vel[0]):.4f}  "
    f"j_dy={abs(jaco_p_end_vel[1]-real_p_end_vel[1]):.4f}  "
    f"j_dz={abs(jaco_p_end_vel[2]-real_p_end_vel[2]):.4f}"
    )



physics_loop()
render_loop()

root.mainloop()