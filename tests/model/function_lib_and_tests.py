"""
Multiplies two matrix. 
Returns C=A*B. 
Vectors are not admitted.
Matrix have the shape:
[[row 0],
 [row 1],
 [...],
 [row n]]
"""
def matrix_mult(A, B):
    C=[]
    for i in range(len(A)):
        C.append([])
        for j in range(len(B[0])):
            d=0
            for k in range(len(A[0])):
                d=d+A[i][k]*B[k][j]
            C[i].append(d)
    return(C)


"""
Applies a transformation to a vector.

Returns r=A*x.
A is a matrix.
v is a vector.

A has the shape:
[[row 0],
 [row 1],
 [...],
 [row n]]

v is a row vector, but it acts as if it was a column vector while calculating the transformation.

r is a row vector.

The decision of using row vectors is due to the fact that they are easier to use and easier to define in python. numpy library wasn't used beacuse it produces undesired behaviours 
and it's functions cannot be fully controlled.
"""
def linear_aplication(A, v):
    r=[]
    for i in range(len(A)):
        d=0
        for k in range(len(A[0])):
            d=d+A[i][k]*v[k]
        r.append(d)
    return(r)



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



print(Matrix.inverse(Matrix([[1,2],[3,4]])))