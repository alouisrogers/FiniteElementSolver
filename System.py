import numpy as np
from numpy.linalg import inv
import math
from Node import Node
from Element import Element
from Force import Force

class System:

    def __init__(self, elements, nodes):
        self.elements = elements
        self.nodes = nodes
        self.boundaryConditions = []
        self.forces = np.zeros((len(self.nodes) * 2, 1))
        self.globalK = np.zeros((len(self.nodes) * 2, len(self.nodes) * 2))
        self.connections = []

    def addBoundaryCondition(self, boundaryCondition):
        self.boundaryConditions.append(boundaryCondition)
        return self

    def addForce(self, magnitude, theta, node):
        xcompenent = magnitude * math.cos(math.radians(theta))
        ycomponent = magnitude * math.sin(math.radians(theta))
        self.forces[node] = xcompenent
        self.forces[node + 1] = ycomponent

    def connect(self, node1, node2):
        self.connections.append(node1, node2)

    def kmatrix(self):

    def assemble(self):
        for element in self.elements:
            kmatrix = element.kMatrix()
            xdof = [element.getNode1().getNum(), element.getNode2().getNum()]
            ydof = [element.getNode1().getNum(), element.getNode2().getNum()]
            for xval in xdof:
                for yval in ydof:
                    quadrant = 0
                    for i in range(-1,1):
                        for j in range(-1,1):
                            quadrant += 1
                            self.globalK[2 * xval + i - 1][2 * yval + j - 1] \
                                += kmatrix[2 * xdof.index(xval) + 1 + i][2 * ydof.index(yval) + j + 1]
        return self.globalK


    def solve(self):
        print (self.globalK)
        for boundaryCondition in self.boundaryConditions:
            colrow = 2 * boundaryCondition[0] - 2
            if boundaryCondition[1] == 'x':
                colrow = colrow
            elif boundaryCondition[1] == 'y':
                colrow = colrow + 1
            self.globalK[colrow, :] = float('NaN')
            self.globalK[:, colrow] = float('NaN')
        kTemp = []
        for row in range(len(self.globalK)):
            temprow = []
            for col in range(len(self.globalK)):
                if not np.isnan(self.globalK[row][col]):
                    temprow.append(self.globalK[row][col])
            if len(temprow) > 0:
                kTemp.append(temprow)
        self.globalK = np.array(kTemp)

n1 = Node(0, 0, 1)
n2 = Node(1, 0, 2)
n3 = Node(2, 0, 3)
e1 = Element(elementNumber=1, node1=n1, node2=n2, strength=200e9, area=1200e-6)
e2 = Element(elementNumber=1, node1=n2, node2=n3, strength=200e9, area=1200e-6)
s = System([e1, e2], [n1, n2, n3])
s.addBoundaryCondition((1,'x')).addBoundaryCondition((1,'y')).addBoundaryCondition((3,'x')).addBoundaryCondition((3,'y'))
s.assemble()
s.solve()


