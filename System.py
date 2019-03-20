import numpy
import math
import itertools

class System:

    def __init__(self, elements, modulus, area, nodes, fixedNodes, connectivity, forces):
        self.elements = elements
        self.nodes = nodes
        self.fixedNodes = fixedNodes
        self.forces = numpy.zeros((len(self.nodes) * 2, 1))
        self.addForces(forces)
        self.kglobal = numpy.zeros((len(self.nodes) * 2, len(self.nodes) * 2))
        self.connectivity = connectivity
        self.modulus = modulus
        self.area = area

    def addForces(self, forces):
        self.forces = numpy.zeros((len(self.nodes) * 2, 1))
        for force in forces:
            if force.getDirection() == 'x':
                self.forces[2 * force.getNode()] += force.getMagnitude()
            if force.getDirection() == 'y':
                self.forces[2 * force.getNode() + 1] += force.getMagnitude()

    def assemble(self):
        for i in range(self.elements):
            nodeA, nodeB = self.connectivity[i]
            if nodeA > nodeB:
                temp = nodeA
                nodeA = nodeB
                nodeB = temp
            Ax = self.nodes[nodeA][0]
            Ay = self.nodes[nodeA][1]
            Bx = self.nodes[nodeB][0]
            By = self.nodes[nodeB][1]
            length = math.sqrt((math.pow(Ax - Bx, 2) + math.pow(Ay - By, 2)))
            theta = math.radians(math.atan(By - Ay / (Bx - Ax)))
            k = ((self.modulus * self.area) / length) * numpy.array([[pow(math.cos(theta), 2), math.cos(theta)
                                                                      * math.sin(theta), -1 * pow(math.cos(theta), 2),
                              -1 * math.cos(theta) * math.sin(theta)],
                             [math.cos(theta) * math.sin(theta), pow(math.sin(theta), 2),
                              -1 * math.cos(theta) * math.sin(theta),
                              -1 * pow(math.sin(theta), 2)],
                             [-1 * pow(math.cos(theta), 2), -1 * math.cos(theta) * math.sin(theta),
                              pow(math.cos(theta), 2),
                              math.cos(theta) * math.sin(theta)],
                             [-1 * math.cos(theta) * math.sin(theta), -1 * pow(math.sin(theta), 2),
                              math.cos(theta) * math.sin(theta), pow(math.sin(theta), 2)]])
            indexes = [2 * nodeA, 2 * nodeA + 1, 2 * nodeB, 2 * nodeB + 1]
            inputData = [indexes, indexes]
            positions = list(itertools.product(*inputData))
            k = k.flatten()
            for idx in range(len(positions)):
                self.kglobal[positions[idx][0]][positions[idx][1]] += k[idx]

    def applyBoundaryConditions(self):
        removed_one = []
        for i in range(len(self.fixedNodes)):
            removed_one.append(self.fixedNodes[i] * 2)
            removed_one.append(self.fixedNodes[i] * 2 + 1)

        removed_one.sort(reverse=True)
        for pos in removed_one:
            self.kglobal = numpy.delete(self.kglobal, (pos), axis=0)
            self.kglobal = numpy.delete(self.kglobal, (pos), axis=1)
            self.forces = numpy.delete(self.forces, (pos))

        zeroColumns = numpy.all(numpy.abs(self.kglobal) < 1e-5, axis=0)
        zeroRows = numpy.all(numpy.abs(self.kglobal) < 1e-5, axis=1)
        for row in zeroRows:
            self.forces = numpy.delete(self.forces, (row))

        self.kglobal = self.kglobal[:, ~numpy.all(numpy.abs(self.kglobal) < 1e-5, axis=0)]
        self.kglobal = self.kglobal[~numpy.all(numpy.abs(self.kglobal) < 1e-5, axis=1)]

    def solve(self):
        self.assemble()
        self.applyBoundaryConditions()
        return numpy.matmul(numpy.linalg.inv(self.kglobal), self.forces)




