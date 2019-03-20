import numpy as np
import math
class Element:

    def __init__(self, elementNumber, node1, node2, strength, area):
        self.elementNumber = elementNumber
        self.strength = strength
        self.area = area
        if node1.getNum() < node2.getNum():
            self.node1 = node1
            self.node2 = node2
        else:
            self.node1 = node2
            self.node2 = node1

    def kMatrix(self):
        length = self.length()
        stiffness = (self.strength * self.area) / length
        if self.node1.getX() == self.node2.getX() and self.node2.getY() > self.node1.getY():
            theta = math.radians(90)
        elif self.node1.getX() == self.node2.getX() and self.node1.getY() > self.node2.getY():
            theta = math.radians(270)
        else:
            theta = math.radians(math.atan((self.node2.getY()-self.node1.getY())/(self.node2.getX()-self.node1.getX())))
        k = np.array([[pow(math.cos(theta), 2), math.cos(theta) * math.sin(theta), -1 * pow(math.cos(theta), 2),
                      -1 * math.cos(theta) * math.sin(theta)],
                     [math.cos(theta) * math.sin(theta), pow(math.sin(theta),2), -1 * math.cos(theta) * math.sin(theta),
                      -1 * pow(math.sin(theta), 2)],
                     [-1 * pow(math.cos(theta), 2), -1 * math.cos(theta) * math.sin(theta), pow(math.cos(theta), 2),
                      math.cos(theta) * math.sin(theta)],
                     [-1 * math.cos(theta) * math.sin(theta), -1 * pow(math.sin(theta), 2),
                      math.cos(theta) * math.sin(theta), pow(math.sin(theta), 2)]])
        k = stiffness * k
        for row in range(len(k)):
            for col in range(len(k)):
                if k[row][col] < 1 and k[row][col] > -1:
                    k[row][col] = 0
        return k

    def getNode1(self):
        return self.node1

    def getNode2(self):
        return self.node2

    def length(self):
        return math.sqrt(pow(self.node2.getX() - self.node1.getX(),2)
                         + pow(self.node2.getY() - self.node1.getY(), 2))

