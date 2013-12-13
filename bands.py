import numpy as np
import pygame
import pdb


def normalize(vector):
    return vector / np.linalg.norm(vector)


class Node:
    def __init__(self, pos = None, prv = None, nxt = None,
                 locked = False, mass = 1.0, jointRigidity = 1.0):
        self.pos = pos          # X, Y, V numpy array
        self.prv = prv          # previous node
        self.nxt = nxt          # next node
        self.locked = locked    # if true, forces have no effect on this node
        self.mass = mass        # 1 / how much a froce effects that node
        self.k = jointRigidity  # coefficient for the tension force
        self.dist = 0           # way to first node
        self.minDist = 20
        if self.prv:
            self.dist = self.getDistance()

    def insertNodeProg(self, pos, locked = False, mass = 1.0, jointRigidity = 1.0):
        # insert new node after
        self.nxt = Node(pos, self, self.nxt, locked, mass, jointRigidity)
        self.nxt.nxt.prv = self.nxt
        return

    def insertNodeRetro(self, pos, locked = False, mass = 1.0, jointRigidity = 1.0):
        # insert new node after
        self.prv = Node(pos, self.prv, self, locked, mass, jointRigidity)
        self.prv.prv.nxt = self.prv
        return


    def calcForces(self):
        totForce = np.array([0, 0, 0])
        if not self.locked:
            totForce = totForce + self.getTensionForce()
            totForce = totForce + self.getObstacleForce()
            totForce = totForce + self.getDVForce(3)
            totForce = totForce + self.getVForce()
        return totForce


    def applyForces(self):
        self.pos = self.pos + 1 / self.mass * self.calcForces()
        if self.pos[2] > 255:
            self.pos[2] = 255
        if self.pos[2] < 0:
            self.pos[2] = 0
        
    def getTensionForce(self):
        pos = self.pos[0:2]
        tensionForce = np.array([0, 0])

        if self.prv:
            prv = self.prv.pos[0:2]
            kPrv = 1 / (1 / self.k + 1 / self.prv.k)
            tensionForce = tensionForce + kPrv * (prv - pos) / \
                           np.linalg.norm((prv - pos))

        if self.nxt:
            nxt = self.nxt.pos[0:2]
            kNxt = 1 / (1 / self.k + 1 / self.nxt.k)
            tensionForce = tensionForce + kNxt * (nxt- pos) / \
                           np.linalg.norm((nxt- pos))

        return np.append(tensionForce, 0)

    def getObstacleForce(self):
        pos = self.pos[0:2]
        obstacleForce = np.array([0, 0])
        obsPos = np.array([150, 130])
        dist = np.linalg.norm(obsPos - pos)

        if dist < 30:
            obstacleForce = (pos - obsPos) / (dist*dist)

            if self.prv and self.nxt:
                prv = self.prv.pos[0:2]
                nxt = self.nxt.pos[0:2]
                obstacleForce = - np.linalg.norm(obstacleForce) * \
                                (normalize(nxt - pos) + \
                                normalize(prv - pos))

#           if self.prv and self.nxt:
#               obstacleForce = obstacleForce - obstacleForce * \
#                               ((self.prv.pos - self.nxt.pos) / \
#                               np.linalg.norm(self.prv.pos - self.nxt.pos)) * \
#                               ((self.prv.pos - self.nxt.pos) / \
#                               np.linalg.norm(self.prv.pos - self.nxt.pos))

        return np.append(obstacleForce * 20, 0)

    def getDVForce(self, aMax):
        force = np.array([0, 0, 0])
        accRetro = self.getAccelerationRetro()
        accProg = self.getAccelerationProg()

#       force[2] = force[2] + (accRetro + accProg)

        if accRetro > aMax:
            force[2] = force[2] + accRetro - aMax
        elif accRetro < -aMax:
            force[2] = force[2] + accRetro + aMax

        if accProg > aMax:
            force[2] = force[2] + accProg - aMax
        elif accProg < -aMax:
            force[2] = force[2] + accProg + aMax

        return force
        

#       force = np.array([0, 0, 0])
#       dVR = self.getDVDXYRetro()
#       dVP = self.getDVDXY()
#       if dVR > aMax:
#           force[2] = -dVR
#       elif dVP < -aMax:
#           force[2] = dVP
#       return force

    def smoothCurve(self, fPedMax):
        if self.getCurvature() > fPedMax:
            if np.linalg.norm(self.pos - self.nxt.pos) > self.minDist:
                self.insertNodeProg(self.pos + (self.nxt.pos - self.pos) / 3)
            if np.linalg.norm(self.pos - self.prv.pos) > self.minDist:
                self.insertNodeRetro(self.pos + (self.prv.pos - self.pos) / 3)


    def getXY(self):
        return (int(self.pos[0]), int(self.pos[1]))

    def getV(self):
        return int(self.pos[2])

    def getDVDXY(self):
        derivative = 0
        diff = np.array([0, 0, 0])

        if self.nxt:
            diff = self.nxt.pos - self.pos
            derivative = diff[2]
            diff[2] = 0
            derivative = derivative / np.linalg.norm(diff)
        return derivative

    def getDVDXYRetro(self):
        derivative = 0
        diff = np.array([0, 0, 0])

        if self.prv:
            diff = self.pos - self.prv.pos
            derivative = diff[2]
            diff[2] = 0
            derivative = derivative / np.linalg.norm(diff)
        return derivative

    def getAccelerationProg(self):
        acc = 0

        if self.nxt:
            diff = self.nxt.pos - self.pos
            diff[2] = 0
            acc = (self.nxt.pos[2]**2 - self.pos[2]**2) / np.linalg.norm(diff) / 2
        return acc

    def getAccelerationRetro(self):
        acc = 0

        if self.prv:
            diff = self.prv.pos - self.pos
            diff[2] = 0
            acc = (self.prv.pos[2]**2 - self.pos[2]**2) / np.linalg.norm(diff) / 2
        return acc


    def getCurvature(self):
        curvature = 0
        if self.nxt and self.prv:
            pos = self.pos[0:2]
            nxt = self.nxt.pos[0:2]
            prv = self.prv.pos[0:2]

            curvature = normalize(nxt - pos) - normalize(pos - prv) 
            curvature = self.pos[2] * np.linalg.norm(curvature)
        return curvature

    def getDistance(self):
        pos = self.pos[0:2]
        prv = self.prv.pos[0:2]
        return np.linalg.norm(pos - prv) + self.prv.dist

    def getVForce(self):
        return np.array([0, 0, 0.3])
        


     
if __name__ == "__main__":

    pygame.init()
    fpsClock = pygame.time.Clock()
    window = pygame.display.set_mode((640, 480))


    head = Node(np.array([90, 90, 0]), None, None, True)
    node = head

    positions = [[131, 100, 255],
                 [180,  80, 255],
                 [200, 150, 255],
                 [250, 190, 255],
                 [200, 250,   0]]

    for pos in positions:
        node.nxt = Node(np.array(pos), node, None)
        node = node.nxt

    node.locked = True

    node = head
    while(node):
        print(node.pos)
        node = node.nxt

    while True:
        window.fill(pygame.Color(0, 0, 0))
        pygame.draw.circle(window, pygame.Color(255, 255, 0), (150, 130), 30, 2)

        node = head
        while(node):
            if node.prv:
                pygame.draw.line(window, pygame.Color(150, 150, 150),
                                (node.prv.dist, node.prv.getV()),
                                (node.dist, node.getV()), 2)
                pygame.draw.line(window, pygame.Color(255, 0, 0),
                                (node.dist - 2, 0),
                                (node.dist - 2, node.getAccelerationRetro() * 20), 4)
            if node.nxt:
                pygame.draw.line(window, pygame.Color(255, 0, 0), node.getXY(),
                                node.nxt.getXY(), 4)
                pygame.draw.line(window, pygame.Color(0, 255, 0),
                                (node.dist + 2, 0),
                                (node.dist + 2, node.getAccelerationProg() * 20), 4)

            forceVector = node.pos + node.calcForces() * 100
            pygame.draw.line(window, pygame.Color(255, 255, 0),
                    node.getXY(), (forceVector[0], forceVector[1]), 1)
            pygame.draw.circle(window, pygame.Color(0, node.getV(), 200),
                    node.getXY(), 3, 3)



            node.applyForces()
            node.smoothCurve(10)
            node = node.nxt

        pygame.display.update()
        fpsClock.tick(30)
