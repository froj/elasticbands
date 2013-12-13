import numpy as np
import pygame


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
        self.dist = 0
        if self.prv:
            self.dist = self.getDistance()

    def calcForces(self):
        totForce = np.array([0, 0, 0])
        if not self.locked:
            totForce = totForce + self.getTensionForce()
            totForce = totForce + self.getObstacleForce()
            totForce = totForce + self.getDVForce(2)
        return totForce


    def applyForces(self):
        self.pos = self.pos + 1 / self.mass * self.calcForces()
        
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
        dVR = self.getDVDXYRetro()
        dVP = self.getDVDXY()
        if dVR > aMax:
            force[2] = -dVR
        elif dVP < -aMax:
            force[2] = dVP
        return force

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

    def getCurvature(self):
        curvature = 0
        if self.nxt:
            pos = self.pos
            nextPos = self.nxt.pos
            prevPos = self.prv.pos
            pos[2] = 0
            nextPos[2] = 0
            prevPos[2] = 0

            curvature = normalize(nextPos - pos) - normalize(pos - prevPos) 
            curvature = self.pos[2] * np.linalg.norm(curvature)
        return curvature

    def getDistance(self):
        pos = self.pos[0:2]
        prv = self.prv.pos[0:2]
        return np.linalg.norm(pos - prv) + self.prv.dist
        


     
if __name__ == "__main__":

    pygame.init()
    fpsClock = pygame.time.Clock()
    window = pygame.display.set_mode((640, 480))


    head = Node(np.array([90, 90, 0]), None, None, True)
    node = head

    positions = [[131, 100, 255],
                 [180,  80, 255],
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
            if node.nxt:
                pygame.draw.line(window, pygame.Color(255, 0, 0), node.getXY(),
                                node.nxt.getXY(), 4)

            forceVector = node.pos + node.calcForces() * 100
            pygame.draw.line(window, pygame.Color(255, 255, 0),
                    node.getXY(), (forceVector[0], forceVector[1]), 1)
            pygame.draw.circle(window, pygame.Color(0, node.getV(), 200),
                    node.getXY(), 3, 3)



            node.applyForces()
            node = node.nxt

        pygame.display.update()
        fpsClock.tick(30)
