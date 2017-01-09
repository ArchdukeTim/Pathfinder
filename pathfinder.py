import math
import matplotlib.pyplot as plt
import copy

class PathFinder:

    def __init__(self, path):
        self.origPath = path

        self.pathAlpha = .7
        self.pathBeta = .3
        self.pathTolerance = .0000001

        self.velocityAlpha = .1
        self.velocityBeta = .3
        self.velocityTolerance = .0000001

    @staticmethod
    def printPath(path):
        print("X: \t Y:")
        for j in path:
            print(j)


    def inject(self, orig, numToInject):
        morePoints = [None] * (len(orig) + ((numToInject) * (len(orig) - 1)))
        index = 0
        for i in range(0, len(orig) - 1):
            morePoints[index] = [orig[i][0], orig[i][1]]
            index += 1

            for j in range(1, numToInject + 1):
                morePoints[index] = [j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0],
                                    j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1]]

                index += 1

        morePoints[index] = [orig[-1][0], orig[-1][1]]
        return morePoints

    def smoother(self, path, weight_data, weight_smooth, tolerance):

        newPath = copy.deepcopy(path)
        change = copy.copy(tolerance)
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                for j in range(0, 2):
                    aux = newPath[i][j]
                    newPath[i][j] += (weight_data * (path[i][j] - newPath[i][j])) + (weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j])))
                    change += abs(aux - newPath[i][j])


        return newPath

    @staticmethod
    def nodeOnlyWayPoints(path):
        li = []
        li.append(path[0])

        for i in range(1, len(path) - 1):
            vector1 = math.atan2((path[i][1] - path[i - 1][1]), path[i][0] - path[i - 1][0])
            vector2 = math.atan2((path[i + 1][1] - path[i][1]), path[i + 1][0] - path[i][0])

            if abs(vector2 - vector1) >= .01:
                li.append(path[i])

        li.append(path[-1])

        return li


    def velocity(self, smoothPath, timeStep):
        # self.printPath(smoothPath)
        dxdt = [None] * len(smoothPath)
        dydt = [None] * len(smoothPath)
        velocity = [[[] for i in range(2)] for j in range(len(smoothPath))]

        dxdt[0] = 0
        dydt[0] = 0
        velocity[0][0] = 0
        velocity[0][1] = 0
        self.heading[0][1] = 0

        for i in range(1, len(smoothPath)):
            dxdt[i] = ((smoothPath[i][0] - smoothPath[i - 1][0]) / timeStep)
            dydt[i] = ((smoothPath[i][1] - smoothPath[i - 1][1]) / timeStep)

            velocity[i][0] = velocity[i - 1][0] + timeStep

            self.heading[i][0] = self.heading[i - 1][0] + timeStep

            velocity[i][1] = math.sqrt(dxdt[i] ** 2 + dydt[i] ** 2)

        return velocity


    def velocityFix(self, smoothVelocity, origVelocity, tolerance):
        difference = self.errorSum(origVelocity, smoothVelocity)

        fixVel = copy.deepcopy(smoothVelocity)

        increase = 0.0
        while abs(difference[len(difference) - 1]) > tolerance:
            increase = difference[len(difference) - 1] / 1 / 50

            for i in range(1, len(fixVel) - 1):
                fixVel[i][1] = fixVel[i][1] - increase

            difference = self.errorSum(origVelocity, fixVel)

        return fixVel


    def errorSum(self, origVelocity, smoothVelocity):
        tempOrigDist = copy.deepcopy(origVelocity)
        tempSmoothDist = copy.deepcopy(smoothVelocity)
        difference = copy.deepcopy(origVelocity)

        timeStep = origVelocity[1][0] - origVelocity[0][0]

        tempOrigDist[0] = origVelocity[0][1]
        tempSmoothDist[0] = smoothVelocity[0][1]

        for i in range(1, len(origVelocity)):
            tempOrigDist[i] = origVelocity[i][1] * timeStep + tempOrigDist[i - 1]
            tempSmoothDist[i] = smoothVelocity[i][1] * timeStep + tempSmoothDist[i - 1]

            difference[i] = tempSmoothDist[i] - tempOrigDist[i]

        return difference


    def injectionCounter2Steps(self, numNodeOnlyPoints, maxTimeToComplete, timeStep):

        first = second = third = 0
        oldPointsTotal = 0

        self.numFinalPoints = 0

        totalPoints = maxTimeToComplete / timeStep

        if totalPoints < 100:
            pointsFirst = 0
            pointsTotal = 0

            for i in range (4, 7):
                for j in range(1, 9):
                    pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints
                    pointsTotal = (j * (pointsFirst - 1) + pointsFirst)

                    if pointsTotal <= totalPoints and pointsTotal > oldPointsTotal:
                        first = i;
                        second = j;
                        self.numFinalPoints = pointsTotal;
                        oldPointsTotal = pointsTotal;

            ret = [first, second, third]

        else:

            for i in range (1, 6):
                for j in range(1, 9):
                    for k in range(1, 8):
                        pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints
                        pointsSecond = (j * (pointsFirst - 1) + pointsFirst)
                        pointsTotal = (k * (pointsSecond - 1) + pointsSecond)

                        if pointsTotal <= totalPoints:
                            first = i;
                            second = j;
                            third = k;
                            self.numFinalPoints = pointsTotal;

            ret = [first, second, third]

        return ret

    def leftRight(self, smoothPath, robotTrackWidth):
        leftPath = copy.deepcopy(smoothPath)
        rightPath = copy.deepcopy(smoothPath)
        gradient = copy.deepcopy(smoothPath)

        for i in range(0, len(smoothPath) - 1):
            gradient[i][1] = math.atan2(smoothPath[i + 1][1] - smoothPath[i][1], smoothPath[i + 1][0] - smoothPath[i][0]);

        gradient[len(gradient) - 1][1] = gradient[len(gradient) - 2][1];

        for i in range(0, len(gradient)):
            leftPath[i][0] = robotTrackWidth / 2 * math.cos(gradient[i][1] + math.pi / 2) + smoothPath[i][0];
            leftPath[i][1] = robotTrackWidth / 2 * math.sin(gradient[i][1] + math.pi / 2) + smoothPath[i][1];

            rightPath[i][0] = robotTrackWidth / 2 * math.cos(gradient[i][1] - math.pi / 2) + smoothPath[i][0];
            rightPath[i][1] = robotTrackWidth / 2 * math.sin(gradient[i][1] - math.pi / 2) + smoothPath[i][1];


            deg = math.degrees(gradient[i][1]);

            gradient[i][1] = deg;

            if i > 0:

                if (deg - gradient[i - 1][1]) > 180:
                    gradient[i][1] = -360 + deg;

                if (deg - gradient[i - 1][1]) < -180:
                    gradient[i][1] = 360 + deg;


        self.heading = gradient
        self.leftPath = leftPath
        self.rightPath = rightPath


    @staticmethod
    def getXVector(arr):
        temp = []
        for i in arr:
            temp.append(i[0])
        return temp

    @staticmethod
    def getYVector(arr):
        temp = []
        for i in arr:
            temp.append(i[1])
        return temp

    @staticmethod
    def transposeVector(arr):
        temp = copy.deepcopy(arr)
        for i in temp:
            i = i[1], i[0]

        return temp

    def setPathAlpha(self, alpha):
        self.pathAlpha = alpha

    def setPathBeta(self, beta):
        self.pathBeta = beta

    def setPathTolerance(self, tolerance):
        self.pathTolerance = tolerance

    def calculate(self, totalTime, timeStep, robotTrackWidth):
        self.nodeOnlyPath = self.nodeOnlyWayPoints(self.origPath)

        inject = self.injectionCounter2Steps(len(self.nodeOnlyPath), totalTime, timeStep)
        for i in range(0, len(inject)):
            if i == 0:
                self.smoothPath = self.inject(self.nodeOnlyPath, inject[0])
                self.smoothPath = self.smoother(self.smoothPath, self.pathAlpha, self.pathBeta, self.pathTolerance)

            else:
                self.smoothPath = self.inject(self.smoothPath, inject[i])
                self.smoothPath = self.smoother(self.smoothPath, .1, .3, .0000001)

        self.leftRight(self.smoothPath, robotTrackWidth)

        self.printPath(self.leftPath)
        self.printPath(self.rightPath)

        self.origCenterVelocity = self.velocity(self.smoothPath, timeStep)
        self.origLeftVelocity = self.velocity(self.leftPath, timeStep)
        self.origRightVelocity = self.velocity(self.rightPath, timeStep)



        self.smoothCenterVelocity = copy.deepcopy(self.origCenterVelocity)
        self.smoothLeftVelocity = copy.deepcopy(self.origLeftVelocity)

        self.smoothRightVelocity = copy.deepcopy(self.origRightVelocity)
        self.smoothCenterVelocity[-1][1] = 0.0
        self.smoothLeftVelocity[-1][1] = 0.0
        self.smoothRightVelocity[-1][1] = 0.0



        self.smoothCenterVelocity = self.smoother(self.smoothCenterVelocity, self.velocityAlpha, self.velocityBeta, self.velocityTolerance)
        self.smoothLeftVelocity = self.smoother(self.smoothLeftVelocity,  self.velocityAlpha, self.velocityBeta, self.velocityTolerance)
        self.smoothRightVelocity = self.smoother(self.smoothRightVelocity, self.velocityAlpha, self.velocityBeta, self.velocityTolerance)


        self.smoothCenterVelocity = self.velocityFix(self.smoothCenterVelocity, self.origCenterVelocity, 0.0000001);
        self.smoothLeftVelocity = self.velocityFix(self.smoothLeftVelocity, self.origLeftVelocity, 0.0000001);
        self.smoothRightVelocity = self.velocityFix(self.smoothRightVelocity, self.origRightVelocity, 0.0000001);



def run():

    waypoints = [[ 1.0, 1.0 ], [ 5.0, 1.0 ], [ 9.0, 12.0 ], [ 12.0, 9.0 ], [ 15.0, 6.0 ], [ 19.0, 12.0 ]]

    totalTime = 5
    timeStep = .05
    robotTrackWidth = 2

    path = PathFinder(waypoints)
    path.calculate(totalTime, timeStep, robotTrackWidth)
    plt.figure(1)
    plt.plot(*zip(*path.smoothCenterVelocity), 'b')
    plt.plot(*zip(*path.smoothLeftVelocity), 'r')
    plt.plot(*zip(*path.smoothRightVelocity), 'g')
    plt.show()

    plt.figure(2)
    plt.plot(*zip(*path.smoothPath), 'b')
    plt.plot(*zip(*path.origPath), 'r--')
    plt.plot(*zip(*path.origPath), 'ro')
    plt.axis([0, 27, 0, 24])
    plt.show()

if __name__ == '__main__':
    run()
