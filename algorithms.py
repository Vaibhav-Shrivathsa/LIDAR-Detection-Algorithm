import pygame
import math
import numpy as np


class Algorithm:
    """
    Class representing an algorithm to calculate the center of a robot
    given the cartesian points that the LIDAR detects of the robots
    """

    def locateCenter(self, hitPoints, screen, programRunner):
        """
        Calculates the center of a robot given the
        points that the LIDAR detects.
        Screen is the pygame object used to display to the screen.
        It is passed as an argument for debugging uses.
        programRunner is the entire object from playground.py that runs the program
        """
        pass

    def plotColor(self):
        """
        Returns color of trail that programRunner should use to plot the algorithm's estimations
        """
        pass

    def name(self):
        """
        Returns name of algorithm
        """
        pass


class AveragingAlgorithm(Algorithm):
    """
    Simply average all the hit points
    """
    def __init__(self):
        pass

    def locateCenter(self, hitPoints, screen, programRunner):
        average = [0, 0]
        for point in hitPoints:
            average[0] += point[0]
            average[1] += point[1]
        return average[0] / len(hitPoints), average[1] / len(hitPoints)

    def plotColor(self):
        return 200, 80, 160

    def name(self):
        return "Averaging Algorithm"


class AveragingAlgorithmWithFilledSides(Algorithm):
    """
    Constructs line segments between the hit points and includes the points
    on these line segments in the average.
    """
    def __init__(self):
        pass

    def locateCenter(self, hitPoints, screen, programRunner):
        average = [0, 0]
        totalDistance = 0
        prevPoint = None
        for point in hitPoints:
            if prevPoint != None:
                averageWithPrev = self.avgPoints(prevPoint, point)
                dist = math.dist(point, prevPoint)
                average[0] += averageWithPrev[0] * dist
                average[1] += averageWithPrev[1] * dist
                totalDistance += dist
            prevPoint = point

        return average[0] / totalDistance, average[1] / totalDistance

    def plotColor(self):
        return 173, 216, 230

    def avgPoints(self, point1, point2):
        return (point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2

    def name(self):
        return "Averaging Algorithm with Filled Sides"


class SlidingBoxAlgorithm(Algorithm):
    """
    Algorithm that calculates the possible set of (x, y, theta) the robot's center may be in
    and computes the average (x, y) of the set, giving equal weight to all (x, y, theta) points.
    """
    def __init__(self):
        self.ANGLE_SUBDIVISIONS = 8
        self.LEEWAY_FACTOR = 1.1

        self.ANGLE_INCREMENT = (math.pi / 2) / self.ANGLE_SUBDIVISIONS

        c, s = np.cos(self.ANGLE_INCREMENT), np.sin(self.ANGLE_INCREMENT)
        self.rotByAngleIncrement = np.array(((c, -s), (s, c)))
        self.rotByAngleIncrementInv = np.array(((c, s), (-s, c)))

    def locateCenter(self, hitPoints, screen, programRunner):
        if len(hitPoints) == 0:
            return np.array([0, 0])

        center = np.array([0, 0])
        totalArea = 0

        hitPoints = list(map(lambda x: np.asarray(x), hitPoints))

        rotByCurrAngleInv = np.identity(2)

        for i in range(self.ANGLE_SUBDIVISIONS):
            maxX = max(map(lambda point: point[0], hitPoints))
            maxY = max(map(lambda point: point[1], hitPoints))
            minX = min(map(lambda point: point[0], hitPoints))
            minY = min(map(lambda point: point[1], hitPoints))

            # Since we are not checking all infinite possible
            # rotations of valid squares, we use a valid square size
            # slightly larger than the actual regulation sized square
            # to give us some leeway when finding valid squares
            robotWidth = programRunner.SIM_ROBOT_SIZE * self.LEEWAY_FACTOR
            halfRobotWidth = robotWidth / 2

            # Check if there are valid squares of the current rotation
            if maxX - minX <= robotWidth and maxY - minY <= robotWidth:
                centerMaxX = minX + halfRobotWidth
                centerMaxY = minY + halfRobotWidth
                centerMinX = maxX - halfRobotWidth
                centerMinY = maxY - halfRobotWidth

                area = (centerMaxX - centerMinX) * (centerMaxY - centerMinY)
                totalArea += area
                middleOfRectangle = np.array([(centerMinX + centerMaxX) / 2, (centerMinY + centerMaxY) / 2])
                middleOfRectangle = rotByCurrAngleInv.dot(middleOfRectangle)
                center = center + middleOfRectangle * area

                # Draws the out the rectangles in which the robot's center may be
                # rectPoints = [np.array([centerMaxX, centerMinY]),
                #               np.array([centerMaxX, centerMaxY]),
                #               np.array([centerMinX, centerMaxY]),
                #               np.array([centerMinX, centerMinY])]
                # rectPoints = list(map(lambda x: tuple(rotByCurrAngleInv.dot(x)), rectPoints))
                # pygame.draw.polygon(screen, (100, 200, 255), rectPoints, 1)

            hitPoints = list(map(lambda x: self.rotByAngleIncrement.dot(x), hitPoints))
            rotByCurrAngleInv = self.rotByAngleIncrementInv.dot(rotByCurrAngleInv)

        if totalArea == 0:
            return 0, 0
        return tuple(center / totalArea)

    def plotColor(self):
        return 42, 195, 222

    def name(self):
        return "Sliding Box Algorithm"
