import pygame
import math

import algorithms

pygame.init()


class ProgramRunner:
    WINDOWSIZE = 700
    TOP_PADDING = 30

    LIDAR_PIXEL_SIZE = 10
    LIDAR_ICON_CORNER_RADIUS = 2
    FIELD_SIZE = .8  # Percentage of the dimensions of the window that the field outline will take up
    ROBOT_BUILD_LINE_THICKNESS = 3
    ROBOT_BUILD_FRAME_SIZE = .50  # Percentage of the dimensions of the window that the robot frame
                                  # will take up
    ROBOT_BUILD_FRAME_THICKNESS = 2
    CENTER_ICON_PIXEL_SIZE = 5
    CENTER_ICON_CORNER_RADIUS = 3

    MIN_SQRDIST_BETWEEN_SEGMENT_POINTS = 5 * 5
    PATH_LINE_THICKNESS = 2

    SIM_ROBOT_SIZE = 80
    SIM_ROBOT_SPEED = 400  # Pixels Per Second
    SIM_ROBOT_LINE_THICKNESS = 2

    LIDAR_ANGLE_SUBDIVISIONS = 450
    LIDAR_ANGLE_INCREMENT = 2 * math.pi / LIDAR_ANGLE_SUBDIVISIONS
    LIDAR_RAY_THICKNESS = 1
    ALGORITHM_PLOT_LINE_THICKNESS = 1
    SIM_ROBOT_DOT_PIXEL_SIZE = 5
    SIM_ROBOT_DOT_CORNER_RADIUS = 5
    ALGORITHM_DOT_PIXEL_SIZE = 5
    ALGORITHM_DOT_CORNER_RADIUS = 5

    def __init__(self, *algorithms):
        # Pygame Related Variables
        self.screen = pygame.display.set_mode((self.WINDOWSIZE, self.WINDOWSIZE))
        self.clock = pygame.time.Clock()
        self.state = ProgramState.PLACING_LIDAR
        self.events = None

        # Temporary variables to store data inbetween frames of the same program state
        # ----
        self.lidarPos = None

        self.robotSegments = []
        self.segmentPrevPoint = None

        self.pathSegments = []

        self.algorithms = algorithms
        self.algorithmPaths = []
        self.algorithmPrevGuess = []
        self.algorithmErrors = []
        for algorithm in algorithms:
            self.algorithmPaths.append([])
            self.algorithmPrevGuess.append(None)
            self.algorithmErrors.append(0)

        self.finishedInitialSetup = False

        self.simRobotPos = None
        self.summedPathDists = []
        self.simRobotDistTraveled = 0
        self.simDirection = None
        self.simRobotSegments = []
        self.simPaused = False
        self.renderLidarLines = True
        self.currTimeStepsElapsedInLoop = 0

        pygame.display.set_caption("LIDAR Playground")

    # Code Inspired From: https://www.101computing.net/getting-started-with-pygame/
    # Main Program Loop
    def runProgram(self):
        keepRunning = True
        while keepRunning:
            dt = self.clock.tick(60)

            mouseClickedDown = False
            self.events = pygame.event.get()

            for event in self.events:
                if event.type == pygame.QUIT:
                    keepRunning = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mouseClickedDown = True
                # elif event.type == pygame.MOUSEBUTTONUP:
                #     mouseClickedup = True

            self.screen.fill(Colors.BLACK)

            match self.state:
                case ProgramState.PLACING_LIDAR:
                    self.placingLidar(mouseClickedDown)
                case ProgramState.BUILDING_ROBOT:
                    self.buildingRobot(mouseClickedDown)
                case ProgramState.MAKING_PATH:
                    self.makingPath()
                case ProgramState.RUNNING_SIM:
                    self.runSim(dt)

            pygame.display.flip()

        pygame.quit()

    def placingLidar(self, mouseClickedDown):
        """
        Handles allowing users to place down where the LIDAR is located
        """
        self.drawFieldBoundary()

        mousePos = pygame.mouse.get_pos()
        self.drawLidarIcon(mousePos)

        if mouseClickedDown and self.screenPointIsInField(mousePos):
            self.lidarPos = mousePos
            if (self.finishedInitialSetup):
                self.state = ProgramState.RUNNING_SIM
            else:
                self.state = ProgramState.BUILDING_ROBOT

        self.renderTextAtTop("Click To Place Lidar")

    def drawLidarIcon(self, lidarPos):
        """
        Helper Method that displays Lidar Icon
        """
        topLeftPixel = (lidarPos[0] - (self.LIDAR_PIXEL_SIZE / 2), lidarPos[1] - (self.LIDAR_PIXEL_SIZE / 2))
        lidarIcon = pygame.Rect(topLeftPixel[0], topLeftPixel[1], self.LIDAR_PIXEL_SIZE, self.LIDAR_PIXEL_SIZE)
        pygame.draw.rect(self.screen, Colors.RED, lidarIcon, 0, self.LIDAR_ICON_CORNER_RADIUS)

    def drawFieldBoundary(self):
        """
        Helper Method that displays the field
        """
        topLeftOffset = ((1 - self.FIELD_SIZE) / 2) * self.WINDOWSIZE
        fieldSize = self.WINDOWSIZE * self.FIELD_SIZE
        fieldOutline = pygame.Rect(topLeftOffset, topLeftOffset, fieldSize, fieldSize)
        pygame.draw.rect(self.screen, Colors.DARK_GRAY, fieldOutline)

    # Helper method that tests if a point on the screen is within the field on the screen
    def screenPointIsInField(self, point):
        """
        Helper method that tests if a point on the screen is within the field on the screen
        """
        xDistFromEdge = min(point[0], self.WINDOWSIZE - point[0])
        yDistFromEdge = min(point[1], self.WINDOWSIZE - point[1])
        minDist = min(xDistFromEdge, yDistFromEdge)
        borderOutsideFieldWidth = ((1 - self.FIELD_SIZE) * self.WINDOWSIZE) / 2
        return minDist >= borderOutsideFieldWidth

    # -------------------------------------------------------------------------------------

    def buildingRobot(self, mouseClickedDown):
        """
        Handles allowing the user to build the cross-section of the robot that
        the lIDAR will be able to pick up.
        """
        self.renderPrevSegmentsAndRobot()

        if self.keyDownstrokeDetected(pygame.K_e) and len(self.robotSegments) > 0:
            self.state = ProgramState.MAKING_PATH
            self.segmentPrevPoint = None
            self.convertScreenSegmentsToRobotCoords()

        elif self.keyDownstrokeDetected(pygame.K_z):
            if self.segmentPrevPoint != None:
                self.segmentPrevPoint = None
            elif len(self.robotSegments) > 0:
                self.robotSegments.pop()
        else:
            mousePos = pygame.mouse.get_pos()

            if self.segmentPrevPoint == None:
                if mouseClickedDown and self.screenPointIsInRobotFrame(mousePos):
                    self.segmentPrevPoint = mousePos
            else:
                pygame.draw.line(self.screen, Colors.GREEN, self.segmentPrevPoint, mousePos,
                                 self.ROBOT_BUILD_LINE_THICKNESS)
                if mouseClickedDown and self.screenPointIsInRobotFrame(mousePos) and self.segmentPrevPoint != mousePos:
                    self.robotSegments.append((self.segmentPrevPoint, mousePos))
                    self.segmentPrevPoint = None

        self.renderTextAtTop("Click To Place Line Segments of Robot [Z: Undo, E: Exit Creation Tool]")

    def renderPrevSegmentsAndRobot(self):
        """
        Helper method for buildingRobot() that draws the gray outline and previously drawn segments
        """
        # Robot Frame
        topLeftOffset = ((1 - self.ROBOT_BUILD_FRAME_SIZE) / 2) * self.WINDOWSIZE
        frameSize = self.WINDOWSIZE * self.ROBOT_BUILD_FRAME_SIZE
        robotFrame = pygame.Rect(topLeftOffset, topLeftOffset, frameSize, frameSize)
        pygame.draw.rect(self.screen, Colors.GRAY, robotFrame, self.ROBOT_BUILD_FRAME_THICKNESS)

        # Center Icon
        centerIconPos = ((self.WINDOWSIZE / 2) - (self.CENTER_ICON_PIXEL_SIZE / 2),
                         (self.WINDOWSIZE / 2) - (self.CENTER_ICON_PIXEL_SIZE / 2))
        centerIcon = pygame.Rect(centerIconPos[0], centerIconPos[1],
                                 self.CENTER_ICON_PIXEL_SIZE, self.CENTER_ICON_PIXEL_SIZE)
        pygame.draw.rect(self.screen, Colors.GRAY, centerIcon)

        for segment in self.robotSegments:
            pygame.draw.line(self.screen, Colors.GREEN, segment[0], segment[1],
                             self.ROBOT_BUILD_LINE_THICKNESS)

    def convertScreenPointToRobotCoords(self, point):
        """
        Takes the screen coordinate of a point placed in the robot builder section of the
        application and converts it into "robot coordinates." The edges of the gray bounding box
        represent the -1 and 1 lines of each coordinate in "robot coordinates."
        """
        centerCoord = self.WINDOWSIZE / 2
        robotSize = self.WINDOWSIZE * self.ROBOT_BUILD_FRAME_SIZE
        xPercent = (point[0] - centerCoord) / (robotSize / 2)
        yPercent = (point[1] - centerCoord) / (robotSize / 2)
        return (xPercent, yPercent)

    def screenPointIsInRobotFrame(self, point) -> bool:
        """
        Helper method for buildingRobot()
        Tests if a point on the screen is within the gray bounding box
        """
        robotCoords = self.convertScreenPointToRobotCoords(point)
        return abs(robotCoords[0]) <= 1 and abs(robotCoords[1]) <= 1

    def convertScreenSegmentsToRobotCoords(self):
        """
        Helper method for buildingRobot()
        The robotSegments variable contains the points of the robot segements in screenspace,
        but once the screenspace coordinates are not needed, this method converts them to
        robot coordinates.
        """
        for i in range(len(self.robotSegments)):
            segment = self.robotSegments[i]
            convertedFirstPoint = self.convertScreenPointToRobotCoords(segment[0])
            convertedSecondPoint = self.convertScreenPointToRobotCoords(segment[1])
            self.robotSegments[i] = (convertedFirstPoint, convertedSecondPoint)

    # ------------------------------------------------------------------------

    def makingPath(self):
        """
        Handles allowing the user to draw a path that the simulated robot will go along
        """
        self.drawFieldBoundary()

        self.renderPreviousPathSegments()

        if (self.keyDownstrokeDetected(pygame.K_e) and self.segmentPrevPoint == None):
            self.finishedInitialSetup = True
            self.state = ProgramState.RUNNING_SIM
        elif self.keyDownstrokeDetected(pygame.K_s):
            if self.segmentPrevPoint == None:
                if (len(self.pathSegments) == 0):
                    self.segmentPrevPoint = pygame.mouse.get_pos()  # Start Line Drawing
                else:
                    self.pathSegments = []  # Restart Line Drawing
            else:
                self.segmentPrevPoint = None  # Stop Line Drawing
        else:
            if self.segmentPrevPoint != None:
                mousePos = pygame.mouse.get_pos()
                pygame.draw.line(self.screen, Colors.GOLD, self.segmentPrevPoint, mousePos,
                                 self.PATH_LINE_THICKNESS)
                if (self.squaredDist(mousePos, self.segmentPrevPoint) >=
                        self.MIN_SQRDIST_BETWEEN_SEGMENT_POINTS):
                    if self.screenPointIsInField(mousePos):
                        self.pathSegments.append((self.segmentPrevPoint, mousePos))
                        self.segmentPrevPoint = mousePos

        self.drawLidarIcon(self.lidarPos)
        self.renderTextAtTop("Wave Mouse to Draw Robot Path [S: Start / Stop / Clear Drawing, E: Exit Path Builder]")

    def squaredDist(self, point1, point2):
        return ((point1[0] - point2[0]) * (point1[0] - point2[0]) +
                (point1[1] - point2[1]) * (point1[1] - point2[1]))

    def dist(self, point1, point2):
        return self.squaredDist(point1, point2) ** (1 / 2)

    def renderPreviousPathSegments(self):
        for segment in self.pathSegments:
            pygame.draw.line(self.screen, Colors.GOLD, segment[0], segment[1],
                             self.PATH_LINE_THICKNESS)

    # ----------------------------------------------------------------------

    def runSim(self, dt):
        """
        Method that handles one timestep of the simulation
        """
        self.drawFieldBoundary()
        self.renderPreviousPathSegments()
        self.drawLidarIcon(self.lidarPos)

        if self.keyDownstrokeDetected(pygame.K_SPACE):
            self.simPaused = not self.simPaused

        if self.keyDownstrokeDetected(pygame.K_t):
            self.renderLidarLines = not self.renderLidarLines

        if self.keyDownstrokeDetected(pygame.K_p):
            self.simRobotDistTraveled = 0
            self.state = ProgramState.MAKING_PATH
            for i in range(len(self.algorithms)):
                self.algorithmPaths[i] = []
                self.algorithmPrevGuess[i] = None
            self.simRobotPos = None

        elif self.keyDownstrokeDetected(pygame.K_l):
            self.state = ProgramState.PLACING_LIDAR
            for i in range(len(self.algorithms)):
                self.algorithmPaths[i] = []
                self.algorithmPrevGuess[i] = None
            self.simRobotPos = None

        elif self.simRobotPos is None:
            if len(self.pathSegments) == 0:
                self.simRobotPos = (self.WINDOWSIZE / 2, self.WINDOWSIZE / 2)
                self.simDirection = (0, 1)
                self.recalcSimRobotSegments()
            else:
                self.simRobotPos = self.pathSegments[0][0]
                self.calculateDistanceSums()
            self.renderSimRobot()

        else:
            if len(self.pathSegments) > 0:
                if not self.simPaused:
                    self.simRobotDistTraveled += (dt / 1000) * self.SIM_ROBOT_SPEED
                    self.currTimeStepsElapsedInLoop += 1
                directionVector, robotPositon = self.recalcSimRobotPosition()
                self.simDirection = directionVector
                self.simRobotPos = robotPositon
                self.recalcSimRobotSegments()

            lidarInfo = self.calculateAndDisplayLidarInfo()
            self.renderSimRobot()
            if len(lidarInfo) > 1:
                for i in range(len(self.algorithms)):
                    algorithm = self.algorithms[i]
                    positionGuess = algorithm.locateCenter(lidarInfo, self.screen, self)

                    topLeftPixel = (positionGuess[0] - (self.ALGORITHM_DOT_PIXEL_SIZE / 2),
                                    positionGuess[1] - (self.ALGORITHM_DOT_PIXEL_SIZE / 2))
                    dot = pygame.Rect(topLeftPixel[0], topLeftPixel[1], self.ALGORITHM_DOT_PIXEL_SIZE,
                                      self.ALGORITHM_DOT_PIXEL_SIZE)
                    pygame.draw.rect(self.screen, algorithm.plotColor(), dot, 0, self.ALGORITHM_DOT_CORNER_RADIUS)

                    if self.algorithmPrevGuess[i] != None:
                        self.algorithmPaths[i].append((self.algorithmPrevGuess[i], positionGuess))
                    self.algorithmPrevGuess[i] = positionGuess
                    if (not self.simPaused):
                        self.algorithmErrors[i] += self.dist(positionGuess, self.simRobotPos)
            self.renderAlgorithmPaths()

            self.renderRobotPositionDot(self.simRobotPos)
            self.renderTextAtTop("Running Sim [P: Redo Path, L: Replace Lidar, Space: Pause, T: Toggle Lidar Lines]")

    def calculateDistanceSums(self):
        """
        Calculates the total distance the robot travels after reach the end of every segment.
        """
        prevDist = 0
        self.summedPathDists = []
        for i in range(len(self.pathSegments)):
            segment = self.pathSegments[i]
            newDist = prevDist + self.dist(segment[0], segment[1])
            self.summedPathDists.append(newDist)
            prevDist = newDist

    def recalcSimRobotPosition(self):
        """
        Recalculates the position of the simulated robot's center
        """
        if self.simRobotDistTraveled > self.summedPathDists[len(self.summedPathDists) - 1]:
            for i in range(len(self.algorithms)):
                print(self.algorithms[i].name() + " had error: " + str(
                    self.algorithmErrors[i] / (self.SIM_ROBOT_SIZE * self.currTimeStepsElapsedInLoop)))
                self.algorithmPaths[i] = []
                self.algorithmPrevGuess[i] = None
                self.algorithmErrors[i] = 0
            self.simRobotDistTraveled %= self.summedPathDists[len(self.summedPathDists) - 1]
            print("------")
            self.currTimeStepsElapsedInLoop = 0
        for i in range(len(self.pathSegments)):
            if self.summedPathDists[i] >= self.simRobotDistTraveled:
                segmentRobotIsOn = self.pathSegments[i]

                lengthOfSegment = None
                if i == 0:
                    lengthOfSegment = self.summedPathDists[i]
                else:
                    lengthOfSegment = self.summedPathDists[i] - self.summedPathDists[i - 1]

                distFromEnd = self.summedPathDists[i] - self.simRobotDistTraveled
                percentFromEnd = distFromEnd / lengthOfSegment
                newRobotPos = self.scale(self.subtract(segmentRobotIsOn[0], segmentRobotIsOn[1]), percentFromEnd)
                newRobotPos = self.add(newRobotPos, segmentRobotIsOn[1])
                return (self.segmentToNormalizedVector(segmentRobotIsOn), newRobotPos)
        return (0, 0)

    def segmentToNormalizedVector(self, vector):
        """
        Takes segment of a path and returns the normalized vector pointing from its intitial to terminal point
        """
        return self.scale(self.subtract(vector[1], vector[0]), 1 / self.dist(vector[0], vector[1]))

    def recalcSimRobotSegments(self):
        """
        Recalculates the locations of all the segments of the robot
        """
        simRobotSegments = []
        for segment in self.robotSegments:
            robotJHat = self.scale(self.simDirection, (-0.5) * self.SIM_ROBOT_SIZE)
            robotIHat = (-robotJHat[1], robotJHat[0])
            convertedSegFirstPoint = self.matrixMultiply(robotIHat, robotJHat, segment[0])
            convertedSegSecondPoint = self.matrixMultiply(robotIHat, robotJHat, segment[1])
            convertedSegFirstPoint = self.add(convertedSegFirstPoint, self.simRobotPos)
            convertedSegSecondPoint = self.add(convertedSegSecondPoint, self.simRobotPos)
            simRobotSegments.append((convertedSegFirstPoint, convertedSegSecondPoint))
        self.simRobotSegments = simRobotSegments

    # Unholy methods that I implemented because didn't want to import numPy
    def matrixMultiply(self, iHat, jHat, point):
        return self.add(self.scale(iHat, point[0]), self.scale(jHat, point[1]))

    def add(self, point1, point2):
        return (point1[0] + point2[0], point1[1] + point2[1])

    def subtract(self, point1, point2):
        return (point1[0] - point2[0], point1[1] - point2[1])

    def scale(self, point, c):
        return (c * point[0], c * point[1])

    def floatPointToInt(self, point):
        return (int(point[0]), int(point[1]))

    # 2D Determinant
    def crossProduct(self, vec1, vec2):
        return vec1[0] * vec2[1] - vec2[0] * vec1[1]

    def calculateAndDisplayLidarInfo(self):
        """
        Calculates point cloud that simulated LIDAR sees and optionally displays the LIDAR beams
        """
        hitPoints = []
        angle = 0
        for i in range(self.LIDAR_ANGLE_SUBDIVISIONS):
            angle += self.LIDAR_ANGLE_INCREMENT
            rayVector = (math.cos(angle), math.sin(angle))
            minDistFromLidar = 2 * self.WINDOWSIZE
            for segment in self.simRobotSegments:
                segmentVector = self.subtract(segment[1], segment[0])
                lidarDisplacement = self.subtract(segment[0], self.lidarPos)
                dispCrossRay = self.crossProduct(lidarDisplacement, rayVector)
                rayCrossSeg = self.crossProduct(rayVector, segmentVector)

                if rayCrossSeg != 0:
                    rayHitsSegment = False
                    if rayCrossSeg > 0:
                        rayHitsSegment = 0 <= dispCrossRay <= rayCrossSeg
                    else:
                        rayHitsSegment = 0 >= dispCrossRay >= rayCrossSeg

                    if rayHitsSegment:
                        hitDist = self.crossProduct(lidarDisplacement, segmentVector) / rayCrossSeg
                        if hitDist < minDistFromLidar and hitDist > 0:
                            minDistFromLidar = hitDist

            if minDistFromLidar < 2 * self.WINDOWSIZE:
                hitPosition = self.add(self.lidarPos, self.scale(rayVector, minDistFromLidar))
                hitPoints.append(hitPosition)
                if (self.renderLidarLines):
                    pygame.draw.line(self.screen, Colors.WHITE, self.lidarPos, hitPosition, self.LIDAR_RAY_THICKNESS)

        return hitPoints

    def renderAlgorithmPaths(self):
        """
        Plots the paths that Algorithm objects passed to the ProgramRunner have estimated that the robot has taken
        """
        for i in range(len(self.algorithms)):
            algorithmPath = self.algorithmPaths[i]
            for segment in algorithmPath:
                pygame.draw.line(self.screen, self.algorithms[i].plotColor(), segment[0], segment[1],
                                 self.ALGORITHM_PLOT_LINE_THICKNESS)

    def renderRobotPositionDot(self, robotPos):
        """
        Draws the gold dot indicating the actual position of the center of the robot.
        """
        topLeftPixel = (
        robotPos[0] - (self.SIM_ROBOT_DOT_PIXEL_SIZE / 2), robotPos[1] - (self.SIM_ROBOT_DOT_PIXEL_SIZE / 2))
        dot = pygame.Rect(topLeftPixel[0], topLeftPixel[1], self.SIM_ROBOT_DOT_PIXEL_SIZE,
                          self.SIM_ROBOT_DOT_PIXEL_SIZE)
        pygame.draw.rect(self.screen, Colors.GOLD, dot, 0, self.SIM_ROBOT_DOT_CORNER_RADIUS)

    def renderSimRobot(self):
        """
        Draws the simulated robot's cross-section.
        """
        for segment in self.simRobotSegments:
            pygame.draw.line(self.screen, Colors.GREEN, segment[0], segment[1],
                             self.SIM_ROBOT_LINE_THICKNESS)

    # ------------------------------------------------------------------------------

    def keyDownstrokeDetected(self, key) -> bool:
        """
        Tests if a passed key has a new detected downwards keystroke
        """
        for event in self.events:
            if event.type == pygame.KEYDOWN:
                if event.key == key:
                    return True
        return False

    def renderTextAtTop(self, displayString):
        """
        Displays a passed string to the top of the window
        """
        text = Fonts.DEFAULT.render(displayString, True, Colors.WHITE);
        topLeftCoords = (self.WINDOWSIZE / 2 - (Fonts.DEFAULT.size(displayString)[0] / 2), self.TOP_PADDING)
        self.screen.blit(text, topLeftCoords)


# Various Classes Used in ProgramRunner

class Colors:
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GREEN = (0, 255, 0)
    GRAY = (60, 60, 60)
    DARK_GRAY = (20, 20, 20)
    RED = (255, 0, 0)
    GOLD = (224, 175, 104)
    PINK = (198, 118, 222)


class ProgramState:
    PLACING_LIDAR = 0
    BUILDING_ROBOT = 1
    MAKING_PATH = 2
    RUNNING_SIM = 3


class Fonts:
    DEFAULT = pygame.font.SysFont(None, 24)


class LidarInfo:
    def __init__(self, angleIncrement, radialDistances, lidarPos):
        self.angleIncrement = angleIncrement
        self.radialDistances = radialDistances
        self.lidarPos = lidarPos


class Algorithm:
    def locateCenter(hitPoints, screen):
        pass

    def plotColor(self):
        pass


averagingAlgorithm = algorithms.AveragingAlgorithm()
secondAvergagingAlgorithm = algorithms.AveragingAlgorithmWithFilledSides()
slidingBoxAlgorithm = algorithms.SlidingBoxAlgorithm()
program = ProgramRunner(averagingAlgorithm, slidingBoxAlgorithm)
program.runProgram()
