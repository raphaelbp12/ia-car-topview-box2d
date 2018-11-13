import math
from Box2D import (b2BodyDef, b2CircleShape, b2Color, b2EdgeShape,
                   b2FixtureDef, b2PolygonShape, b2RayCastCallback, b2Vec2,
                   b2_dynamicBody, b2_pi)


class RayCastMultipleCallback(b2RayCastCallback):
    """This raycast collects multiple hits."""

    def __repr__(self):
        return 'Multiple hits'

    def __init__(self, **kwargs):
        b2RayCastCallback.__init__(self, **kwargs)
        self.fixtures = []
        self.hit = False
        self.points = []
        self.normals = []

    def ReportFixture(self, fixture, point, normal, fraction):
        self.hit = True
        self.fixtures.append(fixture)
        self.points.append(b2Vec2(point))
        self.normals.append(b2Vec2(normal))
        return 1.0

class Sensors:
    def __init__(self, world):
        self.world = world

        # self.laserDegrees = [b2_pi, 0]
        self.laserDegrees = [0, 1.0/8 * b2_pi, 2.0/8 * b2_pi, 3.0/8 * b2_pi, 4.0/8* b2_pi, 5.0/8 * b2_pi, 6.0/8 * b2_pi, 7.0/8 * b2_pi, 8.0/8 *b2_pi, 9.0/8 * b2_pi, 10.0/8 * b2_pi, 11.0/8 * b2_pi, 12.0/8 *b2_pi, 13.0/8 * b2_pi, 14.0/8 * b2_pi, 15.0/8 * b2_pi]
        self.laserPoints= []
        self.laserRadius= 300
        self.laserImpactPoints= {}
        self.laserImpactDistances= {}
        self.carCenter = b2Vec2(0,0)
        self.carOrientation = b2Vec2(0,0)

    def laser(self):
        self.laserPoints = []
        self.laserImpactPoints = []
        self.laserImpactDistances = []

        for laserDegree in self.laserDegrees:
            degrees = laserDegree + self.carOrientation - (b2_pi / 2.0) + b2_pi

            # print("laserDegree", laserDegree, degrees, self.carOrientation)

            laserPointX = self.carCenter.x + self.laserRadius * math.cos(degrees)
            laserPointY = self.carCenter.y + self.laserRadius * math.sin(degrees)

            laserPoint = b2Vec2(laserPointX,laserPointY)

            self.laserPoints.append(laserPoint)

            callback = RayCastMultipleCallback()

            self.world.RayCast(callback, self.carCenter, laserPoint)

            callbackPoints = [b2Vec2(self.laserRadius,self.laserRadius)]
            callbackFixtures = []

            if callback.hit:
                callbackPoints = callback.points
                callbackFixtures = callback.fixtures
            
            self.calcLaserDistances(laserDegree, callbackPoints, callbackFixtures)

    def calcLaserDistances(self, laserDegree, points, fixtures):
        indexWallFix = -1
        # print('calcLaserDistances', fixtures)
        for index in range(len(fixtures)):
            if fixtures[index].userData == "wall":
                indexWallFix = index

        if indexWallFix != -1:
            point = points[indexWallFix]
            distance = math.sqrt(math.pow((point.x - self.carCenter.x), 2) + math.pow((point.y - self.carCenter.y), 2))
            self.laserImpactDistances.append(distance)
            self.laserImpactPoints.append(point)
        else:
            self.laserImpactDistances.append(self.laserRadius)

    def update(self, carCenter, carOrientation):
        self.carCenter = carCenter
        self.carOrientation = carOrientation
        self.laser()
        # print("laserImpactDistances", self.laserImpactDistances)