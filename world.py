import Box2D  # The main library
import pygame
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)
from Box2D import (b2BodyDef, b2CircleShape, b2Color, b2EdgeShape,
                   b2FixtureDef, b2PolygonShape, b2RayCastCallback, b2Vec2,
                   b2_dynamicBody, b2_pi)

from top_down import (TDCar, TDTire)
from goal import (Goal)
from contactListener import (ContactListener)

class World:
    def __init__(self, worldDrawScale, TIME_STEP):
        self.world = world(gravity=(0, 0), doSleep=True)

        self.ticks = 0
        self.worldDrawScale = worldDrawScale

        self.contactListener = ContactListener()
        self.world.contactListener = self.contactListener

        self.TIME_STEP = TIME_STEP

        self.goals = [Goal(self.world, "goal", 300, 300)]

        self.goalsBodies = []
        for goal in self.goals:
            self.goalsBodies.append(goal.body)

        self.horizontal_ground_body_fixDef = b2FixtureDef(
                shape=b2PolygonShape(box=(1270, 5)),
                userData = "wall",
            )

        self.vertical_ground_body_fixDef = b2FixtureDef(
                        shape=b2PolygonShape(box=(5, 710)),
                        userData = "wall",
                    )
        # And a static body to hold the ground shape
        self.h1_ground_body = self.world.CreateStaticBody(
            position=(5, 0),
            fixtures=self.horizontal_ground_body_fixDef,
        )
        self.h2_ground_body = self.world.CreateStaticBody(
            position=(5, 715),
            fixtures=self.horizontal_ground_body_fixDef,
        )
        self.v1_ground_body = self.world.CreateStaticBody(
            position=(0, 5),
            fixtures=self.vertical_ground_body_fixDef,
        )
        self.v2_ground_body = self.world.CreateStaticBody(
            position=(1275, 5),
            fixtures=self.vertical_ground_body_fixDef,
        )

        # car = Car(world, 15, 15, 15)
        self.car = TDCar(self.world)

        
        self.totalRunnedDistance = 0
        self.numberContactWall = 0
        self.ticksOnCrashToWall = []
        self.ticksOnGetObjective = []

        self.laserPoints = []
        self.laserImpactPoints = []

        self.rightOrLeft = 0
        self.backwardOrForward = 0

        self.colors = {
            staticBody: (0, 0, 255, 255),
            dynamicBody: (127, 127, 127, 255),
        }

    def verifyContactToGoal(self, contact):
        contacted = self.verifyContact(contact, 'car', 'goal')
        if (not contacted):
            contacted = self.verifyContact(contact, 'wheel', 'goal')

        # print('verifyContactToGoal', contacted)
        if(contacted):
            remainingGoals = []
            for goal in self.goals:
                if(contact.fixtureA.userData == goal.fixture.userData or contact.fixtureB.userData == goal.fixture.userData):
                    self.ticksOnGetObjective.append(self.ticks)
                    self.world.DestroyBody(goal.body)
                    # print("body destroyed", goal, self.contactListener.hasContact)
                else:
                    remainingGoals.append(goal)
            self.goals = remainingGoals

    def verifyIfIsPartOfUserData(Self, fixPartialName, fixture):
        countChar = len(fixPartialName)
        if (type(fixture.userData) is str and len(fixture.userData) >= countChar and fixture.userData[:countChar] == fixPartialName):
            return True
        else:
            return False


    def verifyContact(self, contact, fixExactName, fixPartialName):

        # print('contact', contact)
        # print('fixA', contact.fixtureA.userData, 'fixB', contact.fixtureB.userData)
        if ((contact.fixtureA.userData and self.verifyIfIsPartOfUserData(fixPartialName, contact.fixtureA) and (contact.fixtureB.userData == fixExactName)) or ((contact.fixtureA.userData == fixExactName) and  contact.fixtureB.userData and self.verifyIfIsPartOfUserData(fixPartialName, contact.fixtureB))):
            return True
        return False

    def update(self, screen, backwardOrForward, rightOrLeft):
        self.ticks = self.ticks + 1
        if self.contactListener.hasContact:
            contact = self.contactListener.getBeginContact()
            self.verifyContactToGoal(contact)
        self.backwardOrForward = backwardOrForward
        self.rightOrLeft = rightOrLeft

        self.draw(screen)
        self.car.update(backwardOrForward, rightOrLeft)
        self.laserPoints = self.car.sensors.laserPoints
        self.laserImpactPoints = self.car.sensors.laserImpactPoints
        self.world.Step(self.TIME_STEP, 10, 10)


    def draw(self, screen):
        for body in ([self.h1_ground_body, self.h2_ground_body, self.v1_ground_body, self.v2_ground_body] + self.car.getBodies() + self.goalsBodies):  # or: world.bodies
            # The body gives us the position and angle of its shapes
            for fixture in body.fixtures:
                # The fixture holds information like density and friction,
                # and also the shape.
                shape = fixture.shape

                # Naively assume that this is a polygon shape. (not good normally!)
                # We take the body's transform and multiply it with each
                # vertex, and then convert from meters to pixels with the scale
                # factor.
                vertices = [(body.transform * v) * self.worldDrawScale for v in shape.vertices]

                # But wait! It's upside-down! Pygame and Box2D orient their
                # axes in different ways. Box2D is just like how you learned
                # in high school, with positive x and y directions going
                # right and up. Pygame, on the other hand, increases in the
                # right and downward directions. This means we must flip
                # the y components.
                # vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]

                color = self.colors[body.type]

                if (fixture.userData == "wheel front"):
                    color = (255, 0, 0, 255)
                if (fixture.userData == "wheel rear"):
                    color = (0, 0, 255, 255)
                if (type(fixture.userData) is str and len(fixture.userData) >= 4 and fixture.userData[:4] == "goal"):
                    color = (0, 150, 150, 255)

                pygame.draw.polygon(screen, color, vertices)

        for point in self.laserPoints:
            color = (0, 255, 0, 255)
            carPosScaled = b2Vec2(self.car.body.worldCenter.x * self.worldDrawScale, self.car.body.worldCenter.y * self.worldDrawScale)
            pointScaled = b2Vec2(point.x * self.worldDrawScale, point.y * self.worldDrawScale)
            pygame.draw.line(screen, color, carPosScaled, pointScaled)

        for point in self.laserImpactPoints:
            color = (0, 255, 0, 255)
            pointScaled = b2Vec2(point.x * self.worldDrawScale, point.y * self.worldDrawScale)
            # print("laserImpactPoints point", point, pointScaled)
            pygame.draw.circle(screen, color, (int(pointScaled.x), int(pointScaled.y)), 2, 0)