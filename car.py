from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Vec2, b2RevoluteJointDef)

class Car:
    def __init__(self, world, x, y, scale):
        self.world = world
        self.fixDef = b2FixtureDef(
                shape=b2PolygonShape(box=(0.1*scale, 0.3*scale)),
                density=30.0,
                friction=4.0/scale*3,
                userData = "car",
            )
        self.body = self.world.CreateDynamicBody(
            position=(x, y),
            fixtures=self.fixDef
        )

        self.carPosition = self.body.position

        self.xx = self.carPosition.x
        self.yy = self.carPosition.y
        
        self.maxSteeringAngle = 1
        self.steeringAngle = 0
        self.STEER_SPEED = 1.0*scale
        self.sf = False
        self.sb = False
        self.ENGINE_SPEED = 10*scale*scale

        self.p1r= b2Vec2(0,0)
        self.p2r= b2Vec2(0,0)
        self.p3r= b2Vec2(0,0)
        self.p1l= b2Vec2(0,0)
        self.p2l= b2Vec2(0,0)
        self.p3l= b2Vec2(0,0)

        self.frontRightWheel = self.createWheel(self.world, self.xx+(0.1*scale), self.yy-(0.2*scale), scale, "wheel front")
        self.frontLeftWheel = self.createWheel(self.world, self.xx-(0.1*scale), self.yy-(0.2*scale), scale, "wheel front")
        self.rearRightWheel = self.createWheel(self.world, self.xx+(0.1*scale), self.yy+(0.2*scale), scale, "wheel rear")
        self.rearLeftWheel = self.createWheel(self.world, self.xx-(0.1*scale), self.yy+(0.2*scale), scale, "wheel rear")

        self.jointFrontRight = self.createRevJoint(self.world, self.body, self.frontRightWheel, scale, False)
        self.jointFrontLeft = self.createRevJoint(self.world, self.body, self.frontLeftWheel, scale, False)
        self.jointRearRight = self.createRevJoint(self.world, self.body, self.rearRightWheel, scale, True)
        self.jointRearLeft = self.createRevJoint(self.world, self.body, self.rearLeftWheel, scale, True)

    def createWheel(self, world, x, y, scale, name):
        fixDef = b2FixtureDef(
                shape=b2PolygonShape(box=(0.04*scale, 0.08*scale)),
                density=30.0,
                friction=40.0/scale*3,
                userData = name,
                isSensor=True
            )
        wheel = world.CreateDynamicBody(
            position=(x, y),
            fixtures=fixDef
        )

        return wheel

    def getBodies(self):
        bodyList = [self.body, self.frontRightWheel, self.frontLeftWheel, self.rearRightWheel, self.rearLeftWheel]
        return bodyList

    def createRevJoint(self, world, body1, wheel, scale, limited):
        revoluteJointDef = b2RevoluteJointDef(
            bodyA=body1,
            bodyB=wheel,
            localAnchorA=wheel.worldCenter,
            localAnchorB=body1.worldCenter,
            enableLimit=True,
            maxMotorTorque=200*scale,
            motorSpeed=0,
        )
        if limited:
            revoluteJointDef.enableLimit = True
            revoluteJointDef.lowerAngle=0
            revoluteJointDef.upperAngle=0

        revoluteJoint = world.CreateJoint(revoluteJointDef)
        return revoluteJoint

    def setVelocitiesAndDirection(self, backwardOrForward, leftOrRight):
        self.steeringAngle = leftOrRight * self.maxSteeringAngle
        if (backwardOrForward and backwardOrForward != 0):
            self.steer(backwardOrForward * -1)

    def steer(self, backwardOrForward):
        self.frontRightWheel.ApplyForce(b2Vec2(self.p3r.x * backwardOrForward,self.p3r.y * backwardOrForward),self.frontRightWheel.GetWorldPoint(b2Vec2(0,0)), True)
        self.frontLeftWheel.ApplyForce(b2Vec2(self.p3l.x * backwardOrForward,self.p3l.y * backwardOrForward),self.frontLeftWheel.GetWorldPoint(b2Vec2(0,0)), True)
    
    def cancelVel(self, wheel):
        aaaa=b2Vec2(0,0)
        bbbb=b2Vec2(0,0)
        newlocal=b2Vec2(0,0)
        newworld=b2Vec2(0,0)
    
        aaaa=wheel.GetLinearVelocityFromLocalPoint(b2Vec2(0,0))
        bbbb=wheel.GetLocalVector(aaaa)
        newlocal.x = -bbbb.x
        newlocal.y = bbbb.y
        newworld = wheel.GetWorldVector(newlocal)
        wheel.linearVelocity = (newworld)


    def move(self):
        self.mspeed = self.steeringAngle - self.jointFrontLeft.angle
        self.jointFrontLeft.motorSpeed = (self.mspeed * self.STEER_SPEED)
        self.mspeed = self.steeringAngle - self.jointFrontRight.angle
        self.jointFrontRight.motorSpeed = (self.mspeed * self.STEER_SPEED)
        # console.log('move', 'mspeed', self.mspeed)

    def update(self, backwardOrForward, leftOrRight):

            self.setVelocitiesAndDirection(backwardOrForward, leftOrRight)
            self.carPosition = self.body.worldCenter

            self.move()
            self.cancelVel(self.frontRightWheel)
            self.cancelVel(self.frontLeftWheel)
            self.cancelVel(self.rearRightWheel)
            self.cancelVel(self.rearLeftWheel)
        
            self.p1r = self.frontRightWheel.worldCenter
            self.p2r = self.frontRightWheel.GetWorldPoint(b2Vec2(0,1))
            self.p3r.x = (self.p2r.x - self.p1r.x)*self.ENGINE_SPEED
            self.p3r.y = (self.p2r.y - self.p1r.y)*self.ENGINE_SPEED
                
            self.p1l = self.frontLeftWheel.worldCenter
            self.p2l = self.frontLeftWheel.GetWorldPoint(b2Vec2(0,1))
            self.p3l.x = (self.p2l.x - self.p1l.x)*self.ENGINE_SPEED
            self.p3l.y = (self.p2l.y - self.p1l.y)*self.ENGINE_SPEED
    
            # self.sensors.update(self.carPosition, self.body.angle)

            # console.log('linear velocity', self.body.GetLinearVelocity())