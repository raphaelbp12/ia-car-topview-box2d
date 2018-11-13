import math
from Box2D import (b2BodyDef, b2CircleShape, b2Color, b2EdgeShape,
                   b2FixtureDef, b2PolygonShape, b2RayCastCallback, b2Vec2,
                   b2_dynamicBody, b2_pi)

class Goal:
    
    def __init__(self, world, userData, x, y):
        self.fixDef = b2FixtureDef(
                shape=b2PolygonShape(box=(5, 5)),
                isSensor=True,
                density=1.0,
                friction=1.0,
                userData = userData,
            )

        self.body = world.CreateStaticBody(	
            position=(x, y),
            fixtures=self.fixDef	
        )
        
        self.fixture = self.body.CreateFixture(self.fixDef)

        self.carDistance = 0
        self.carAngle = 0