from Box2D import (b2ContactListener, b2Contact)

class ContactListener(b2ContactListener):
    def __init__(self):
        self.beginContact = b2Contact
        self.hasContact = False
        b2ContactListener.__init__(self)
    def BeginContact(self, contact):
        self.beginContact = contact
        self.hasContact = True

        fixA = contact.fixtureA
        fixB = contact.fixtureB
        # print("contact", fixA, fixB)

        pass
    def EndContact(self, contact):
        pass
    def PreSolve(self, contact, oldManifold):
        pass
    def PostSolve(self, contact, impulse):
        pass

    def getBeginContact(self):
        self.hasContact = False
        return self.beginContact