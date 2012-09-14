# Copyright (c) PyBullet Team
# See LICENSE for details.

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_DEPTH_TEST, glEnable, glTranslate,
    glColor, glTranslate)
from OpenGL.GLU import (
    gluPerspective, gluNewQuadric, gluSphere)

from bullet.bullet import (
    Vector3, Transform, SphereShape, PairCachingGhostObject,
    KinematicCharacterController, AxisSweep3, DiscreteDynamicsWorld)

from demolib import Ground, simulate

class Ghost(object):
    def __init__(self):
        radius = 1
        position = Vector3(1, 2, 3)
        color = (0, 255, 0)

        self.radius = radius

        self.body = PairCachingGhostObject()
        self.body.setRestitution(0.9)
        self.body.setCollisionFlags(PairCachingGhostObject.CF_CHARACTER_OBJECT)

        self.shape = SphereShape(self.radius)
        self.body.setCollisionShape(self.shape)

        transform = Transform()
        transform.setIdentity()
        transform.setOrigin(position)
        self.body.setWorldTransform(transform)

        self.quad = gluNewQuadric()
        self.color = color


    def render(self):
        o = self.body.getWorldTransform().getOrigin()
        glColor(*self.color)
        glTranslate(o.x, o.y, o.z)
        gluSphere(self.quad, self.radius, 25, 25)



def main():
    pygame.init()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    broadphase = AxisSweep3(Vector3(-100, -100, -100), Vector3(100, 100, 100))
    broadphase.getOverlappingPairCache().setInternalGhostPairCallback()
    dynamicsWorld = DiscreteDynamicsWorld(broadphase=broadphase)
    dynamicsWorld.setGravity(Vector3(0, 0, 0))

    ground = Ground()
    dynamicsWorld.addRigidBody(ground.body)

    ghost = Ghost()
    dynamicsWorld.addCollisionObject(ghost.body)

    character = KinematicCharacterController(ghost.body, 0.5, 1)
    character.setWalkDirection(Vector3(0, 0, 0))
    dynamicsWorld.addAction(character)

    simulate(dynamicsWorld, [ground, ghost])


if __name__ == '__main__':
    main()
