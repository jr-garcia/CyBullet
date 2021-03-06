
import time

import pygame.display
import pygame.event

from OpenGL.GL import (
    GL_TRIANGLE_STRIP, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    glPushMatrix, glPopMatrix, glColor, glClear,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluNewQuadric, gluSphere

from bullet.bullet import (
    Vector3, Transform, BoxShape, SphereShape, DefaultMotionState, RigidBody)


class Ground:
    def __init__(self):
        self.boxHalfExtents = Vector3(20, 2, 20)
        groundShape = BoxShape(self.boxHalfExtents)
        groundTransform = Transform()
        groundTransform.setIdentity()
        groundTransform.setOrigin(Vector3(0, -4, 0))
        groundMotion = DefaultMotionState()
        groundMotion.setWorldTransform(groundTransform)
        self.body = RigidBody(groundMotion, groundShape)
        self.body.setRestitution(0.5)
        self.motion = groundMotion


    def render(self):
        x, y, z = (
            self.boxHalfExtents.x, self.boxHalfExtents.y, self.boxHalfExtents.z)
        o = self.motion.getWorldTransform().getOrigin()
        glColor(0, 0, 255)
        glTranslate(o.x, o.y, o.z)
        glBegin(GL_TRIANGLE_STRIP)
        glVertex(-x, y, -z)
        glVertex(x, y, -z)
        glVertex(-x, y, z)
        glVertex(x, y, z)
        glEnd()



class Ball:
    def __init__(self, position, color, radius=2):
        self.radius = radius
        ballShape = SphereShape(self.radius)
        ballTransform = Transform()
        ballTransform.setIdentity()
        ballTransform.setOrigin(position)
        ballMotion = DefaultMotionState()
        ballMotion.setWorldTransform(ballTransform)
        self.body = RigidBody(ballMotion, ballShape, 2.0)
        self.body.setRestitution(0.9)
        self.motion = ballMotion
        self.quad = gluNewQuadric()
        self.color = color


    def render(self):
        o = self.motion.getWorldTransform().getOrigin()
        glColor(*self.color)
        glTranslate(o.x, o.y, o.z)
        gluSphere(self.quad, self.radius, 25, 25)


class Controller(object):
    """
    Input handler which just knows how to exit the main loop.
    """
    def key(self, event):
        if event.key == pygame.locals.K_ESCAPE or event.key == pygame.locals.K_q:
            raise SystemExit(0)



def step(world):
    timeStep = fixedTimeStep = 1.0 / 60.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay)


def render(objects):
    glPushMatrix()
    for o in objects:
        glPushMatrix()
        o.render()
        glPopMatrix()
    glPopMatrix()

    pygame.display.flip()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


def simulate(world, objects, controller=None):
    if controller is None:
        controller = Controller()
    while True:
        step(world)
        render(objects)
        events = pygame.event.get()
        for e in events:
            if e.type in (pygame.locals.KEYUP, pygame.locals.KEYDOWN):
                controller.key(e)
