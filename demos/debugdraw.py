
import time

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_LINES,
    glEnable, glPushMatrix, glPopMatrix, glClear, glColor,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluPerspective

from bullet.bullet import (Vector3, DiscreteDynamicsWorld)

from demolib import Ball


class DebugDraw:
    def reset(self):
        self.lines = []
        self.contacts = []


    def drawLine(self, *args):
        self.lines.append(args)


    def drawContactPoint(self, *args):
        self.contacts.append(args)



def main():
    pygame.init()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    objects = []
    dynamicsWorld = DiscreteDynamicsWorld()
    debug = DebugDraw()
    dynamicsWorld.setDebugDrawer(debug)

    b1 = Ball(Vector3(-30, 0, 0), (255, 0, 0))
    b1.body.applyCentralImpulse(Vector3(30, 40, 0))
    objects.append(b1)

    b2 = Ball(Vector3(+30, 0, 0), (0, 255, 0))
    b2.body.applyCentralImpulse(Vector3(-30, 40, 0))
    objects.append(b2)

    for o in objects:
        dynamicsWorld.addRigidBody(o.body)

    timeStep = 1.0 / 60.0
    while True:
        timeStep - time.time() % timeStep
        time.sleep(timeStep)
        dynamicsWorld.stepSimulation(timeStep, 100, timeStep)

        glPushMatrix()
        for o in objects:
            glPushMatrix()
            o.render()
            glPopMatrix()
        glPopMatrix()

        debug.reset()
        dynamicsWorld.debugDrawWorld()
        glBegin(GL_LINES)
        for line in debug.lines:
            glColor(*line[6:])
            glVertex(*line[:3])
            glVertex(*line[3:6])
        if debug.contacts:
            print 'Contact!', debug.contacts
        glEnd()

        pygame.display.flip()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


if __name__ == '__main__':
    main()
