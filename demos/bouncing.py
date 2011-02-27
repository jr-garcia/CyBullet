
import time

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST,
    glEnable, glPushMatrix, glPopMatrix, glClear, glTranslate)
from OpenGL.GLU import gluPerspective

from bullet.bullet import Vector3, DiscreteDynamicsWorld

from demolib import Ground, Ball

def main():
    pygame.init()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    objects = []
    dynamicsWorld = DiscreteDynamicsWorld()

    objects.append(Ground())
    objects.append(Ball(Vector3(1, 10, 0), (255, 0, 0)))
    objects.append(Ball(Vector3(0, 20, 1), (0, 255, 0)))
    objects.append(Ball(Vector3(0, 30, 1), (255, 255, 0)))
    objects.append(Ball(Vector3(0, 40, 1), (0, 255, 255, 0)))

    for o in objects:
        dynamicsWorld.addRigidBody(o.body)

    while True:
        time.sleep(1.0 / 60.0)
        dynamicsWorld.stepSimulation(1.0 / 60.0, 60, 1.0 / 60.0)

        glPushMatrix()
        for o in objects:
            glPushMatrix()
            o.render()
            glPopMatrix()
        glPopMatrix()

        pygame.display.flip()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


if __name__ == '__main__':
    main()

