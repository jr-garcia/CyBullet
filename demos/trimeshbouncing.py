
import time

from numpy import array

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_TRIANGLES,
    glEnable, glPushMatrix, glPopMatrix, glClear, glColor,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluPerspective, gluNewQuadric, gluSphere

from bullet.bullet import (
    Vector3, Transform,
    BoxShape, SphereShape, 
    IndexedMesh, TriangleIndexVertexArray, BvhTriangleMeshShape,
    DefaultMotionState,
    RigidBody,
    DiscreteDynamicsWorld)


class Ground:
    def __init__(self):
        self.groundIndices = array(range(9), 'i')
        self.groundVertices = array([
                0,  0, 0,
                10, 0, 0,
                10, 0, 10], 'f')
        groundMesh = IndexedMesh()
        groundMesh.setIndices(9, 0, self.groundIndices)
        groundMesh.setVertices(3, 0, self.groundVertices)
        groundStuff = TriangleIndexVertexArray()
        groundStuff.addIndexedMesh(groundMesh)
        groundShape = BvhTriangleMeshShape(groundStuff)
        groundShape.buildOptimizedBvh()

        groundTransform = Transform()
        groundTransform.setIdentity()
        groundTransform.setOrigin(Vector3(-1, -5, -1))
        groundMotion = DefaultMotionState()
        groundMotion.setWorldTransform(groundTransform)

        self.body = RigidBody(groundMotion, groundShape)
        self.motion = groundMotion


    def render(self):
        o = self.motion.getWorldTransform().getOrigin()
        glColor(0, 0, 255)
        glTranslate(o.x, o.y, o.z)
        glBegin(GL_TRIANGLES)
        for i in range(0, len(self.groundIndices), 3):
            x, y, z = self.groundIndices[i:i + 3]
            x = self.groundVertices[x]
            y = self.groundVertices[y]
            z = self.groundVertices[z]
            glVertex(x, y, z)
        glEnd()



class Ball:
    def __init__(self, position, color):
        self.radius = 2
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

