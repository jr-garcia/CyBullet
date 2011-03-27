# Copyright (c) PyBullet Team
# See LICENSE for details.

import pygame.display

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_TRIANGLE_STRIP,
    glEnable, glPushMatrix, glPopMatrix, glClear, glColor,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluPerspective, gluNewQuadric, gluSphere

from numpy import array

from bullet.bullet import (
    Vector3, Transform, DefaultMotionState,
    IndexedMesh, TriangleIndexVertexArray, BvhTriangleMeshShape,
    SphereShape,
    RigidBody, DiscreteDynamicsWorld)

def main():
    pygame.init()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)



    world = DiscreteDynamicsWorld()

    groundIndices = array(range(3), 'i')
    groundVertices = array([
            0,  0, 0,
            10, 0, 0,
            10, 0, 10], 'f')
    groundMesh = IndexedMesh()
    groundMesh.setIndices(3, 0, groundIndices)
    groundMesh.setVertices(9, 0, groundVertices)
    groundStuff = TriangleIndexVertexArray()
    groundStuff.addIndexedMesh(groundMesh)
    groundShape = BvhTriangleMeshShape(groundStuff)
    groundShape.buildOptimizedBvh()

    groundTransform = Transform()
    groundTransform.setIdentity()
    groundTransform.setOrigin(Vector3(-1, -5, -1))
    groundMotion = DefaultMotionState()
    groundMotion.setWorldTransform(groundTransform)

    ground = RigidBody(shape=groundShape)
    world.addRigidBody(ground)

    ballShape = SphereShape(3)
    ball = RigidBody(shape=ballShape, mass=1.0)
    world.addRigidBody(ball)

    for i in range(100):
        world.stepSimulation(1.0 / 60.0, 10)

        o = ball.getWorldTransform().getOrigin()
        print 'ball pos = %0.6f,%0.6f,%0.6f' % (o.x, o.y, o.z)
        o = ground.getWorldTransform().getOrigin()
        print 'ground pos = %0.6f,%0.6f,%.06f' % (o.x, o.y, o.z)


if __name__ == '__main__':
    main()

