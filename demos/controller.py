# Copyright (c) PyBullet Team
# See LICENSE for details.

from __future__ import division, unicode_literals, absolute_import

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_POLYGON, GL_MODELVIEW, GL_PROJECTION,
    GL_DEPTH_TEST, glEnable, glTranslate,
    glColor, glVertex3f, glBegin, glEnd, glScale,
    glMatrixMode, glLoadIdentity)
from OpenGL.GLU import (
    gluLookAt, gluPerspective, gluNewQuadric, gluSphere)

from bullet.bullet import (
    Vector3, Transform, SphereShape, BoxShape, PairCachingGhostObject,
    KinematicCharacterController, AxisSweep3, DiscreteDynamicsWorld,
    RigidBody)

from demolib import Controller, Ground, simulate

class Ghost(object):
    def __init__(self):
        radius = 1
        position = Vector3(1, 2, 3)
        color = (0, 255, 0)

        self.radius = radius

        self.body = PairCachingGhostObject()
        self.body.setRestitution(0.9)
        # self.body.setCollisionFlags(PairCachingGhostObject.CF_CHARACTER_OBJECT)

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



class Controller(Controller):
    def __init__(self, kinematic):
        self.kinematic = kinematic
        self.direction = Vector3(0, 0, 0)


    def key(self, event):
        super(Controller, self).key(event)

        if event.key == pygame.locals.K_UP:
            change = Vector3(0, 0, -0.1)
        elif event.key == pygame.locals.K_DOWN:
            change = Vector3(0, 0, 0.1)
        elif event.key == pygame.locals.K_LEFT:
            change = Vector3(-0.1, 0, 0)
        elif event.key == pygame.locals.K_RIGHT:
            change = Vector3(0.1, 0, 0)
        else:
            change = Vector3(0, 0, 0)

        if event.type == pygame.locals.KEYDOWN:
            self.direction += change
        elif event.type == pygame.locals.KEYUP:
            self.direction -= change
        else:
            return

        self.kinematic.setWalkDirection(self.direction)


MAP = [
    "x.xxxxxxxxxxxxxxx",
    "x.x.....x..x....x",
    "x.x.xxx.xx...xxxx",
    "x.....x..x.x....x",
    "xxx.x.xxxx.xxxx.x",
    "x.x.x.xxx..x..x.x",
    "x...x.....xxx...x",
    "x.xxxxxxxxx.x..xx",
    "x...x.........xxx",
    "xxxxx.xxxxxxxxxxx",
    ]


class Wall(object):
    def __init__(self, position, color):
        self.position = position
        self.color = color
        self.shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        self.body = RigidBody(None, self.shape, mass=0.0)
        transform = Transform()
        transform.setIdentity()
        transform.setOrigin(self.position)
        self.body.setWorldTransform(transform)


    def render(self):
        glTranslate(self.position.x, self.position.y, self.position.z)
        glColor(0.75, 0.0, 0.0)
        glScale(2.0, 1.0, 2.0)

        glBegin(GL_POLYGON)
        glVertex3f(  0.5, -0.5, 0.5 )
        glVertex3f(  0.5,  0.5, 0.5 )
        glVertex3f( -0.5,  0.5, 0.5 )
        glVertex3f( -0.5, -0.5, 0.5 )
        glEnd()

        glBegin(GL_POLYGON)
        glVertex3f( 0.5, -0.5, -0.5 )
        glVertex3f( 0.5,  0.5, -0.5 )
        glVertex3f( 0.5,  0.5,  0.5 )
        glVertex3f( 0.5, -0.5,  0.5 )
        glEnd()

        glBegin(GL_POLYGON)
        glVertex3f( -0.5, -0.5,  0.5)
        glVertex3f( -0.5,  0.5,  0.5)
        glVertex3f( -0.5,  0.5, -0.5)
        glVertex3f( -0.5, -0.5, -0.5)
        glEnd()

        glBegin(GL_POLYGON);
        glVertex3f(  0.5,  0.5,  0.5 )
        glVertex3f(  0.5,  0.5, -0.5 )
        glVertex3f( -0.5,  0.5, -0.5 )
        glVertex3f( -0.5,  0.5,  0.5 )
        glEnd()

        glBegin(GL_POLYGON)
        glVertex3f(  0.5, -0.5, -0.5 )
        glVertex3f(  0.5, -0.5,  0.5 )
        glVertex3f( -0.5, -0.5,  0.5 )
        glVertex3f( -0.5, -0.5, -0.5 )
        glEnd()



class Maze(object):
    def __init__(self, map):
        self.map = map
        self.walls = []


    def addToWorld(self, world):
        self.width = len(self.map[0])
        self.height = len(self.map)
        for y in range(self.height):
            for x in range(self.width):
                if self.map[y][x] == "x":
                    self._putWall(world, x - self.width / 2, y - self.height / 2)


    def _putWall(self, world, x, y):
        wall = Wall(Vector3(x * 2, -1, y * 2), (128, 0, 0))
        self.walls.append(wall)
        world.addRigidBody(wall.body)


WIDTH = 640
HEIGHT = 480

def main():
    pygame.init()
    pygame.display.set_mode(
        (WIDTH, HEIGHT), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)
    glEnable(GL_DEPTH_TEST)

    broadphase = AxisSweep3(Vector3(-100, -100, -100), Vector3(100, 100, 100))
    broadphase.getOverlappingPairCache().setInternalGhostPairCallback()
    dynamicsWorld = DiscreteDynamicsWorld(broadphase=broadphase)
    dynamicsWorld.setGravity(Vector3(0, -9.8, 0))

    ground = Ground()
    dynamicsWorld.addRigidBody(ground.body)

    maze = Maze(MAP)
    maze.addToWorld(dynamicsWorld)

    ghost = Ghost()
    dynamicsWorld.addCollisionObject(ghost.body)

    character = KinematicCharacterController(ghost.body, 0.5, 1)
    character.setWalkDirection(Vector3(0, 0, 0))
    dynamicsWorld.addAction(character)

    pos = ghost.body.getWorldTransform().getOrigin()

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, WIDTH / HEIGHT, 0.5, 1000.0)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(pos.x, pos.y + 25, pos.z + 15,
              pos.x, pos.y, pos.z,
              0, 1, 0)

    simulate(dynamicsWorld, [ground, ghost] + maze.walls, Controller(character))


if __name__ == '__main__':
    main()
