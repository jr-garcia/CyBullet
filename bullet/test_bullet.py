
from unittest import TestCase

import numpy

from bullet import (
    Vector3,
    CollisionShape, EmptyShape, BoxShape, Box2dShape, BvhTriangleMeshShape,
    ActionInterface, KinematicCharacterController,
    DefaultMotionState,
    CollisionObject, RigidBody,
    CollisionWorld, DiscreteDynamicsWorld)


class VectorTests(TestCase):
    def test_initialization(self):
        v = Vector3(1, 2, 3)
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)


class DefaultMotionStateTests(TestCase):
    def test_getWorldTransform(self):
        state = DefaultMotionState()
        del state



class Box2dShapeTests(TestCase):
    def test_instantiate(self):
        shape = Box2dShape(Vector3(3, 5, 7))
        self.assertTrue(isinstance(shape, CollisionShape))



class BoxShapeTests(TestCase):
    def test_instantiate(self):
        shape = BoxShape(Vector3(2, 4, 6))
        self.assertTrue(isinstance(shape, CollisionShape))



class BvhTriangleMeshShapeTests(TestCase):
    def test_incorrectInitializer(self):
        goodTriangleType = 'int32'
        goodTriangleData = [1]
        badTriangleType = 'int8'
        badTriangleData = [[1, 2], [3, 4]]

        goodVertexType = 'float32'
        goodVertexData = [1]
        badVertexType = 'float64'
        badVertexData = [[1, 2], [3, 4]]

        goodTriangles = numpy.array(goodTriangleData, goodTriangleType)
        goodVertices = numpy.array(goodVertexData, goodVertexType)

        badArgs = [
            (goodTriangles, None),
            (None, goodVertices),
            (goodTriangles, numpy.array(badVertexData, goodVertexType)),
            (goodTriangles, numpy.array(goodVertexData, badVertexType)),
            (numpy.array(badTriangleData, goodTriangleType), goodVertices),
            (numpy.array(goodTriangleData, badTriangleType), goodVertices),
            ]

        for (triangles, vertices) in badArgs:
            try:
                BvhTriangleMeshShape(triangles, vertices)
            except (TypeError, ValueError):
                pass
            else:
                self.fail(
                    "BvhTriangleMeshShape accepted (%r, %r)" % (
                        triangles, vertices))


    def test_initialized(self):
        BvhTriangleMeshShape(
            numpy.array([0, 1, 2], 'int32'),
            numpy.array([1, 2, 3], 'float32'))



class KinematicCharacterControllerTests(TestCase):
    def test_instantiate(self):
        shape = BoxShape(Vector3(1, 2, 3))
        controller = KinematicCharacterController(shape, 2.5, 1)
        self.assertTrue(isinstance(controller, ActionInterface))


    def test_setWalkDirection(self):
        shape = BoxShape(Vector3(1, 2, 3))
        controller = KinematicCharacterController(shape, 2.5, 1)
        controller.setWalkDirection(Vector3(1, 0, 0))


    def test_setVelocityForTimeInterval(self):
        shape = BoxShape(Vector3(1, 2, 3))
        controller = KinematicCharacterController(shape, 2.5, 1)
        controller.setVelocityForTimeInterval(Vector3(12.0, 0, 0), 6.0)



class CollisionWorldTests(TestCase):
    def test_empty(self):
        world = CollisionWorld()
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_addCollisionObject(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = EmptyShape()
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 1)


    def test_removeCollisionObject(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = EmptyShape()
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        world.removeCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_addBoxShape(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = BoxShape(Vector3(2, 3, 4))
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)



class DiscreteDynamicsWorldTests(TestCase):
    def test_empty(self):
        world = DiscreteDynamicsWorld()
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_gravity(self):
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(3, 2, 1))
        gravity = world.getGravity()
        self.assertEqual(gravity.x, 3)
        self.assertEqual(gravity.y, 2)
        self.assertEqual(gravity.z, 1)


    def test_addRigidBody(self):
        world = DiscreteDynamicsWorld()
        body = RigidBody()
        world.addRigidBody(body)


    def test_stepSimulation(self):
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(1, 2, 3))
        obj = RigidBody()
        world.addRigidBody(obj)
        expectedSteps = 64
        numSteps = world.stepSimulation(1.0, expectedSteps, 1.0 / expectedSteps)
        self.assertEqual(numSteps, expectedSteps)
        position = obj.getMotionState().getWorldTransform().getOrigin()

        # Unfortunately, there is some error (as compared to physical reality)
        # in Bullet's results.  My fault?  Bullet's fault?  I'm not sure.
        self.assertEqual(position.x, 0.5 + 0.5 / expectedSteps)
        self.assertEqual(position.y, 1.0 + 1.0 / expectedSteps)
        self.assertEqual(position.z, 1.5 + 1.5 / expectedSteps)
