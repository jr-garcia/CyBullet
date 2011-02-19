# Copyright (c) 2010 Jean-Paul Calderone
# See LICENSE for details.

from unittest import TestCase

import numpy

from bullet import (
    Vector3, Transform,
    CollisionShape, BoxShape, Box2dShape, SphereShape, CapsuleShape,
    BvhTriangleMeshShape,
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


    def test_repr(self):
        v = Vector3(3, 5, 7)
        self.assertEqual(repr(v), '<Vector x=3.0 y=5.0 z=7.0>')



class TransformTests(TestCase):
    def test_origin(self):
        transform = Transform()
        transform.setOrigin(Vector3(1, 2, 3))
        origin = transform.getOrigin()
        self.assertEqual(origin.x, 1)
        self.assertEqual(origin.y, 2)
        self.assertEqual(origin.z, 3)


    def test_setIdentity(self):
        transform = Transform()
        transform.setOrigin(Vector3(2, 3, 4))
        transform.setIdentity()
        origin = transform.getOrigin()
        self.assertEqual(origin.x, 0)
        self.assertEqual(origin.y, 0)
        self.assertEqual(origin.z, 0)


class DefaultMotionStateTests(TestCase):
    def test_worldTransform(self):
        xform = Transform()
        xform.setOrigin(Vector3(3, 5, 7))
        state = DefaultMotionState()
        state.setWorldTransform(xform)
        xform = state.getWorldTransform()
        origin = xform.getOrigin()
        self.assertEqual(origin.x, 3)
        self.assertEqual(origin.y, 5)
        self.assertEqual(origin.z, 7)



class Box2dShapeTests(TestCase):
    def test_instantiate(self):
        shape = Box2dShape(Vector3(3, 5, 7))
        self.assertTrue(isinstance(shape, CollisionShape))



class BoxShapeTests(TestCase):
    def test_instantiate(self):
        shape = BoxShape(Vector3(2, 4, 6))
        self.assertTrue(isinstance(shape, CollisionShape))



class SphereShapeTests(TestCase):
    def test_instantiate(self):
        shape = SphereShape(3.0)
        self.assertTrue(isinstance(shape, CollisionShape))



class CapsuleShapeTests(TestCase):
    def test_instantiate(self):
        shape = CapsuleShape(1.0, 2.0)
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
    def setUp(self):
        self.shape = BoxShape(Vector3(1, 2, 3))
        self.controller = KinematicCharacterController(self.shape, 2.5, 1)


    def test_instantiate(self):
        self.assertTrue(isinstance(self.controller, ActionInterface))


    def test_setWalkDirection(self):
        self.controller.setWalkDirection(Vector3(1, 0, 0))


    def test_setVelocityForTimeInterval(self):
        self.controller.setVelocityForTimeInterval(Vector3(12.0, 0, 0), 6.0)


    def test_ghost(self):
        self.assertTrue(isinstance(self.controller.ghost, CollisionObject))


    def test_warp(self):
        self.controller.warp(Vector3(5, 7, 9))
        transform = self.controller.ghost.getWorldTransform()
        origin = transform.getOrigin()
        self.assertEquals(origin.x, 5)
        self.assertEquals(origin.y, 7)
        self.assertEquals(origin.z, 9)



class CollisionObjectTests(TestCase):
    def test_restitution(self):
        obj = CollisionObject()
        obj.setRestitution(1.0)
        self.assertEqual(obj.getRestitution(), 1.0)


    def test_setCollisionShape(self):
        obj = CollisionObject()
        # Let go of the shape reference, it shouldn't matter.
        obj.setCollisionShape(SphereShape(3))
        shape = obj.getCollisionShape()
        self.assertTrue(isinstance(shape, SphereShape))
        self.assertEquals(shape.getRadius(), 3)


    def test_worldTransform(self):
        obj = CollisionObject()
        trans = Transform()
        trans.setIdentity()
        trans.setOrigin(Vector3(3, 5, 7))
        obj.setWorldTransform(trans)
        origin = obj.getWorldTransform().getOrigin()
        self.assertEquals(origin.x, 3)
        self.assertEquals(origin.y, 5)
        self.assertEquals(origin.z, 7)



class RigidBodyTests(TestCase):
    def test_setAngularFactor(self):
        body = RigidBody()
        body.setAngularFactor(1.0)


    def test_setLinearVelocity(self):
        body = RigidBody()
        body.setLinearVelocity(Vector3(1, 2, 3))


    def test_applyCentralForce(self):
        body = RigidBody(mass=1.0)
        body.applyCentralForce(Vector3(1, 2, 3))
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(0, 0, 0))
        world.addRigidBody(body)
        expectedSteps = 64
        numSteps = world.stepSimulation(1.0, expectedSteps, 1.0 / expectedSteps)
        self.assertEqual(numSteps, expectedSteps)
        position = body.getMotionState().getWorldTransform().getOrigin()
        self.assertEqual(position.x, 0.5 + 0.5 / expectedSteps)
        self.assertEqual(position.y, 1.0 + 1.0 / expectedSteps)
        self.assertEqual(position.z, 1.5 + 1.5 / expectedSteps)


    def test_isInWorld(self):
        body = RigidBody()
        self.assertFalse(body.isInWorld())
        world = DiscreteDynamicsWorld()
        world.addRigidBody(body)
        self.assertTrue(body.isInWorld())
        world.removeRigidBody(body)
        self.assertFalse(body.isInWorld())



class CollisionWorldTests(TestCase):
    def test_empty(self):
        world = CollisionWorld()
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_addCollisionObject(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = SphereShape(3)
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 1)


    def test_removeCollisionObject(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = SphereShape(3)
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


    def test_removeRigidBody(self):
        world = DiscreteDynamicsWorld()
        body = RigidBody()
        world.addRigidBody(body)
        world.removeRigidBody(body)


    def test_addAction(self):
        world = DiscreteDynamicsWorld()
        action = KinematicCharacterController(SphereShape(1), 1.0, 1)
        world.addAction(action)


    def test_cycle(self):
        world = DiscreteDynamicsWorld()
        class Cheat(RigidBody):
            pass
        body = Cheat(None, BoxShape(Vector3(3, 4, 5)))
        body.cycle = world
        world.addRigidBody(body)
        del body, world
        import gc
        gc.collect()


    def test_stepSimulation(self):
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(1, 2, 3))
        obj = RigidBody(None, None, 1)
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


    def test_linearVelocity(self):
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(0, 0, 0))
        obj = RigidBody(None, None, 1)
        obj.setLinearVelocity(Vector3(1, 2, 3))
        world.addRigidBody(obj)
        expectedSteps = 64
        numSteps = world.stepSimulation(1.0, expectedSteps, 1.0 / expectedSteps)
        self.assertEqual(numSteps, expectedSteps)

        position = obj.getMotionState().getWorldTransform().getOrigin()
        self.assertEquals(position.x, 1)
        self.assertEquals(position.y, 2)
        self.assertEquals(position.z, 3)
