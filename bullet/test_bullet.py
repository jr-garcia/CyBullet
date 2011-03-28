# Copyright (c) 2010 Jean-Paul Calderone
# See LICENSE for details.

from unittest import TestCase

import numpy

from bullet import (
    Vector3, Quaternion, Transform,
    CollisionShape, BoxShape, Box2dShape, SphereShape, CapsuleShape,
    IndexedMesh, TriangleIndexVertexArray, BvhTriangleMeshShape,
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



class QuaternionTests(TestCase):
    def test_fromScalars(self):
        """
        A L{Quaternion} can be constructed from four scalar values giving x, y,
        z, and w.
        """
        quat = Quaternion.fromScalars(1, 2, 3, 4)
        self.assertTrue(isinstance(quat, Quaternion))
        self.assertEquals(quat.getX(), 1)
        self.assertEquals(quat.getY(), 2)
        self.assertEquals(quat.getZ(), 3)
        self.assertEquals(quat.getW(), 4)


    def test_fromAxisAngle(self):
        """
        A L{Quaternion} can be constructed from a Vector3 giving an axis and a
        scalar giving an angle from that axis.
        """
        quat = Quaternion.fromAxisAngle(Vector3(0, 1, 0), 45)
        self.assertTrue(isinstance(quat, Quaternion))
        # XXX Assert something about the value



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


    def test_rotation(self):
        transform = Transform()
        transform.setRotation(Quaternion.fromScalars(1, 2, 3, 4))
        quat = transform.getRotation()
        self.assertTrue(isinstance(quat, Quaternion))
        self.assertEquals(quat.getX(), 0.18257419764995575)
        self.assertEquals(quat.getY(), 0.3651483952999115)
        self.assertEquals(quat.getZ(), 0.54772257804870605)
        self.assertEquals(quat.getW(), 0.73029673099517822)

    # XXX Quaternion(1, 2, 3, 4) segfaults?



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



class IndexedMeshTests(TestCase):
    def test_instantiate(self):
        mesh = IndexedMesh()
        self.assertTrue(isinstance(mesh, IndexedMesh))


    def test_setIndicesInvalidArray(self):
        mesh = IndexedMesh()
        self.assertRaises(TypeError, mesh.setIndices, 0, 0, None)
        self.assertRaises(
            ValueError, mesh.setIndices, 0, 0, numpy.array([], 'Q'))


    def test_setIndices(self):
        mesh = IndexedMesh()
        mesh.setIndices(3, 2, numpy.array([1, 2, 3], 'i'))


    def test_setVerticesInvalidArray(self):
        mesh = IndexedMesh()
        self.assertRaises(TypeError, mesh.setVertices, 0, 0, None)
        self.assertRaises(
            ValueError, mesh.setVertices, 0, 0, numpy.array([], 'Q'))


    def test_setVertices(self):
        mesh = IndexedMesh()
        mesh.setVertices(1, 2, numpy.array([1, 2, 3], 'i'))



class TriangleIndexVertexArrayTests(TestCase):
    def test_rejectIntegerIndexedMesh(self):
        """
        TriangleIndexVertexArray.addIndexedMesh raises ValueError if passed an
        IndexedMesh containing anything other than floats or doubles.
        """
        triangles = TriangleIndexVertexArray()
        mesh = IndexedMesh()
        mesh.setIndices(1, 0, numpy.array([0, 1, 2] * 3, 'i'))
        mesh.setVertices(3, 0, numpy.array([3, 4, 5], 'i'))
        self.assertRaises(ValueError, triangles.addIndexedMesh, mesh)


    def test_addIndexedMesh(self):
        triangles = TriangleIndexVertexArray()

        for i in range(3):
            mesh = IndexedMesh()
            mesh.setIndices(1, 0, numpy.array([0, 1, 2] * 3, 'i'))
            mesh.setVertices(3, 0, numpy.array([3, 4, 5], 'f'))
            triangles.addIndexedMesh(mesh)


    def test_getNumSubParts(self):
        """
        TriangleIndexVertexArray.getNumSubParts returns the number of indexed
        meshes which have been added to the TriangleIndexVertexArray.
        """
        triangles = TriangleIndexVertexArray()
        self.assertEquals(triangles.getNumSubParts(), 0)
        mesh = IndexedMesh()
        mesh.setIndices(1, 0, numpy.array([0, 1, 2] * 3, 'i'))
        mesh.setVertices(3, 0, numpy.array([3, 4, 5], 'f'))
        triangles.addIndexedMesh(mesh)
        self.assertEquals(triangles.getNumSubParts(), 1)



class BvhTriangleMeshShapeTests(TestCase):
    def test_meshInitializer(self):
        shape = BvhTriangleMeshShape(TriangleIndexVertexArray())
        self.assertTrue(isinstance(shape, CollisionShape))


    def test_buildOptimizedBvh(self):
        """
        BvhTriangleMeshShape.buildOptimizedBvh does something.
        """
        triangles = TriangleIndexVertexArray()
        mesh = IndexedMesh()
        mesh.setIndices(1, 0, numpy.array([0, 1, 2] * 3, 'i'))
        mesh.setVertices(3, 0, numpy.array([3, 4, 5], 'f'))
        triangles.addIndexedMesh(mesh)
        shape = BvhTriangleMeshShape(triangles)
        shape.buildOptimizedBvh()



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
        transform = body.getMotionState().getWorldTransform()
        position = transform.getOrigin()
        self.assertEqual(position.x, 0.5 + 0.5 / expectedSteps)
        self.assertEqual(position.y, 1.0 + 1.0 / expectedSteps)
        self.assertEqual(position.z, 1.5 + 1.5 / expectedSteps)
        rot = transform.getRotation()
        self.assertEqual(
            (rot.getX(), rot.getY(), rot.getZ(), rot.getW()), (0, 0, 0, 1))


    def test_applyForce(self):
        body = RigidBody(mass=1.0)
        body.applyForce(Vector3(1, 2, 3), Vector3(1, 1, 1))
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(0, 0, 0))
        world.addRigidBody(body)
        expectedSteps = 64
        numSteps = world.stepSimulation(1.0, expectedSteps, 1.0 / expectedSteps)
        self.assertEqual(numSteps, expectedSteps)
        transform = body.getMotionState().getWorldTransform()
        position = transform.getOrigin()
        self.assertEqual(position.x, 0.5 + 0.5 / expectedSteps)
        self.assertEqual(position.y, 1.0 + 1.0 / expectedSteps)
        self.assertEqual(position.z, 1.5 + 1.5 / expectedSteps)
        rot = transform.getRotation()
        self.assertNotEqual(
            (rot.getX(), rot.getY(), rot.getZ(), rot.getW()),
            (0, 0, 0, 1))


    def test_applyCentralImpulse(self):
        body = RigidBody(mass=1.0)
        body.applyCentralImpulse(Vector3(1, 2, 3))
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(0, 0, 0))
        world.addRigidBody(body)
        expectedSteps = 64
        numSteps = world.stepSimulation(1.0, expectedSteps, 1.0 / expectedSteps)
        self.assertEqual(numSteps, expectedSteps)
        transform = body.getMotionState().getWorldTransform()
        position = transform.getOrigin()
        self.assertEqual(position.x, 1.0)
        self.assertEqual(position.y, 2.0)
        self.assertEqual(position.z, 3.0)
        rot = transform.getRotation()
        self.assertEqual(
            (rot.getX(), rot.getY(), rot.getZ(), rot.getW()), (0, 0, 0, 1))


    def test_applyImpulse(self):
        body = RigidBody(mass=1.0)
        body.applyImpulse(Vector3(1, 2, 3), Vector3(1, 1, 1))
        world = DiscreteDynamicsWorld()
        world.setGravity(Vector3(0, 0, 0))
        world.addRigidBody(body)
        expectedSteps = 64
        numSteps = world.stepSimulation(1.0, expectedSteps, 1.0 / expectedSteps)
        self.assertEqual(numSteps, expectedSteps)
        transform = body.getMotionState().getWorldTransform()
        position = transform.getOrigin()
        self.assertEqual(position.x, 1.0)
        self.assertEqual(position.y, 2.0)
        self.assertEqual(position.z, 3.0)
        rot = transform.getRotation()
        self.assertNotEqual(
            (rot.getX(), rot.getY(), rot.getZ(), rot.getW()),
            (0, 0, 0, 1))


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


    def test_setDebugDrawer(self):
        world = CollisionWorld()
        drawer = object()
        world.setDebugDrawer(drawer)


class DebugRecorder(object):
    def __init__(self):
        self.lines = []
        self.contacts = []


    def drawLine(self, *args):
        self.lines.append(args)


    def drawContactPoint(self, *args):
        self.contacts.append(args)



class DebugDrawerTests(TestCase):
    def setUp(self):
        self.world = CollisionWorld()
        self.recorder = DebugRecorder()
        self.world.setDebugDrawer(self.recorder)


    def test_lines(self):
        obj = CollisionObject()
        shape = BoxShape(Vector3(1, 2, 3))
        obj.setCollisionShape(shape)
        self.world.addCollisionObject(obj)
        self.world.debugDrawWorld()
        self.assertTrue(len(self.recorder.lines) > 0)
        for line in self.recorder.lines:
            self.assertEquals(len(line), 9)


    def test_collisions(self):
        first = CollisionObject()
        shape = BoxShape(Vector3(2, 1, 1))
        first.setCollisionShape(shape)
        self.world.addCollisionObject(first)

        second = CollisionObject()
        shape = BoxShape(Vector3(1, 2, 1))
        second.setCollisionShape(shape)
        self.world.addCollisionObject(second)

        self.world.debugDrawWorld()
        self.assertTrue(len(self.recorder.contacts) > 0)
        for contact in self.recorder.contacts:
            self.assertEquals(len(contact), 11)



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
