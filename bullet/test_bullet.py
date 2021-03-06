# Copyright (c) 2010 Jean-Paul Calderone
# See LICENSE for details.

from math import pi
from operator import mul
from gc import collect
from weakref import ref as weakref

from unittest import TestCase

import numpy

from bullet import (
    ACTIVE_TAG, ISLAND_SLEEPING, WANTS_DEACTIVATION, DISABLE_DEACTIVATION,
    DISABLE_SIMULATION,

    NO_DEBUG, DRAW_WIREFRAME, DRAW_AABB, DRAW_FEATURES_TEXT,
    DRAW_CONTACT_POINTS, DRAW_TEXT, DRAW_CONSTRAINTS, DRAW_CONSTRAINT_LIMITS,

    Vector3, Quaternion, Transform,
    CollisionShape, BoxShape, Box2dShape, SphereShape, CapsuleShape,
    CylinderShape, CylinderShapeX, CylinderShapeZ, StaticPlaneShape,
    IndexedMesh, TriangleIndexVertexArray, BvhTriangleMeshShape,

    PairCachingGhostObject, ActionInterface, KinematicCharacterController,
    BroadphaseProxy, DefaultMotionState,

    CollisionObject, RigidBody,
    OverlappingPairCache, HashedOverlappingPairCache, AxisSweep3,
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


    def test_normalized(self):
        """
        L{Vector3.normalized} returns a new L{Vector3} pointing in the same
        direction as the first L{Vector3} but with unit magnitude.
        """
        v = Vector3(3, 5, 7)
        n = v.normalized()
        self.assertEqual(v.x, 3)
        self.assertEqual(v.y, 5)
        self.assertEqual(v.z, 7)

        self.assertAlmostEqual(n.x, 0.3292927, 6)
        self.assertAlmostEqual(n.y, 0.5488213, 6)
        self.assertAlmostEqual(n.z, 0.7683498, 6)
        self.assertAlmostEqual(n.x ** 2 + n.y ** 2 + n.z ** 2, 1.0, 6)


    def test_vectorAddition(self):
        """
        A L{Vector3} instance added to another L{Vector3} instance results in a
        new L{Vector3} instance with components equal to the sum of the
        component pairs of the inputs.
        """
        v1 = Vector3(1, 2, 3)
        v2 = Vector3(2, -1, 1)
        v3 = v1 + v2
        self.assertEqual(v3.x, 3)
        self.assertEqual(v3.y, 1)
        self.assertEqual(v3.z, 4)


    def test_vectorSubtraction(self):
        """
        A L{Vector3} instance subtracted from another L{Vector3} instance
        results in a new L{Vector3} instance with components equal to the
        difference of the component pairs of the inputs.
        """
        v1 = Vector3(1, 2, 3)
        v2 = Vector3(2, -1, 1)
        v3 = v1 - v2
        self.assertEqual(v3.x, -1)
        self.assertEqual(v3.y, 3)
        self.assertEqual(v3.z, 2)


    def test_scalarMultiplication(self):
        """
        A L{Vector3} instance multiplied by an integer or a float results in a
        new L{Vector3} instance scaled by that amount relative to the original.
        """
        v = Vector3(2, 4, 6)
        v2 = v * 2
        v3 = v * 3.5
        self.assertEqual(v.x, 2)
        self.assertEqual(v.y, 4)
        self.assertEqual(v.z, 6)

        self.assertEqual(v2.x, 4)
        self.assertEqual(v2.y, 8)
        self.assertEqual(v2.z, 12)

        self.assertEqual(v3.x, 7)
        self.assertEqual(v3.y, 14)
        self.assertEqual(v3.z, 21)


    def test_cross(self):
        """
        L{Vector3.cross} accepts another L{Vector3} and returns a new L{Vector3}
        which is perpendicular to the first two.
        """
        v1 = Vector3(1, 0, 0)
        v2 = Vector3(0, 1, 0)
        v3 = v1.cross(v2)
        self.assertEqual(v3.x, 0)
        self.assertEqual(v3.y, 0)
        self.assertEqual(v3.z, 1)


    def test_dot(self):
        """
        L{Vector3.dot} accepts another L{Vector3} and returns a float which is
        the result of the dot product of the two vectors.
        """
        self.assertEqual(Vector3(1, 0, 0).dot(Vector3(0, 1, 0)), 0)
        self.assertEqual(Vector3(1, 0, 0).dot(Vector3(1, 0, 0)), 1)
        self.assertEqual(Vector3(2, 0, 0).dot(Vector3(2, 0, 0)), 4)


    def test_length(self):
        """
        L{Vector3.length} returns the length (magnitude) of the vector.
        """
        self.assertEqual(5, Vector3(5, 0, 0).length())
        self.assertEqual(5.5, Vector3(5.5, 0, 0).length())
        self.assertEqual(5, Vector3(3, 4, 0).length())
        self.assertEqual(5, Vector3(0, 3, 4).length())
        self.assertEqual(5, Vector3(3, 0, 4).length())



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


    def test_getAxisAndAngle(self):
        """
        A L{Quaternion} represents a rotation around an axis.
        L{Quaternion.getAngle} returns the rotation in radians,
        L{Quaternion.getAxis} returns the axis as a L{Vector3}.
        """
        quat = Quaternion.fromScalars(0, 0, 0, 1)
        axis = quat.getAxis()
        self.assertEqual(axis.x, 1)
        self.assertEqual(axis.y, 0)
        self.assertEqual(axis.z, 0)
        self.assertEqual(quat.getAngle(), 0)

        quat = Quaternion.fromScalars(0, 0, 1, 0)
        axis = quat.getAxis()
        self.assertEqual(axis.x, 0)
        self.assertEqual(axis.y, 0)
        self.assertEqual(axis.z, 1)
        self.assertAlmostEqual(quat.getAngle(), pi, 6)


    def test_multiplication(self):
        """
        Multiplying one L{Quaternion} by another returns a new L{Quaternion}
        giving the product of the two.
        """
        a = Quaternion.fromAxisAngle(Vector3(1, 0, 0), pi / 4)
        b = Quaternion.fromAxisAngle(Vector3(1, 0, 0), pi / 6)
        c = a * b
        axis = c.getAxis()
        angle = c.getAngle()
        self.assertEqual(axis.x, 1)
        self.assertEqual(axis.y, 0)
        self.assertEqual(axis.z, 0)
        self.assertAlmostEqual(angle, pi / 4 + pi / 6, 6)


    def test_unrelatedMultiplication(self):
        """
        Multiplying a L{Quaternion} but another type results in a L{TypeError}.
        """
        self.assertRaises(
            TypeError, mul, Quaternion.fromScalars(0, 0, 0, 1), "foo")



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



class StaticPlaneShapeTests(TestCase):
    """
    Tests for L{StaticPlaneShape}, a shape representing an infinite, immobile
    plane.
    """
    def test_instantiate(self):
        """
        L{StaticPlaneShape} is initialized with a L{Vector3} giving its surface
        normal and a I{plane constant} XXX TODO what the heck, plane constant?
        """
        shape = StaticPlaneShape(Vector3(3, 5, 9), 1)
        self.assertTrue(isinstance(shape, StaticPlaneShape))



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



class CylinderShapeTestsMixin(object):
    def test_cannotInstantiateWithoutHalfExtents(self):
        """
        If a Vector3 is not passed as the only argument to the cylinder's
        initializer, L{TypeError} is raised.
        """
        self.assertRaises(TypeError, CylinderShape)
        self.assertRaises(TypeError, CylinderShape, 1)
        self.assertRaises(TypeError, CylinderShape, None)
        self.assertRaises(TypeError, CylinderShape, "foo")
        self.assertRaises(TypeError, CylinderShape, (1, 2, 3))
        self.assertRaises(TypeError, CylinderShape, Vector3(1, 2, 3), 5)


    def test_getHalfExtentsWithoutMargin(self):
        """
        L{CylinderShape.getHalfExtentsWithoutMargin} returns the cylinder's half
        extents vector.
        """
        halfExtents = Vector3(2, 5, 7)
        shape = CylinderShape(halfExtents)
        result = shape.getHalfExtentsWithoutMargin()
        self.assertEqual(result.x, 1.9600000381469727)
        self.assertEqual(result.y, 4.9600000381469727)
        self.assertEqual(result.z, 6.9600000381469727)



class CylinderShapeTests(TestCase, CylinderShapeTestsMixin):
    def test_instantiate(self):
        shape = CylinderShape(Vector3(1, 2, 3))
        self.assertTrue(isinstance(shape, CylinderShape))


    def test_radius(self):
        """
        L{CylinderShape.getRadius} returns the radius of the cylinder.
        """
        shape = CylinderShape(Vector3(1, 2, 3))
        self.assertEqual(shape.getRadius(), 1)





class CylinderShapeXTests(TestCase, CylinderShapeTestsMixin):
    def test_instantiate(self):
        shape = CylinderShapeX(Vector3(1, 2, 3))
        self.assertTrue(isinstance(shape, CylinderShapeX))


    def test_radius(self):
        """
        L{CylinderShapeX.getRadius} returns the radius of the cylinder.
        """
        shape = CylinderShapeX(Vector3(1, 2, 3))
        self.assertEqual(shape.getRadius(), 2)



class CylinderShapeZTests(TestCase, CylinderShapeTestsMixin):
    def test_instantiate(self):
        shape = CylinderShapeZ(Vector3(1, 2, 3))
        self.assertTrue(isinstance(shape, CylinderShapeZ))


    def test_radius(self):
        """
        L{CylinderShapeZ.getRadius} returns the radius of the cylinder.
        """
        shape = CylinderShapeZ(Vector3(1, 2, 3))
        self.assertEqual(shape.getRadius(), 1)



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
        self.ghost = PairCachingGhostObject()
        self.ghost.setCollisionShape(self.shape)
        self.controller = KinematicCharacterController(self.ghost, 2.5, 1)


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


    def test_gravity(self):
        """
        L{KinematicCharacterController.setGravity} specifies the effective
        gravity for the character as a scalar on the Y axis.
        L{KinematicCharacterController.getGravity} returns the previously set
        gravity.
        """
        self.controller.setGravity(42.5)
        self.assertEqual(42.5, self.controller.getGravity())



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


    def test_activationState(self):
        """
        CollisionObject.setActivationState changes an objects activation state
        and CollisionObject.getActivationState returns the current state.
        """
        for state in [ACTIVE_TAG, ISLAND_SLEEPING, WANTS_DEACTIVATION,
                      DISABLE_DEACTIVATION, DISABLE_SIMULATION]:
            obj = CollisionObject()
            obj.setActivationState(state)
            self.assertEqual(state, obj.getActivationState())


    def test_getBroadphaseHandle(self):
        """
        L{CollisionObject.getBroadphaseHandle} returns C{None} when there is no
        associated BroadphaseProxy (one is probably supplied only after the
        object is added to a L{CollisionWorld}).
        """
        obj = CollisionObject()
        self.assertEqual(None, obj.getBroadphaseHandle())



class RigidBodyCollisionShapeTests(TestCase):
    """
    Tests for L{RigidBody.getCollisionShape}.
    """
    def test_defaulInitializer(self):
        """
        If no shape is passed to L{RigidBody.__init__}, the default BoxShape is
        returned from L{RigidBody.getCollisionShape}.
        """
        self.assertTrue(isinstance(RigidBody().getCollisionShape(), BoxShape))


    def test_initializedValue(self):
        """
        If a shape is passed to L{RigidBody.__init__}, that shape is returned
        from L{RigidBody.getCollisionShape}.
        """
        shape = SphereShape(3)
        body = RigidBody(shape=shape)
        self.assertTrue(shape is body.getCollisionShape())



class RigidBodyTests(TestCase):
    def test_fromConstructionInfo(self):
        mass = 8
        motion = DefaultMotionState()
        shape = SphereShape(3)
        inertia = Vector3(1, 2, 3)
        transform = Transform()
        transform.setOrigin(Vector3(5, 6, 7))

        linearDamping = 0.3
        angularDamping = 0.4
        friction = 4.5
        restition = 5.4
        linearSleepingThreshold = 4.3
        angularSleepingThreshold = 3.2

        body = RigidBody.fromConstructionInfo(
            motion, shape, mass, inertia, transform,
            linearDamping, angularDamping, friction, restition,
            linearSleepingThreshold, angularSleepingThreshold)

        self.assertTrue(isinstance(body, RigidBody))
        self.assertEqual(1. / mass, body.getInvMass())
        self.assertTrue(motion is body.getMotionState())
        self.assertTrue(shape is body.getCollisionShape())
        self.assertAlmostEqual(
            1. / inertia.x, body.getInvInertiaDiagLocal().x, 6)
        self.assertAlmostEqual(
            1. / inertia.y, body.getInvInertiaDiagLocal().y, 6)
        self.assertAlmostEqual(
            1. / inertia.z, body.getInvInertiaDiagLocal().z, 6)

        # The worldTransform is ignored if a MotionState is supplied.
        self.assertEqual(0, body.getWorldTransform().getOrigin().x)
        self.assertEqual(0, body.getWorldTransform().getOrigin().y)
        self.assertEqual(0, body.getWorldTransform().getOrigin().z)

        self.assertAlmostEqual(linearDamping, body.getLinearDamping(), 6)
        self.assertAlmostEqual(angularDamping, body.getAngularDamping(), 6)
        self.assertAlmostEqual(friction, body.getFriction(), 6)
        self.assertAlmostEqual(restition, body.getRestitution(), 6)
        self.assertAlmostEqual(
            linearSleepingThreshold, body.getLinearSleepingThreshold(), 6)
        self.assertAlmostEqual(
            angularSleepingThreshold, body.getAngularSleepingThreshold(), 6)


    def test_fromConstructionInfoWithoutMotionState(self):
        """
        When L{RigidBody.fromConstructionInfo} is used to construct a
        L{RigidBody} without a L{MotionState}, the C{worldTransform} parameter
        is used to specify the body's initial position.
        """
        mass = 8
        shape = SphereShape(3)
        inertia = Vector3(1, 2, 3)
        startTransform = Transform()
        startTransform.setOrigin(Vector3(5, 6, 7))

        linearDamping = 0.3
        angularDamping = 0.4
        friction = 4.5
        restition = 5.4
        linearSleepingThreshold = 4.3
        angularSleepingThreshold = 3.2

        body = RigidBody.fromConstructionInfo(
            None, shape, mass, inertia, startTransform,
            linearDamping, angularDamping, friction, restition,
            linearSleepingThreshold, angularSleepingThreshold)

        origin = body.getWorldTransform().getOrigin()
        self.assertEqual(5, origin.x)
        self.assertEqual(6, origin.y)
        self.assertEqual(7, origin.z)


    def test_getInvMass(self):
        """
        L{RigidBody.getInvMass} returns the inverse of the mass of the body.
        """
        body = RigidBody(mass=16)
        self.assertEqual(1. / 16, body.getInvMass())


    def test_getInvInertiaDiagLocal(self):
        """
        L{RigidBody.getInvInertiaDiagLocal} returns the inverse of the local
        inertia vector.
        """
        body = RigidBody()
        inertia = body.getInvInertiaDiagLocal()
        self.assertTrue(isinstance(inertia, Vector3))


    def test_setAngularFactor(self):
        body = RigidBody()
        body.setAngularFactor(1.0)


    def test_linearVelocity(self):
        body = RigidBody()
        body.setLinearVelocity(Vector3(1, 2, 3))
        velocity = body.getLinearVelocity()
        self.assertEquals(velocity.x, 1)
        self.assertEquals(velocity.y, 2)
        self.assertEquals(velocity.z, 3)


    def test_friction(self):
        body = RigidBody()
        body.setFriction(3.5)
        self.assertEqual(3.5, body.getFriction())


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



class HashedOverlappingPairCacheTests(TestCase):
    def test_setInternalGhostPairCallback(self):
        cache = HashedOverlappingPairCache()
        cache.setInternalGhostPairCallback()


class AxisSweep3Tests(TestCase):
    def test_getOverlappingPairCache(self):
        broadphase = AxisSweep3(Vector3(0, 0, 0), Vector3(1, 1, 1))
        cache = broadphase.getOverlappingPairCache()
        self.assertTrue(isinstance(cache, OverlappingPairCache))


class WorldObjectGCMixin(object):
    def _worldKeepsObjectAlive(self, worldType, objType, objName=None):
        world = worldType()

        obj = objType()

        # If it can have a collision shape set, set it.
        if getattr(obj, 'setCollisionShape', None):
            obj.setCollisionShape(SphereShape(1))

        if objName is None:
            objName = objType.__name__

        add = getattr(worldType, 'add' + objName)
        add(world, obj)

        ref = weakref(obj)
        del obj
        collect()

        self.assertNotEqual(None, ref())


    def _worldForgetsRemovedObject(self, worldType, objType, objName=None):
        world = worldType()

        obj = objType()

        # If it can have a collision shape set, set it.
        if getattr(obj, 'setCollisionShape', None):
            obj.setCollisionShape(SphereShape(1))

        if objName is None:
            objName = objType.__name__

        add = getattr(worldType, 'add' + objName)
        remove = getattr(worldType, 'remove' + objName)
        add(world, obj)
        remove(world, obj)

        ref = weakref(obj)
        del obj
        collect()

        self.assertEqual(None, ref())


    def _worldCollectionForgetsObject(self, worldType, objType, objName=None):
        world = worldType()

        obj = objType()

        # If it can have a collision shape set, set it.
        if getattr(obj, 'setCollisionShape', None):
            obj.setCollisionShape(SphereShape(1))

        if objName is None:
            objName = objType.__name__

        add = getattr(worldType, 'add' + objName)
        add(world, obj)

        ref = weakref(obj)
        del obj, world
        collect()

        self.assertEqual(None, ref())



class CollisionWorldTests(TestCase, WorldObjectGCMixin):
    def test_empty(self):
        world = CollisionWorld()
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_addCollisionObject(self):
        """
        L{CollisionWorld.addCollisionObject} accepts a L{CollisionObject}
        instance and adds it to the world for collision detection.  A default
        collision group and collision mask are used if they are not passed.
        """
        world = CollisionWorld()
        obj = CollisionObject()
        shape = SphereShape(3)
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 1)
        proxy = obj.getBroadphaseHandle()
        self.assertEqual(BroadphaseProxy.DefaultFilter, proxy.collisionFilterGroup)
        self.assertEqual(BroadphaseProxy.AllFilter, proxy.collisionFilterMask)


    def test_addCollisionObjectWithoutShape(self):
        """
        L{CollisionWorld.addCollisionObject} raises L{ValueError} when passed a
        L{CollisionObject} with no defined L{CollisionShape}.
        """
        world = CollisionWorld()
        obj = CollisionObject()
        self.assertRaises(ValueError, world.addCollisionObject, obj)


    def test_addCollisionObjectCustomGroupAndMask(self):
        """
        The collision group and collision mask passed as the second and third
        arguments to L{CollisionWorld.addCollisionObject} are used for collision
        detections on the L{CollisionObject} added.
        """
        world = CollisionWorld()
        obj = CollisionObject()
        shape = SphereShape(3)
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj, 123, 456)
        self.assertEqual(world.getNumCollisionObjects(), 1)
        proxy = obj.getBroadphaseHandle()
        self.assertEqual(123, proxy.collisionFilterGroup)
        self.assertEqual(456, proxy.collisionFilterMask)


    def test_removeCollisionObject(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = SphereShape(3)
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        world.removeCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_removeCollisionObjectNeverAdded(self):
        """
        Though it is useless and should be avoided, calling
        L{CollisionWorld.removeCollisionObject} with a L{CollisionWorld} which
        is not currently part of the L{CollisionWorld} is a no-op.
        """
        world = CollisionWorld()
        obj = CollisionObject()
        world.removeCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_collisionWorldKeepsCollisionObjectAlive(self):
        """
        When a L{CollisionObject} has been added to a L{CollisionWorld} using
        L{CollisionWorld.addCollisionObject}, the L{CollisionObject} is kept
        alive even if no other references to it exist.
        """
        self._worldKeepsObjectAlive(CollisionWorld, CollisionObject)


    def test_collisionWorldForgetsRemovedCollisionObject(self):
        """
        After a L{CollisionObject} which was previously added to a
        L{CollisionWorld} is removed from it using
        L{CollisionWorld.removeCollisionObject}, the L{CollisionWorld} no longer
        keeps the L{CollisionObject} alive.
        """
        self._worldForgetsRemovedObject(CollisionWorld, CollisionObject)


    def test_collisionWorldCollectionForgetsCollisionObject(self):
        """
        Any L{CollisionObject}s which are still part of a L{CollisionWorld}
        (having been passed to L{CollisionWorld.addCollisionObject} but not
        L{CollisionWorld.removeCollisionObject}) when the L{CollisionWorld} is
        collected are no longer kept alive by that L{CollisionWorld} afterwards.
        """
        self._worldCollectionForgetsObject(CollisionWorld, CollisionObject)


    def test_addBoxShape(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = BoxShape(Vector3(2, 3, 4))
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)


    def test_setDebugDrawer(self):
        """
        An object can be specified as the debug drawer for a world.
        """
        world = CollisionWorld()
        drawer = object()
        world.setDebugDrawer(drawer)



class DebugRecorder(object):
    def __init__(self):
        self.mode = 0
        self.lines = []
        self.contacts = []


    def drawLine(self, *args):
        self.lines.append(args)


    def drawContactPoint(self, *args):
        self.contacts.append(args)


    def setDebugMode(self, mode):
        self.mode = mode


    def getDebugMode(self):
        return self.mode





class DebugDrawerTests(TestCase):
    def setUp(self):
        self.world = DiscreteDynamicsWorld()
        self.recorder = DebugRecorder()
        self.recorder.setDebugMode(DRAW_WIREFRAME | DRAW_CONTACT_POINTS)
        self.world.setDebugDrawer(self.recorder)


    def test_debugFlags(self):
        """
        Various integer debug flags are exposed on the bullet module.
        """
        # Values should agree with those from btIDebugDraw.h
        self.assertEqual(0, NO_DEBUG)
        self.assertEqual(1 << 0, DRAW_WIREFRAME)
        self.assertEqual(1 << 1, DRAW_AABB)
        self.assertEqual(1 << 2, DRAW_FEATURES_TEXT)
        self.assertEqual(1 << 3, DRAW_CONTACT_POINTS)
        self.assertEqual(1 << 6, DRAW_TEXT)
        self.assertEqual(1 << 11, DRAW_CONSTRAINTS)
        self.assertEqual(1 << 12, DRAW_CONSTRAINT_LIMITS)


    def test_lines(self):
        """
        Some lines are drawn using the debug drawer when
        L{CollisionWorld.debugDrawWorld} is called.
        """
        obj = CollisionObject()
        shape = BoxShape(Vector3(1, 2, 3))
        obj.setCollisionShape(shape)
        self.world.addCollisionObject(obj)
        self.world.debugDrawWorld()
        self.assertTrue(len(self.recorder.lines) > 0)
        for line in self.recorder.lines:
            self.assertEquals(len(line), 9)


    def test_collisions(self):
        """
        When objects collide and L{CollisionWorld.debugDrawWorld} is called,
        collisions are drawn using the debug drawer.
        """
        def objAt(pos):
            obj = RigidBody(None, BoxShape(Vector3(1, 1, 1)), 1.0)
            xform = Transform()
            xform.setOrigin(pos)
            obj.setWorldTransform(xform)
            return obj

        first = objAt(Vector3(-2, 0, 0))
        first.applyCentralForce(Vector3(50, 0, 0))
        second = objAt(Vector3(2, 0, 0))
        second.applyCentralForce(Vector3(-50, 0, 0))

        self.world.addRigidBody(first)
        self.world.addRigidBody(second)

        # Simulate the world enough for them to hit each other
        for i in range(100):
            self.world.stepSimulation(1.0 / 60.0)
            self.world.debugDrawWorld()
        self.assertTrue(len(self.recorder.contacts) > 0)
        for contact in self.recorder.contacts:
            self.assertEquals(len(contact), 11)



class DiscreteDynamicsWorldTests(TestCase, WorldObjectGCMixin):
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
        """
        A L{RigidBody} can be added to a L{DiscreteDynamicsWorld} using
        L{DiscreteDynamicsWorld.addRigidBody}.  The default collision group is
        L{BroadphaseProxy.DefaultFilter} and the default collision mask is
        L{BroadphaseProxy.AllFilter}.
        """
        world = DiscreteDynamicsWorld()
        body = RigidBody()
        world.addRigidBody(body)
        proxy = body.getBroadphaseHandle()
        self.assertEqual(
            BroadphaseProxy.DefaultFilter, proxy.collisionFilterGroup)
        self.assertEqual(
            BroadphaseProxy.AllFilter, proxy.collisionFilterMask)


    def test_removeRigidBody(self):
        """
        A L{RigidBody} in a L{DiscreteDynamicsWorld} can be removed using
        L{DiscreteDynamicsWorld.removeRigidBody}.
        """
        world = DiscreteDynamicsWorld()
        body = RigidBody()
        world.addRigidBody(body)
        world.removeRigidBody(body)


    def test_addRigidBodyCustomGroupAndMask(self):
        """
        The collision group and collision mask passed as the second and third
        arguments to L{DiscreteDynamicsWorld.addRigidBody} are used for
        collision detections on the L{RigidBody} added.
        """
        world = DiscreteDynamicsWorld()
        body = RigidBody()
        world.addRigidBody(body, 1234, 2345)
        proxy = body.getBroadphaseHandle()
        self.assertEqual(proxy.collisionFilterGroup, 1234)
        self.assertEqual(proxy.collisionFilterMask, 2345)


    def test_dynamicsWorldKeepsRigidBodyAlive(self):
        """
        When a L{RigidBody} has been added to a L{DiscreteDynamicsWorld} using
        L{DiscreteDynamicsWorld.addRigidBody}, the L{RigidBody} is kept alive
        even if no other references to it exist.
        """
        self._worldKeepsObjectAlive(DiscreteDynamicsWorld, RigidBody)


    def test_dynamicsWorldForgetsRemovedRigidBody(self):
        """
        After a L{RigidBody} which was previously added to a
        L{DiscreteDynamicsWorld} is removed from it using
        L{DiscreteDynamicsWorld.removeRigidBody}, the L{DiscreteDynamicsWorld}
        no longer keeps the L{RigidBody} alive.
        """
        self._worldForgetsRemovedObject(DiscreteDynamicsWorld, RigidBody)


    def test_dynamicsWorldCollectionForgetsRigidBody(self):
        """
        Any L{RigidBody}s which are still part of a L{DiscreteDynamicsWorld}
        (having been passed to L{DiscreteDynamicsWorld.addRigidBody} but not
        L{DiscreteDynamicsWorld.removeRigidBody}) when the
        L{DiscreteDynamicsWorld} is collected are no longer kept alive by that
        L{DiscreteDynamicsWorld} afterwards.
        """
        self._worldCollectionForgetsObject(DiscreteDynamicsWorld, RigidBody)


    def _createAction(self):
        shape = SphereShape(1)
        ghost = PairCachingGhostObject()
        ghost.setCollisionShape(shape)
        action = KinematicCharacterController(ghost, 1.0, 1)
        return action


    def test_addAction(self):
        action = self._createAction()
        world = DiscreteDynamicsWorld()
        world.addAction(action)


    def test_removeAction(self):
        action = self._createAction()
        world = DiscreteDynamicsWorld()
        world.addAction(action)
        world.removeAction(action)


    def test_dynamicsWorldKeepsActionAlive(self):
        """
        When an action has been added to a L{DiscreteDynamicsWorld} using
        L{DiscreteDynamicsWorld.addAction}, the action is kept alive even if no
        other references to it exist.
        """
        self._worldKeepsObjectAlive(
            DiscreteDynamicsWorld, self._createAction, 'Action')


    def test_dynamicsWorldForgetsRemovedAction(self):
        """
        After an action which was previously added to a L{DiscreteDynamicsWorld}
        is removed from it using L{DiscreteDynamicsWorld.removeAction}, the
        L{DiscreteDynamicsWorld} no longer keeps the action alive.
        """
        self._worldForgetsRemovedObject(
            DiscreteDynamicsWorld, self._createAction, 'Action')


    def test_dynamicsWorldCollectionForgetsAction(self):
        """
        Any actions which are still part of a L{DiscreteDynamicsWorld} (having
        been passed to L{DiscreteDynamicsWorld.addAction} but not
        L{DiscreteDynamicsWorld.removeAction}) when the L{DiscreteDynamicsWorld}
        is collected are no longer kept alive by that L{DiscreteDynamicsWorld}
        afterwards.
        """
        self._worldCollectionForgetsObject(
            DiscreteDynamicsWorld, self._createAction, 'Action')


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



class ControllerWorldIntegrationTests(TestCase):
    """
    Tests for use of L{KinematicCharacterController} in a
    L{DiscreteDynamicsWorld}.
    """
    def test_simulate(self):
        """
        A L{KinematicCharacterController}'s L{PairCachingGhostObject}'s which
        has been added to a L{DiscreteDynamicsWorld} is affected by the normal
        rules of dynamics upon L{DiscreteDynamicsWorld.stepSimulation}.
        """
        world = DiscreteDynamicsWorld()

        shape = BoxShape(Vector3(1, 1, 1))
        ghost = PairCachingGhostObject()
        ghost.setCollisionShape(shape)

        # Based on the Bullet examples, it might be correct to set this
        # collision flag.  However, it doesn't appear to actually make any
        # difference.
        # ghost.setCollisionFlags(CollisionObject.CF_CHARACTER_OBJECT)

        transform = Transform()
        transform.setOrigin(Vector3(1, 2, 3))
        ghost.setWorldTransform(transform)

        # Create the controller based on the ghost object just initialized.
        # Perhaps upAxis controls in which direction the gravity set below
        # points.
        controller = KinematicCharacterController(ghost, stepHeight=2, upAxis=1)
        controller.setGravity(-10)

        # Controllers must be added as actions so they have a chance to make
        # their non-physical interactions with the world.
        world.addAction(controller)

        # btBroadphaseProxy::CharacterFilter
        # btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter

        # Add the controller's ghost to the world so collisions can be detected.
        # It might be more correct to add it with a collision filter group of
        # CharacterFilter to mark it as a character.  However, there are no
        # other collision objects in this test so it can't make a difference
        # here.  It might also be more correct to add it with a collision filter
        # like ~CharacterFilter to avoid collisions with other characters.
        # However, again, no other collision objects in this test, so it matters
        # not.
        world.addCollisionObject(ghost)

        # Use an insanely fine grained simulation so that the ultimate position
        # is pretty close to what we would predict using actual (accurate) math
        # instead of (inaccurate) simulation math.
        expectedSteps = 1024
        timeStep = 1.0 / expectedSteps
        steps = world.stepSimulation(timeStep * expectedSteps, expectedSteps, timeStep)
        self.assertEqual(expectedSteps, steps)


        # Original position plus gravitational effects over one second
        expected = (1, 2 + 5, 3)

        # Verify the character's ghost has moved in accordance with gravity.
        origin = ghost.getWorldTransform().getOrigin()

        # Dynamics (gravity) only moves the ghost along the upAxis, so the other
        # coordinates should be exact.

        self.assertEqual(expected[0], origin.x)
        self.assertAlmostEqual(expected[1], origin.y, 2)
        self.assertEqual(expected[2], origin.z)



class CrashTests(TestCase):
    """
    Tests for uses of L{bullet} which at some point triggered crashes of one sort of another.
    """
    def test_collisionObjectInCollisionWorld(self):
        """
        A L{CollisionObject} cannot safely be deallocated before the
        L{CollisionWorld} which contains it.
        """
        world = CollisionWorld()
        shape = SphereShape(1)
        obj = CollisionObject()
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        del obj
        collect()
        del world
        collect()


    def test_rigidBodyInDiscreteDynamicsWorld(self):
        """
        A L{RigidBody} cannot safely be deallocated before the
        L{DiscreteDynamicsWorld} which contains it.
        """
        world = DiscreteDynamicsWorld()
        body = RigidBody(shape=SphereShape(1))
        world.addRigidBody(body)
        del body
        collect()
        del world
        collect()


    def test_ghostInDiscreteDynamicsWorld(self):
        """
        A L{PairCachingGhostObject} cannot safely be deallocated before the
        L{DiscreteDynamicsWorld} which contains it.
        """
        broadphase = AxisSweep3(Vector3(-1, -1, -1), Vector3(1, 1, 1))
        paircache = broadphase.getOverlappingPairCache()
        paircache.setInternalGhostPairCallback()
        world = DiscreteDynamicsWorld()
        ghost = PairCachingGhostObject()
        ghost.setCollisionShape(SphereShape(1))
        world.addCollisionObject(ghost)

        del ghost
        collect()
        del paircache
        collect()
        del broadphase
        collect()
        del world
        collect()
