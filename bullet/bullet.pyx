# Copyright (c) Jean-Paul Calderone
# See LICENSE for details.

"""
PyBullet wraps the Bullet Physics library for Python.

Dynamics features are provided mainly by DynamicsWorld and RigidBody.  For
example:

    world = DiscreteDynamicsWorld()
    ball = RigidBody(shape=SphereShape(1.0), mass=1.0)
    world.addRigidBody(ball)
    world.stepSimulation(1.0, 60, 1.0 / 60.0)
    print ball.getWorldTransform().getOrigin()

"""

from libcpp cimport bool

cimport numpy


cdef extern from "Python.h":
    cdef void Py_INCREF( object )
    cdef void Py_DECREF( object )

    cdef struct _object:
        pass


cdef extern from "btBulletCollisionCommon.h":
    ctypedef float btScalar
    ctypedef int bool

    cdef enum PHY_ScalarType:
        PHY_FLOAT
        PHY_DOUBLE
        PHY_INTEGER
        PHY_SHORT
        PHY_FIXEDPOINT88
        PHY_UCHAR

    cdef cppclass btVector3


    cdef cppclass btIndexedMesh:
        int m_numTriangles
        unsigned char *m_triangleIndexBase
        int m_triangleIndexStride
        PHY_ScalarType m_indexType

        int m_numVertices
        unsigned char *m_vertexBase
        int m_vertexStride
        PHY_ScalarType m_vertexType


    cdef cppclass btQuaternion


    cdef cppclass btStridingMeshInterface:
        int getNumSubParts()


    cdef cppclass btTriangleIndexVertexArray(btStridingMeshInterface):
        btTriangleIndexVertexArray()
        btTriangleIndexVertexArray(
            int numTriangles,
            int *triangleIndexBase,
            int triangleIndexStride,
            int numVertices,
            btScalar *vertexBase,
            int vertexStride)

        void addIndexedMesh(btIndexedMesh &mesh, PHY_ScalarType indexType)


    cdef cppclass btCollisionShape:
        void calculateLocalInertia(btScalar mass, btVector3 &inertia)


    cdef cppclass btConvexShape(btCollisionShape):
        pass

    cdef cppclass btBoxShape(btConvexShape):
        btBoxShape(btVector3 boxHalfExtents)


    cdef cppclass btSphereShape(btConvexShape):
        btSphereShape(btScalar radius)

        btScalar getRadius()


    cdef cppclass btCapsuleShape(btConvexShape):
        btCapsuleShape(btScalar radius, btScalar height)


    cdef cppclass btBvhTriangleMeshShape(btConvexShape):
        btBvhTriangleMeshShape(
            btStridingMeshInterface* meshInterface,
            bool useQuantizedAabbCompression,
            bool buildBvh)


cdef extern from "BulletCollision/CollisionShapes/btBox2dShape.h":
    cdef cppclass btBox2dShape(btConvexShape):
        btBox2dShape(btVector3 boxHalfExtents)


cdef extern from "btBulletDynamicsCommon.h":
    cdef cppclass btTransform:
        btVector3 getOrigin()
        void setOrigin(btVector3)
        void setIdentity()
        void setRotation(btQuaternion&)
        btQuaternion getRotation()


    cdef cppclass btMotionState:
        void getWorldTransform(btTransform &transform)
        void setWorldTransform(btTransform &transform)


    cdef cppclass btDefaultMotionState(btMotionState):
        btDefaultMotionState()


    cdef cppclass btCollisionObject:
        btCollisionObject()

        btCollisionShape* getCollisionShape()
        void setCollisionShape(btCollisionShape*)

        void setRestitution(btScalar)
        btScalar getRestitution()

        btTransform& getWorldTransform()
        void setWorldTransform(btTransform& worldTrans)


    cdef cppclass btRigidBody(btCollisionObject)


    cdef cppclass btActionInterface:
        pass


    cdef cppclass btCharacterControllerInterface(btActionInterface):
        void setWalkDirection(btVector3 walkDirection)

        void setVelocityForTimeInterval(
            btVector3 velocity, btScalar timeInterval)



cdef extern from "BulletCollision/CollisionDispatch/btGhostObject.h":
    cdef cppclass btPairCachingGhostObject(btCollisionObject):
        pass


cdef extern from "BulletDynamics/Character/btKinematicCharacterController.h":
    cdef cppclass btKinematicCharacterController(btCharacterControllerInterface):
        btKinematicCharacterController(
            btPairCachingGhostObject *ghostObject,
            btConvexShape *convexShape,
            btScalar stepHeight, int upAxis)

        void warp(btVector3 origin)



cdef extern from "btBulletCollisionCommon.h" namespace "btRigidBody":
    cdef cppclass btRigidBodyConstructionInfo:
        btRigidBodyConstructionInfo()
        btRigidBodyConstructionInfo(
            btScalar mass,
            btMotionState *motionState,
            btCollisionShape *collisionShape,
            btVector3 localInteria)


cdef extern from "btBulletCollisionCommon.h":
    cdef cppclass btCollisionConfiguration:
        pass

    cdef cppclass btDefaultCollisionConfiguration(btCollisionConfiguration):
        pass

    cdef cppclass btDispatcher:
        pass

    cdef cppclass btCollisionDispatcher(btDispatcher):
        btCollisionDispatcher(btCollisionConfiguration*)

    cdef cppclass btVector3:
        btVector3()
        btVector3(btScalar, btScalar, btScalar)

        void setX(btScalar x)
        void setY(btScalar y)
        void setZ(btScalar z)

        btScalar getX()
        btScalar getY()
        btScalar getZ()


    cdef cppclass btQuaternion:
        btQuaternion()
        btQuaternion(btScalar x, btScalar y, btScalar z, btScalar w)
        btQuaternion(btVector3 axis, btScalar angle)

        btScalar getX()
        btScalar getY()
        btScalar getZ()
        btScalar getW()


    cdef cppclass btBroadphaseInterface:
        pass

    cdef cppclass btAxisSweep3(btBroadphaseInterface):
        btAxisSweep3(btVector3, btVector3)


    cdef cppclass btDbvtBroadphase(btBroadphaseInterface):
        pass


    cdef cppclass btRigidBody(btCollisionObject):
        btRigidBody(btRigidBodyConstructionInfo)

        bool isInWorld()

        btMotionState* getMotionState()

        void setAngularFactor(btScalar angFac)

        void setLinearVelocity(btVector3 velocity)

        void applyCentralForce(btVector3 force)
        void applyForce(btVector3 force, btVector3 relativePosition)

        void applyCentralImpulse(btVector3 impulse)
        void applyImpulse(btVector3 impulse, btVector3 relativePosition)


    cdef cppclass btCollisionWorld:
        btCollisionWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)

        btDispatcher *getDispatcher()
        btBroadphaseInterface *getBroadphase()

        int getNumCollisionObjects()

        void addCollisionObject(btCollisionObject*, short int, short int)
        void removeCollisionObject(btCollisionObject*)



cdef extern from "btBulletDynamicsCommon.h":
    cdef cppclass btConstraintSolver:
        pass


    cdef cppclass btSequentialImpulseConstraintSolver(btConstraintSolver):
        btSequentialImpulseConstraintSolver()


    cdef cppclass btDynamicsWorld: # (btCollisionWorld):
        btDynamicsWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)

        void setGravity(btVector3)
        btVector3 getGravity()

        void addRigidBody(btRigidBody*)
        void removeRigidBody(btRigidBody*)

        void addAction(btActionInterface*)

        int stepSimulation(btScalar, int, btScalar)


    cdef cppclass btDiscreteDynamicsWorld: # (byDynamicsWorld)
        btDiscreteDynamicsWorld(
            btDispatcher*, btBroadphaseInterface*,
            btConstraintSolver*, btCollisionConfiguration*)

        btConstraintSolver *getConstraintSolver()



# Forward declare some things because of circularity in the API.
cdef class CollisionObject



cdef class Vector3:
    """
    A Vector3 represents a point or vector in three-dimensional space.

    When dealing with most vector quantities such as velocity, position, and
    force, bullet will use a Vector3.

    This class is loosely a wrapper around btVector3.
    """
    cdef readonly btScalar x
    cdef readonly btScalar y
    cdef readonly btScalar z

    def __cinit__(self, btScalar x, btScalar y, btScalar z):
        self.x = x
        self.y = y
        self.z = z


    def __repr__(self):
        return '<Vector x=%s y=%s z=%s>' % (self.x, self.y, self.z)



cdef class Quaternion:
    """
    A Quaternion represents a point or vector in four-dimensional space.  It is
    frequently also used to represent rotation and orientation in
    three-dimensional space.

    This class is a wrapper around btQuaternion.
    """
    cdef btQuaternion* quaternion

    def __dealloc__(self):
        del self.quaternion


    @classmethod
    def fromScalars(cls, btScalar x, btScalar y, btScalar z, btScalar w):
        """
        Construct a new Quaternion from four scalar components.
        """
        q = Quaternion()
        q.quaternion = new btQuaternion(x, y, z, w)
        return q


    @classmethod
    def fromAxisAngle(cls, Vector3 axis not None, btScalar angle):
        """
        Construct a new Quaternion from an axis and a rotation around that axis.
        """
        q = Quaternion()
        q.quaternion = new btQuaternion(
            btVector3(axis.x, axis.y, axis.z), angle)
        return q


    def getX(self):
        """
        Get the X component of the Quaternion.
        """
        return self.quaternion.getX()


    def getY(self):
        """
        Get the Y component of the Quaternion.
        """
        return self.quaternion.getY()


    def getZ(self):
        """
        Get the Z component of the Quaternion.
        """
        return self.quaternion.getZ()


    def getW(self):
        """
        Get the W component of the Quaternion.
        """
        return self.quaternion.getW()



cdef class CollisionShape:
    """
    A CollisionShape defines the shape of an object in a CollisionWorld.  Shapes
    are used for collision detection.

    This class is a wrapper around btCollisionShape.
    """
    cdef btCollisionShape *thisptr


    def __dealloc__(self):
        del self.thisptr



cdef class ConvexShape(CollisionShape):
    """
    A ConvexShape is a shape that curves outwards only.  This is a base class
    for many other types of shapes which are convex.

    This class is loosely a wrapper around btConvexShape.
    """



cdef class Box2dShape(ConvexShape):
    """
    A Box2dShape is a box primitive around the origin, its sides axis aligned
    with length specified by half extents, in local shape coordinates.  It has
    size only in the X and Y dimensions.

    This class is a wrapper around btBox2dShape.
    """
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBox2dShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class BoxShape(ConvexShape):
    """
    A Box2dShape is a box primitive around the origin, its sides axis aligned
    with length specified by half extents, in local shape coordinates.  It has
    size only in the X and Y dimensions.

    This class is a wrapper around btBox2dShape.
    """
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBoxShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class SphereShape(ConvexShape):
    """
    A SphereShape is a sphere around the origin, with a specified radius.

    This class is a wrapper around btSphereShape.
    """
    def __cinit__(self, btScalar radius):
        self.thisptr = new btSphereShape(radius)


    def getRadius(self):
        """
        Return the radius of the sphere.
        """
        return (<btSphereShape*>self.thisptr).getRadius()



cdef class CapsuleShape(ConvexShape):
    """
    A CapsuleShape is a capsule around the Y axis.  A capsule is a cylinder with
    a sphere on each end.

    This class is a wrapper around btCapsuleShape.
    """
    def __cinit__(self, btScalar radius, btScalar height):
        self.thisptr = new btCapsuleShape(radius, height)


cdef class IndexedMesh:
    """
    An IndexedMesh is a contiguous vertex array and a contiguous array of index
    data into that vertex array.  It defines a mesh of triangles composed of
    triples of vertex data given by sequential triples of indices from the index
    array.

    This class is a wrapper around btIndexedMesh.
    """
    cdef btIndexedMesh* thisptr

    cdef PHY_ScalarType _dtypeToScalarType(self, numpy.ndarray array):
        cdef char *dname = array.dtype.char
        cdef char dtype = dname[0]

        if dtype == 'f':
            return PHY_FLOAT
        elif dtype == 'd':
            return PHY_DOUBLE
        elif dtype == 'i':
            return PHY_INTEGER
        elif dtype == 'h':
            return PHY_SHORT
        return <PHY_ScalarType>-1


    def __cinit__(self):
        self.thisptr = new btIndexedMesh()
        self.thisptr.m_numTriangles = 0
        self.thisptr.m_triangleIndexBase = NULL
        self.thisptr.m_triangleIndexStride = 0
        self.thisptr.m_numVertices = 0
        self.thisptr.m_vertexBase = NULL
        self.thisptr.m_vertexStride = 0
        self.thisptr.m_indexType = PHY_FLOAT
        self.thisptr.m_vertexType = PHY_FLOAT


    def setIndices(self, int numTriangles, int indexStride,
                   numpy.ndarray indexBase not None):
        """
        Specify the index data for for this IndexedMesh.

        numTriangles specifies the total number of triangles this mesh will
        contain.

        indexStride gives the distance in bytes between the start of each triple
        of values defining a triangle.

        indexBase is a numpy array giving the index data itself.
        """
        cdef PHY_ScalarType indexType = self._dtypeToScalarType(indexBase)
        if indexType == -1:
            raise ValueError("Unsupported index array type")

        self.thisptr.m_numTriangles = numTriangles
        self.thisptr.m_triangleIndexStride = indexStride
        self.thisptr.m_triangleIndexBase = <unsigned char*>indexBase.data
        self.thisptr.m_indexType = indexType


    def setVertices(self, int numVertices, int vertexStride,
                    numpy.ndarray vertexBase not None):
        """
        Specify the vertex data for this IndexedMesh.

        numVertices specifies the total number of vertices this mesh will
        contain.

        vertexStride gives the distance in bytes between the start of each
        triple of values defining a vertex.

        vertexBase is a numpy array giving the vertex data itself.
        """
        cdef PHY_ScalarType vertexType = self._dtypeToScalarType(vertexBase)
        if vertexType == -1:
            raise ValueError("Unsupported index array type")

        self.thisptr.m_numVertices = numVertices
        self.thisptr.m_vertexStride = vertexStride
        self.thisptr.m_vertexBase = <unsigned char*>vertexBase.data
        self.thisptr.m_vertexType = vertexType


    def __dealloc__(self):
        del self.thisptr



cdef class StridingMeshInterface:
    cdef btStridingMeshInterface *thisptr


    def __dealloc__(self):
        del self.thisptr


    def getNumSubParts(self):
        """
        Return the number of separate continuous vertex arrays are part of this
        StridingMeshInterface.
        """
        return self.thisptr.getNumSubParts()



cdef class TriangleIndexVertexArray(StridingMeshInterface):
    def __cinit__(self):
        self.thisptr = new btTriangleIndexVertexArray()


    def addIndexedMesh(self, IndexedMesh mesh not None):
        if mesh.thisptr.m_vertexType == PHY_INTEGER:
            raise ValueError("XXX")
        cdef btTriangleIndexVertexArray *array
        array = <btTriangleIndexVertexArray*>self.thisptr
        array.addIndexedMesh(mesh.thisptr[0], mesh.thisptr.m_indexType)



cdef class BvhTriangleMeshShape(ConvexShape):
    """
    A BvhTriangleMeshShape is a shape defined by an array of triangle vertex
    data and an array of indices against those triangles.

    This class is a wrapper around btBvhTriangleMeshShape.
    """
    cdef StridingMeshInterface stride

    def __init__(self, StridingMeshInterface mesh not None):
        self.stride = mesh
        self.thisptr = new btBvhTriangleMeshShape(mesh.thisptr, True, False)



cdef class Transform:
    """
    A Transform represents an a translation and rotation.

    When dealing with the location and orientation of an object in a
    CollisionWorld, Bullet will use a Transform instance.

    This class is a wrapper around btTransform.  Unlike btTransform, a Transform
    always starts off set to the identity.
    """
    cdef btTransform *thisptr

    def __cinit__(self):
        self.thisptr = new btTransform()
        self.thisptr.setIdentity()


    def __dealloc__(self):
        del self.thisptr


    def getOrigin(self):
        """
        Get the origin vector translation as a Vector3.
        """
        cdef btVector3 origin = self.thisptr.getOrigin()
        return Vector3(origin.getX(), origin.getY(), origin.getZ())


    def setOrigin(self, Vector3 origin not None):
        """
        Set the origin vector translation for this Transform.
        """
        self.thisptr.setOrigin(btVector3(origin.x, origin.y, origin.z))


    def setRotation(self, Quaternion rot not None):
        """
        Set the rotation of this Transform using a Quaternion.
        """
        cdef btQuaternion *quat = rot.quaternion
        self.thisptr.setRotation(quat[0])


    def getRotation(self):
        """
        Get the rotation of this Transform as a Quaternion.
        """
        cdef btQuaternion quat = self.thisptr.getRotation()
        return Quaternion.fromScalars(
            quat.getX(), quat.getY(), quat.getZ(), quat.getW())


    def setIdentity(self):
        """
        Set this Transform to be the identity.
        """
        self.thisptr.setIdentity()



cdef class CollisionObject:
    """
    A CollisionObject is something which can be collided with when added to a
    CollisionWorld.  It has a position and orientation as well as a
    CollisionShape.

    This class is a wrapper around btCollisionObject.
    """
    cdef btCollisionObject *thisptr
    cdef CollisionShape _shape

    def __init__(self):
        self.thisptr = new btCollisionObject()


    def __dealloc__(self):
        del self.thisptr


    def setRestitution(self, btScalar restitution):
        """
        Specify a scaling factor to be applied to the normal impulse computed
        when this object collides with another.
        """
        self.thisptr.setRestitution(restitution)


    def getRestitution(self):
        """
        Get the restitution value for this object.
        """
        return self.thisptr.getRestitution()


    def getCollisionShape(self):
        """
        Get the CollisionShape associated with this object.
        """
        return self._shape


    def setCollisionShape(self, CollisionShape collisionShape):
        """
        Replace this object's CollisionShape.
        """
        self.thisptr.setCollisionShape(collisionShape.thisptr)
        self._shape = collisionShape


    def getWorldTransform(self):
        """
        Get a copy of the transformation for this CollisionObject as a Transform
        instance.
        """
        cdef Transform transform = Transform()
        transform.thisptr[0] = self.thisptr.getWorldTransform()
        return transform


    def setWorldTransform(self, Transform transform not None):
        """
        Replace the transformation for this CollisionObject.
        """
        self.thisptr.setWorldTransform(transform.thisptr[0])



cdef class MotionState:
    """
    A MotionState is a primarily an object to provide callback methods to the
    dynamics implementation.

    A MotionState instance can be set on a RigidBody to receive world transform
    updates for that object.  Bullet may call the setWorldTransform callback
    only for objects which move to avoid the cost of doing so where no new
    information will be provided.

    This class is a wrapper around btMotionState.  This wrapping isn't entirely
    sensible, so it may change in a future release.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use DefaultMotionState instead.
    """
    cdef btMotionState *thisptr

    def __dealloc__(self):
        del self.thisptr


    def getWorldTransform(self):
        """
        Get a copy of the transformation for this motion state as a Transform
        instance.
        """
        transform = Transform()
        self.thisptr.getWorldTransform(transform.thisptr[0])
        return transform


    def setWorldTransform(self, Transform centerOfMassWorldTrans not None):
        """
        Set the transformation for this motion state.  This will probably have
        no meaningful effect on the state of the world containing the object
        this MotionState is associated with.

        XXX If this method is overridden in a subclass, the overridden method
        will not be called by Bullet.
        """
        self.thisptr.setWorldTransform(centerOfMassWorldTrans.thisptr[0])



cdef class DefaultMotionState(MotionState):
    """
    A DefaultMotionState is a MotionState which keeps track of the last
    transformation provided to it.

    A DefaultMotionState can be used with each RigidBody so that the position
    can be queried after a simulation step.

    This class is a wrapper around btDefaultMotionState.  Like MotionState, this
    wrapping doesn't really make sense and will probably be changed.
    """
    def __cinit__(self):
        self.thisptr = new btDefaultMotionState()



cdef class RigidBody(CollisionObject):
    """
    A RigidBody is an object which can be added to a DynamicsWorld and involved
    in dynamics (ie, it can move because of physics).

    Like a CollisionObject, a RigidBody has a CollisionShape.  Additionally, it
    may also have a MotionState to keep track of its movements.  It also has a
    mass which will influence in the usual way how much acceleration it
    experiences due to forces acting on it.

    This class is a wrapper around btRigidBody.
    """
    cdef MotionState motion
    cdef CollisionShape shape

    def __init__(self,
                 MotionState motion = None,
                 CollisionShape shape = None,
                 btScalar mass = 0.0):
        if motion is None:
            motion = DefaultMotionState()
        if shape is None:
            shape = BoxShape(Vector3(0.5, 0.5, 0.5))

        self.motion = motion
        self.shape = shape

        cdef btVector3 inertia
        shape.thisptr.calculateLocalInertia(mass, inertia)
        cdef btRigidBodyConstructionInfo* info
        info = new btRigidBodyConstructionInfo(
            mass, self.motion.thisptr, self.shape.thisptr, inertia)
        self.thisptr = new btRigidBody(info[0])
        del info


    def isInWorld(self):
        """
        Return a boolean indicating whether or not this RigidBody has been added
        to a CollisionWorld.
        """
        cdef btRigidBody *body
        body = <btRigidBody*>self.thisptr
        return body.isInWorld()


    def getMotionState(self):
        """
        Return the MotionState associated with this RigidBody.
        """
        return self.motion


    def setAngularFactor(self, btScalar angularFactor):
        """
        Specify whether this object will be allowed to rotate or not.  If the
        given angularFactor argument is 0, rotation will not be allowed.  If it
        is 1, rotation will be allowed.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        body.setAngularFactor(angularFactor)


    def setLinearVelocity(self, Vector3 v not None):
        """
        Change the linear velocity of this RigidBody for at least a single
        simulation step.  It is unspecified whether the change will persist for
        more than one simulation step.  For reliable and reproducable results,
        you must set the linear velocity before each simulation tick.  This is
        best done in the physics tick callback.

        XXX The physics tick callback is presently unexposed.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 vel = btVector3(v.x, v.y, v.z)
        body.setLinearVelocity(vel)


    def applyCentralForce(self, Vector3 f not None):
        """
        Apply a force to the center of mass of this RigidBody.  The resulting
        acceleration is computed in the usual way, dividing the force by the
        mass of the RigidBody.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 force = btVector3(f.x, f.y, f.z)
        body.applyCentralForce(force)


    def applyForce(self, Vector3 f not None, Vector3 relativePosition not None):
        """
        Apply a force to this RigidBody at a specified offset from its center of
        mass.  The resulting acceleration is computed in the usual way, dividing
        the force by the mass of the RigidBody.  A force applied off-center may
        also result in rotation.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 force = btVector3(f.x, f.y, f.z)
        cdef btVector3 pos = btVector3(
            relativePosition.x, relativePosition.y, relativePosition.z)
        body.applyForce(force, pos)


    def applyCentralImpulse(self, Vector3 i not None):
        """
        Apply an impulse to this RigidBody.  The resulting acceleration will be
        equal to the impulse.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 impulse = btVector3(i.x, i.y, i.z)
        body.applyCentralImpulse(impulse)


    def applyImpulse(self, Vector3 i not None, Vector3 relativePosition not None):
        """
        Apply an impulse to this RigidBody at a specified offset from its center
        of mass.  The resulting accleration will be equal to the impulse.  An
        impulse applied off-center may also result in rotation.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 impulse = btVector3(i.x, i.y, i.z)
        cdef btVector3 pos = btVector3(
            relativePosition.x, relativePosition.y, relativePosition.z)
        body.applyImpulse(impulse, pos)



cdef class ActionInterface:
    """
    ActionInterface is a base class for objects in a DynamicsWorld which are not
    purely dynamic.

    This class is a wrapper around btActionInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use a subclass instead.
    """
    cdef btActionInterface *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class CharacterControllerInterface(ActionInterface):
    """
    A CharacterControllerInterface is an ActionInterface which allows walking
    motion to be applied to an object in a DynamicsWorld without physically
    simulating that walking.

    This class is a wrapper around btCharacterControllerInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use a subclass instead.
    """
    def setWalkDirection(self, Vector3 walkDirection):
        """
        Set the movement direction for this CharacterControllerInterface.  The
        magnitude and the direction of the given walkDirection will determine
        how this object moves in the world.

        Calling setWalkDirection will cause a previous call to
        setVelocityForTimeInterval to be disregarded.
        """
        cdef btCharacterControllerInterface *controller
        controller = <btCharacterControllerInterface*>self.thisptr
        controller.setWalkDirection(
            btVector3(walkDirection.x, walkDirection.y, walkDirection.z))


    def setVelocityForTimeInterval(self,
                                   Vector3 velocity not None,
                                   btScalar timeInterval):
        """
        Set the movement direction for this CharacterControllerInterface.  The
        direction of the given velocity will determine the direction of this
        object's movement in the world, while the timeInterval will determine
        the rate of movement.

        Calling setVelocityForTimeInterval will cause a previous call to
        setWalkDirection to be disregarded.
        """
        cdef btCharacterControllerInterface *controller
        controller = <btCharacterControllerInterface*>self.thisptr
        controller.setVelocityForTimeInterval(
            btVector3(velocity.x, velocity.y, velocity.z), timeInterval)



cdef class PairCachingGhostObject(CollisionObject):
     """
     A PairCachingGhostObject is a CollisionObject which keeps track of all the
     objects it is overlapping.

     This class is a wrapper around btPairCachingGhostObject.
     """
     def __init__(self):
         self.thisptr = new btPairCachingGhostObject()



cdef class KinematicCharacterController(CharacterControllerInterface):
    """
    A KinematicCharacterController is a CharacterControllerInterface which
    implements movement according to the standard rules of kinematics.  That is,
    it will move based on a velocity and direction in the usual way, but without
    regard for interactions with other objects in the world.

    This class is a wrapper around btKinematicCharacterController.
    """
    cdef readonly PairCachingGhostObject ghost
    cdef ConvexShape shape

    def __init__(self, ConvexShape shape not None, float stepHeight, int upAxis):
        self.shape = shape
        self.ghost = PairCachingGhostObject()
        self.thisptr = new btKinematicCharacterController(
            <btPairCachingGhostObject*>self.ghost.thisptr,
            <btConvexShape*>self.shape.thisptr, stepHeight, upAxis)


    def warp(self, Vector3 origin not None):
        """
        Change the location of this object to the given point without traversing
        the intermediate distance.
        """
        cdef btKinematicCharacterController *controller
        controller = <btKinematicCharacterController*>self.thisptr
        controller.warp(btVector3(origin.x, origin.y, origin.z))



cdef class CollisionDispatcher:
    """
    A CollisionDispatcher implements pairwise collision detection.  It supports
    convex-convex collision detection and convex-concave collision detection.

    Roughly speaking, this is the only CollisionDispatcher available, so you
    don't have to understand it, just use it.  Or better yet, ignore it and let
    CollisionWorld (or a subclass) create it for you.

    This class is a wrapper around btCollisionDispatcher.

    XXX Try not to instantiate too many of these, each one leaks some memory.
    """
    cdef btCollisionConfiguration *config
    cdef btDispatcher *thisptr

    def __cinit__(self):
        # XXX btDefaultCollisionConfiguration leaks I suppose.
        self.config = new btDefaultCollisionConfiguration()
        self.thisptr = new btCollisionDispatcher(self.config)


    def __dealloc__(self):
        del self.thisptr
        del self.config



cdef class BroadphaseInterface:
    """
    A BroadphaseInterface generates lists of potentially colliding pairs.  It is
    intended to be faster than CollisionDispatcher, but it may include pairs of
    objects which do not actually collide.  It is applied first to narrow the
    list of pairs the CollisionDispatcher will need to examine.

    This class is loosely a wrapper around btBroadphaseInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use one of the subclasses instead.
    """
    cdef btBroadphaseInterface *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class AxisSweep3(BroadphaseInterface):
    """
    An AxisSweep3 is a BroadphaseInterface implemented using the sweep and prune
    algorithm.  AxisSweep3 is a good general purpose BroadphaseInterface,
    particularly for worlds where most objects have little or no motion.
    However, it can only detect collisions within specified fixed bounds.

    This class is a wrapper around btAxisSweep3.
    """
    def __cinit__(self, Vector3 lower, Vector3 upper):
        self.thisptr = new btAxisSweep3(
            btVector3(lower.x, lower.y, lower.z),
            btVector3(upper.x, upper.y, upper.z))



cdef class ConstraintSolver:
    """
    A ConstraintSolver determines what contact forces to apply.

    This class is loosely a wrapper around btConstraintSolver.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use one of the subclasses instead.
    """
    cdef btConstraintSolver *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class SequentialImpulseConstraintSolver(ConstraintSolver):
    """
    A SequentialImpulseConstraintSolver is a ConstraintSolver based on a fast
    SIMD implementation of the Projected Gauss Seidel method.

    This is basically the only ConstraintSolver available, so whether you
    understood that sentence or not, use this class with your DynamicsWorld.  Or
    just ignore ConstraintSolvers entirely and let the DynamicsWorld create one.

    This class is a wrapper around btSequentialImpulseConstraintSolver.
    """
    def __cinit__(self):
        self.thisptr = new btSequentialImpulseConstraintSolver()



cdef class CollisionWorld:
    """
    A CollisionWorld is a container for CollisionObjects which can detect
    collisions between those objects.

    This class is a wrapper around btCollisionWorld.
    """
    cdef btCollisionWorld *thisptr

    cdef _object *dispatcher
    cdef _object *broadphase

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None):
        if dispatcher is None:
            dispatcher = CollisionDispatcher()
        if broadphase is None:
            broadphase = AxisSweep3(Vector3(0, 0, 0), Vector3(10, 10, 10))

        Py_INCREF(dispatcher)
        self.dispatcher = <_object*>dispatcher

        Py_INCREF(broadphase)
        self.broadphase = <_object*>broadphase

        # Allow subclasses to initialize this differently.
        if self.thisptr == NULL:
            self.thisptr = new btCollisionWorld(
                dispatcher.thisptr, broadphase.thisptr, dispatcher.config)


    def __dealloc__(self):
        del self.thisptr
        Py_DECREF(<object>self.dispatcher)
        Py_DECREF(<object>self.broadphase)


    def getNumCollisionObjects(self):
        """
        Return a count of the number of CollisionObjects which are part of this
        CollisionWorld.
        """
        return self.thisptr.getNumCollisionObjects()


    def addCollisionObject(self, CollisionObject collisionObject):
        """
        Add a new CollisionObject to this CollisionWorld.

        If you have a RigidBody to add, you should add it using
        DynamicsWorld.addRigidBody instead.
        """
        if collisionObject.thisptr.getCollisionShape() == NULL:
            raise ValueError(
                "Cannot add CollisionObject without a CollisionShape")
        self.thisptr.addCollisionObject(collisionObject.thisptr, 0, 0)


    def removeCollisionObject(self, CollisionObject collisionObject):
        """
        Remove a CollisionObject from this CollisionWorld.
        """
        self.thisptr.removeCollisionObject(collisionObject.thisptr)



cdef class DynamicsWorld(CollisionWorld):
    """
    A DynamicsWorld is a container for RigidBodies which implements dynamics (ie
    physics) for those bodies.

    For a dynamics world in which simulation time can actually pass, see one of
    the subclasses of this class.

    This class is a wrapper around btDynamicsWorld.
    """
    cdef list _rigidBodies

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None):
        CollisionWorld.__init__(self, dispatcher, broadphase)
        self._rigidBodies = []


    def addRigidBody(self, RigidBody body not None):
        """
        Add a new RigidBody to this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addRigidBody(<btRigidBody*>body.thisptr)
        self._rigidBodies.append(body)


    def removeRigidBody(self, RigidBody body not None):
        """
        Remove a RigidBody from this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        self._rigidBodies.remove(body)
        world.removeRigidBody(<btRigidBody*>body.thisptr)


    def addAction(self, ActionInterface action not None):
        """
        Add a new ActionInterface to this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addAction(<btActionInterface*>action.thisptr)
        self._rigidBodies.append(action)



cdef class DiscreteDynamicsWorld(DynamicsWorld):
    """
    A DiscreteDynamicsWorld is a DynamicsWorld in which time passes in fixed
    increments.

    This class is a wrapper around btDiscreteDynamicsWorld.
    """
    cdef ConstraintSolver solver

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None,
                 ConstraintSolver solver = None):

        if dispatcher is None:
            dispatcher = CollisionDispatcher()
        if solver is None:
            solver = SequentialImpulseConstraintSolver()
        if broadphase is None:
            broadphase = AxisSweep3(Vector3(0, 0, 0), Vector3(10, 10, 10))

        self.solver = solver
        self.thisptr = <btCollisionWorld*>new btDiscreteDynamicsWorld(
            dispatcher.thisptr, broadphase.thisptr,
            solver.thisptr, dispatcher.config)

        DynamicsWorld.__init__(self, dispatcher, broadphase)


    def setGravity(self, Vector3 gravity):
        """
        Set the gravity in this world.
        XXX This belongs on DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.setGravity(btVector3(gravity.x, gravity.y, gravity.z))


    def getGravity(self):
        """
        Get the gravity in this world.
        XXX This belongs on DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        cdef btVector3 gravity = world.getGravity()
        return Vector3(gravity.getX(), gravity.getY(), gravity.getZ())


    def stepSimulation(self,
                       btScalar timeStep,
                       int maxSubSteps = 1,
                       btScalar fixedTimeStep = 1. / 60.):
        """
        Advance time in the simulation.

        timeStep specifies the amount of simulation time which will pass during
        this call.

        maxSubSteps specifies the maximum number of simulation steps which will
        be taken trying to reach the point in the future specified by timeStep.

        fixedTimeStep specifies the size of each simulation step.  Bullet works
        best if this value is always the same.

        Observe that if fixedTimeStep * maxSubSteps < timeStep, simulation time
        is lost and the simulation will not advance as far as you intended it
        to.

        The number of simulation steps taken is returned.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        return world.stepSimulation(timeStep, maxSubSteps, fixedTimeStep)
