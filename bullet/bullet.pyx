# Copyright (c) 2010 Jean-Paul Calderone
# See LICENSE for details.

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

    cdef cppclass btStridingMeshInterface:
        pass

    cdef cppclass btTriangleIndexVertexArray(btStridingMeshInterface):
        btTriangleIndexVertexArray(
            int numTriangles,
            int *triangleIndexBase,
            int triangleIndexStride,
            int numVertices,
            btScalar *vertexBase,
            int vertexStride)

        void addIndexedMesh(btIndexedMesh &mesh, PHY_ScalarType indexeType)

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
    cdef readonly btScalar x
    cdef readonly btScalar y
    cdef readonly btScalar z

    def __cinit__(self, btScalar x, btScalar y, btScalar z):
        self.x = x
        self.y = y
        self.z = z


    def __repr__(self):
        return '<Vector x=%s y=%s z=%s>' % (self.x, self.y, self.z)



cdef class CollisionShape:
    cdef btCollisionShape *thisptr


    def __dealloc__(self):
        del self.thisptr



cdef class ConvexShape(CollisionShape):
    pass



cdef class Box2dShape(ConvexShape):
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBox2dShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class BoxShape(ConvexShape):
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBoxShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class SphereShape(ConvexShape):
    def __cinit__(self, btScalar radius):
        self.thisptr = new btSphereShape(radius)


    def getRadius(self):
        return (<btSphereShape*>self.thisptr).getRadius()



cdef class CapsuleShape(ConvexShape):
    def __cinit__(self, btScalar radius, btScalar height):
        self.thisptr = new btCapsuleShape(radius, height)


cdef class IndexedMesh:
    cdef btIndexedMesh* thisptr

    def __cinit__(self):
        self.thisptr = new btIndexedMesh()
        self.thisptr.m_numTriangles = 0;
        self.thisptr.m_triangleIndexBase = NULL;
        self.thisptr.m_triangleIndexStride = 0;
        self.thisptr.m_numVertices = 0;
        self.thisptr.m_vertexBase = NULL;
        self.thisptr.m_vertexStride = 0;
        self.thisptr.m_indexType = PHY_FLOAT
        self.thisptr.m_vertexType = PHY_FLOAT


    def setIndices(self, int numTriangles, int indexStride,
                   numpy.ndarray indexBase not None):
        cdef char *dname = indexBase.dtype.char
        cdef char dtype = dname[0]

        cdef PHY_ScalarType indexType

        if dtype == 'f':
            indexType = PHY_FLOAT
        elif dtype == 'd':
            indexType = PHY_DOUBLE
        elif dtype == 'i':
            indexType = PHY_INTEGER
        elif dtype == 'h':
            indexType = PHY_SHORT
        else:
            raise ValueError("Unsupported index array type")

        self.thisptr.m_numTriangles = numTriangles
        self.thisptr.m_triangleIndexStride = indexStride
        self.thisptr.m_triangleIndexBase = indexBase
        self.thisptr.m_indexType = indexType


    def __dealloc__(self):
        del self.thisptr



cdef class TriangleIndexVertexArray(object):
    pass


cdef class BvhTriangleMeshShape(ConvexShape):
    cdef btStridingMeshInterface *stride
    cdef numpy.ndarray triangles
    cdef numpy.ndarray vertices

    def __init__(self,
                 numpy.ndarray[numpy.int32_t] triangles not None,
                 numpy.ndarray[numpy.float32_t] vertices not None):

        self.triangles = triangles
        self.vertices = vertices

        self.stride = new btTriangleIndexVertexArray(
            len(triangles) / 3, <int*>triangles.data, 3,
            len(triangles) / 3, <btScalar*>vertices.data, 3)
        self.thisptr = new btBvhTriangleMeshShape(self.stride, True, True)


    def __dealloc__(self):
        del self.stride



cdef class Transform:
    cdef btTransform *thisptr

    def __cinit__(self):
        self.thisptr = new btTransform()
        self.thisptr.setIdentity()


    def __dealloc__(self):
        del self.thisptr


    def getOrigin(self):
        cdef btVector3 origin = self.thisptr.getOrigin()
        return Vector3(origin.getX(), origin.getY(), origin.getZ())


    def setOrigin(self, Vector3 origin not None):
        self.thisptr.setOrigin(btVector3(origin.x, origin.y, origin.z))


    def setIdentity(self):
        self.thisptr.setIdentity()



cdef class CollisionObject:
    cdef btCollisionObject *thisptr
    cdef CollisionShape _shape

    def __init__(self):
        self.thisptr = new btCollisionObject()


    def __dealloc__(self):
        del self.thisptr


    def setRestitution(self, btScalar restitution):
        self.thisptr.setRestitution(restitution)


    def getRestitution(self):
        return self.thisptr.getRestitution()


    def getCollisionShape(self):
        return self._shape


    def setCollisionShape(self, CollisionShape collisionShape):
        self.thisptr.setCollisionShape(collisionShape.thisptr)
        self._shape = collisionShape


    def getWorldTransform(self):
        cdef Transform transform = Transform()
        transform.thisptr[0] = self.thisptr.getWorldTransform()
        return transform


    def setWorldTransform(self, Transform transform not None):
        self.thisptr.setWorldTransform(transform.thisptr[0])



cdef class MotionState:
    cdef btMotionState *thisptr

    def __dealloc__(self):
        del self.thisptr


    def getWorldTransform(self):
        transform = Transform()
        self.thisptr.getWorldTransform(transform.thisptr[0])
        return transform


    def setWorldTransform(self, Transform centerOfMassWorldTrans not None):
        self.thisptr.setWorldTransform(centerOfMassWorldTrans.thisptr[0])



cdef class DefaultMotionState(MotionState):
    def __cinit__(self):
        self.thisptr = new btDefaultMotionState()



cdef class RigidBody(CollisionObject):
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
        cdef btRigidBody *body
        body = <btRigidBody*>self.thisptr
        return body.isInWorld()


    def getMotionState(self):
        return self.motion


    def setAngularFactor(self, btScalar angularFactor):
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        body.setAngularFactor(angularFactor)


    def setLinearVelocity(self, Vector3 v not None):
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 vel = btVector3(v.x, v.y, v.z)
        body.setLinearVelocity(vel)



cdef class ActionInterface:
    cdef btActionInterface *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class CharacterControllerInterface(ActionInterface):
    def setWalkDirection(self, Vector3 walkDirection):
        cdef btCharacterControllerInterface *controller
        controller = <btCharacterControllerInterface*>self.thisptr
        controller.setWalkDirection(
            btVector3(walkDirection.x, walkDirection.y, walkDirection.z))


    def setVelocityForTimeInterval(self,
                                   Vector3 velocity not None,
                                   btScalar timeInterval):
        cdef btCharacterControllerInterface *controller
        controller = <btCharacterControllerInterface*>self.thisptr
        controller.setVelocityForTimeInterval(
            btVector3(velocity.x, velocity.y, velocity.z), timeInterval)



cdef class PairCachingGhostObject(CollisionObject):
     def __init__(self):
         self.thisptr = new btPairCachingGhostObject()



cdef class KinematicCharacterController(CharacterControllerInterface):
    cdef readonly PairCachingGhostObject ghost
    cdef ConvexShape shape

    def __init__(self, ConvexShape shape not None, float stepHeight, int upAxis):
        self.shape = shape
        self.ghost = PairCachingGhostObject()
        self.thisptr = new btKinematicCharacterController(
            <btPairCachingGhostObject*>self.ghost.thisptr,
            <btConvexShape*>self.shape.thisptr, stepHeight, upAxis)


    def warp(self, Vector3 origin not None):
        cdef btKinematicCharacterController *controller
        controller = <btKinematicCharacterController*>self.thisptr
        controller.warp(btVector3(origin.x, origin.y, origin.z))



cdef class CollisionDispatcher:
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
    cdef btBroadphaseInterface *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class AxisSweep3(BroadphaseInterface):
    def __cinit__(self, Vector3 lower, Vector3 upper):
        self.thisptr = new btAxisSweep3(
            btVector3(lower.x, lower.y, lower.z),
            btVector3(upper.x, upper.y, upper.z))



cdef class ConstraintSolver:
    cdef btConstraintSolver *thisptr

    def __dealloc__(self):
        del self.thisptr




cdef class SequentialImpulseConstraintSolver(ConstraintSolver):
    def __cinit__(self):
        self.thisptr = new btSequentialImpulseConstraintSolver()



cdef class CollisionWorld:
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
        return self.thisptr.getNumCollisionObjects()


    def addCollisionObject(self, CollisionObject collisionObject):
        if collisionObject.thisptr.getCollisionShape() == NULL:
            raise ValueError(
                "Cannot add CollisionObject without a CollisionShape")
        self.thisptr.addCollisionObject(collisionObject.thisptr, 0, 0)


    def removeCollisionObject(self, CollisionObject collisionObject):
        self.thisptr.removeCollisionObject(collisionObject.thisptr)



cdef class DynamicsWorld(CollisionWorld):
    cdef list _rigidBodies

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None):
        CollisionWorld.__init__(self, dispatcher, broadphase)
        self._rigidBodies = []


    def addRigidBody(self, RigidBody body not None):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addRigidBody(<btRigidBody*>body.thisptr)
        self._rigidBodies.append(body)


    def removeRigidBody(self, RigidBody body not None):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        self._rigidBodies.remove(body)
        world.removeRigidBody(<btRigidBody*>body.thisptr)


    def addAction(self, ActionInterface action not None):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addAction(<btActionInterface*>action.thisptr)
        self._rigidBodies.append(action)



cdef class DiscreteDynamicsWorld(DynamicsWorld):
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
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.setGravity(btVector3(gravity.x, gravity.y, gravity.z))


    def getGravity(self):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        cdef btVector3 gravity = world.getGravity()
        return Vector3(gravity.getX(), gravity.getY(), gravity.getZ())


    def stepSimulation(self,
                       btScalar timeStep,
                       int maxSubSteps = 1,
                       btScalar fixedTimeStep = 1. / 60.):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        return world.stepSimulation(timeStep, maxSubSteps, fixedTimeStep)
