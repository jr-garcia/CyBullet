
cdef extern from "btBulletCollisionCommon.h":
    ctypedef float btScalar

    cdef cppclass btVector3
    cdef cppclass btCollisionShape:
        pass

    cdef cppclass btBoxShape(btCollisionShape):
        btBoxShape(btVector3 boxHalfExtents)

    cdef cppclass btEmptyShape(btCollisionShape):
        btEmptyShape()


cdef extern from "BulletCollision/CollisionShapes/btBox2dShape.h":
    cdef cppclass btBox2dShape(btCollisionShape):
        btBox2dShape(btVector3 boxHalfExtents)


cdef extern from "btBulletDynamicsCommon.h":
    cdef cppclass btTransform:
        btVector3 getOrigin()


    cdef cppclass btMotionState:
        void getWorldTransform(btTransform transform)


    cdef cppclass btDefaultMotionState(btMotionState):
        btDefaultMotionState()

    cdef cppclass btCollisionObject:
        btCollisionObject()

        btCollisionShape* getCollisionShape()

        void setCollisionShape(btCollisionShape*)

    cdef cppclass btRigidBody(btCollisionObject)



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

        btScalar getX()
        btScalar getY()
        btScalar getZ()


    cdef cppclass btBroadphaseInterface:
        pass

    cdef cppclass btAxisSweep3(btBroadphaseInterface):
        btAxisSweep3(btVector3, btVector3)

    cdef cppclass btCollisionShape:
        btCollisionShape()

    cdef cppclass btRigidBody(btCollisionObject):
        btRigidBody(btRigidBodyConstructionInfo)

        btMotionState* getMotionState()

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



cdef class CollisionShape:
    cdef btCollisionShape *thisptr


    def __dealloc__(self):
        del self.thisptr



cdef class EmptyShape(CollisionShape):
    def __cinit__(self):
        self.thisptr = new btEmptyShape()



cdef class Box2dShape(CollisionShape):
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBox2dShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class BoxShape(CollisionShape):
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBoxShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class CollisionObject:
    cdef btCollisionObject *thisptr

    def __init__(self):
        self.thisptr = new btCollisionObject()


    def getCollisionShape(self):
        shape = CollisionShape()
        shape.thisptr = self.thisptr.getCollisionShape()
        return shape


    def setCollisionShape(self, CollisionShape collisionShape):
        self.thisptr.setCollisionShape(collisionShape.thisptr)



cdef class Transform:
    cdef btTransform *thisptr

    def __cinit__(self):
        self.thisptr = new btTransform()


    def __dealloc__(self):
        del self.thisptr


    def getOrigin(self):
        cdef btVector3 origin = self.thisptr.getOrigin()
        return Vector3(origin.getX(), origin.getY(), origin.getZ())



cdef class MotionState:
    cdef btMotionState *thisptr
    cdef Transform transform

    def __dealloc__(self):
        del self.thisptr


    def getWorldTransform(self):
        transform = Transform()
        self.thisptr.getWorldTransform(transform.thisptr[0])
        return transform



cdef class DefaultMotionState(MotionState):
    def __cinit__(self):
        self.thisptr = new btDefaultMotionState()



cdef class RigidBody(CollisionObject):
    cdef MotionState motion
    cdef CollisionShape shape

    def __init__(self):
        self.motion = DefaultMotionState()
        self.shape = BoxShape(Vector3(1, 1, 1))
        cdef btVector3 inertia
        inertia = btVector3(0, 0, 0)
        cdef btRigidBodyConstructionInfo* info
        info = new btRigidBodyConstructionInfo(
            3, self.motion.thisptr, self.shape.thisptr, inertia)
        self.thisptr = new btRigidBody(info[0])
        del info


    def __dealloc__(self):
        del self.thisptr


    def getMotionState(self):
        return self.motion



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
    cdef CollisionDispatcher dispatcher
    cdef BroadphaseInterface broadphase

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None):
        if dispatcher is None:
            dispatcher = CollisionDispatcher()
        if broadphase is None:
            broadphase = AxisSweep3(Vector3(0, 0, 0), Vector3(10, 10, 10))

        self.dispatcher = dispatcher
        self.broadphase = broadphase

        # Allow subclasses to initialize this differently.
        if self.thisptr == NULL:
            self.thisptr = new btCollisionWorld(
                dispatcher.thisptr, broadphase.thisptr, dispatcher.config)


    def __dealloc__(self):
        del self.thisptr


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


    def addRigidBody(self, RigidBody body):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addRigidBody(<btRigidBody*>body.thisptr)
        self._rigidBodies.append(body)



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
