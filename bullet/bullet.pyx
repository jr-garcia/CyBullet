
cdef extern from "btBulletCollisionCommon.h":
    ctypedef float btScalar

    cdef cppclass btVector3
    cdef cppclass btCollisionShape:
        pass
    cdef cppclass btBoxShape(btCollisionShape):
        btBoxShape(btVector3 boxHalfExtents)

    cdef cppclass btEmptyShape(btCollisionShape):
        btEmptyShape()


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

        void stepSimulation(btScalar, int, btScalar)


    cdef cppclass btDiscreteDynamicsWorld: # (byDynamicsWorld)
        btDiscreteDynamicsWorld(
            btDispatcher*, btBroadphaseInterface*,
            btConstraintSolver*, btCollisionConfiguration*)



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



cdef class EmptyShape(CollisionShape):
    def __cinit__(self):
        self.thisptr = new btEmptyShape()



cdef class BoxShape(CollisionShape):
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBoxShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class CollisionObject:
    cdef btCollisionObject *thisptr

    def __cinit__(self):
        self.thisptr = new btCollisionObject()


    def getCollisionShape(self):
        shape = CollisionShape()
        shape.thisptr = self.thisptr.getCollisionShape()
        return shape


    def setCollisionShape(self, CollisionShape collisionShape):
        self.thisptr.setCollisionShape(collisionShape.thisptr)



cdef class Transform:
    cdef btTransform *thisptr

    def getOrigin(self):
        cdef btVector3 origin = self.thisptr.getOrigin()
        return Vector3(origin.getX(), origin.getY(), origin.getZ())



cdef class MotionState:
    cdef btMotionState *thisptr

    def getWorldTransform(self):
        transform = Transform()
        transform.thisptr = new btTransform()
        self.thisptr.getWorldTransform(transform.thisptr[0])
        return transform


cdef class RigidBody(CollisionObject):
    def __cinit__(self):
        motionState = new btDefaultMotionState()
        collisionShape = new btBoxShape(btVector3(1, 1, 1))
        cdef btVector3 inertia
        inertia = btVector3(0, 3, 0)
        cdef btRigidBodyConstructionInfo* info
        info = new btRigidBodyConstructionInfo(
            3, motionState, collisionShape, inertia)
        self.thisptr = new btRigidBody(info[0])


    def getMotionState(self):
        cdef btRigidBody *body = <btRigidBody*>self.thisptr
        motion = MotionState()
        motion.thisptr = body.getMotionState()
        return motion



cdef class CollisionWorld:
    cdef btCollisionWorld *thisptr

    def __init__(self):
        config = new btDefaultCollisionConfiguration()
        self.thisptr = new btCollisionWorld(
            new btCollisionDispatcher(config),
            new btAxisSweep3(btVector3(0, 0, 0), btVector3(10, 10, 10)),
            config)


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
    def addRigidBody(self, RigidBody body):
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addRigidBody(<btRigidBody*>body.thisptr)


cdef class DiscreteDynamicsWorld(DynamicsWorld):
    def __init__(self):
        config = new btDefaultCollisionConfiguration()
        self.thisptr = <btCollisionWorld*>new btDiscreteDynamicsWorld(
            new btCollisionDispatcher(config),
            new btAxisSweep3(btVector3(0, 0, 0), btVector3(10, 10, 10)),
            new btSequentialImpulseConstraintSolver(),
            config)


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
        world.stepSimulation(timeStep, maxSubSteps, fixedTimeStep)
