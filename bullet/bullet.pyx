
cdef extern from "btBulletCollisionCommon.h":
    ctypedef float btScalar

    cdef cppclass btVector3
    cdef cppclass btCollisionShape:
        pass
    cdef cppclass btBoxShape(btCollisionShape):
        btBoxShape(btVector3 boxHalfExtents)



cdef extern from "btBulletDynamicsCommon.h":
    cdef cppclass btMotionState:
        pass

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

    cdef cppclass btBroadphaseInterface:
        pass

    cdef cppclass btAxisSweep3(btBroadphaseInterface):
        btAxisSweep3(btVector3, btVector3)

    cdef cppclass btCollisionShape:
        btCollisionShape()

    cdef cppclass btRigidBody(btCollisionObject):
        btRigidBody(btRigidBodyConstructionInfo)

    cdef cppclass btCollisionWorld:
        btCollisionWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)

        int getNumCollisionObjects()

        void addCollisionObject(btCollisionObject*, short int, short int)



# Forward declare some things because of circularity in the API.
cdef class CollisionObject
cdef class RigidBody(CollisionObject)



cdef class CollisionShape:
    cdef btCollisionShape *thisptr

    def __cinit__(self, CollisionObject collisionObject):
        shape = collisionObject.thisptr.getCollisionShape()
        if shape == NULL:
            raise ValueError(
                "Cannot construct CollisionShape from CollisionObject with no "
                "shape.")
        self.thisptr = shape



cdef class CollisionObject:
    cdef btCollisionObject *thisptr

    def getCollisionShape(self):
        return CollisionShape(self)


    def setCollisionShape(self, CollisionShape collisionShape):
        self.thisptr.setCollisionShape(collisionShape.thisptr)



cdef class RigidBody(CollisionObject):
    def __cinit__(self):
        motionState = new btDefaultMotionState()
        collisionShape = new btBoxShape(btVector3(1, 1, 1))
        cdef btVector3 inertia
        inertia = btVector3(0, 0, 0)
        cdef btRigidBodyConstructionInfo* info
        info = new btRigidBodyConstructionInfo(
            3, motionState, collisionShape, inertia)
        self.thisptr = new btRigidBody(info[0])



cdef class CollisionWorld:
    cdef btCollisionWorld *thisptr

    def __cinit__(self):
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
