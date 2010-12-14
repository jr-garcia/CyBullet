
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
        void removeCollisionObject(btCollisionObject*)



# Forward declare some things because of circularity in the API.
cdef class CollisionObject


cdef class Vector3:
    cdef btScalar x
    cdef btScalar y
    cdef btScalar z

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


    def removeCollisionObject(self, CollisionObject collisionObject):
        self.thisptr.removeCollisionObject(collisionObject.thisptr)
