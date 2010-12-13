
cdef extern from "btBulletCollisionCommon.h":

    ctypedef float btScalar

    cdef cppclass btCollisionConfiguration:
        pass

    cdef cppclass btDefaultCollisionConfiguration(btCollisionConfiguration):
        pass

    cdef cppclass btDispatcher:
        pass

    cdef cppclass btCollisionDispatcher(btDispatcher):
        btCollisionDispatcher(btCollisionConfiguration*)

    cdef cppclass btVector3:
        btVector3(btScalar, btScalar, btScalar)

    cdef cppclass btBroadphaseInterface:
        pass

    cdef cppclass btAxisSweep3(btBroadphaseInterface):
        btAxisSweep3(btVector3, btVector3)

    cdef cppclass btCollisionWorld:
        btCollisionWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)



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

