# Copyright (c) Jean-Paul Calderone
# See LICENSE for details.

"""
CyBullet wraps the Bullet Physics library for Python.

"""


from libc.stdint cimport uintptr_t
from libcpp cimport bool

cimport numpy

cdef extern from "Python.h":
    cdef void Py_INCREF( object )
    cdef void Py_DECREF( object )

    cdef struct _object:
        pass

    ctypedef _object PyObject


# Cython template arguments can't be literal pointers
ctypedef btCollisionObject *btCollisionObjectP

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

    cdef int _ACTIVE_TAG "ACTIVE_TAG"
    cdef int _ISLAND_SLEEPING "ISLAND_SLEEPING"
    cdef int _WANTS_DEACTIVATION "WANTS_DEACTIVATION"
    cdef int _DISABLE_DEACTIVATION "DISABLE_DEACTIVATION"
    cdef int _DISABLE_SIMULATION "DISABLE_SIMULATION"

    cdef cppclass btVector3:
        btVector3()
        btVector3(const btScalar &_x, const btScalar &_y, const btScalar &_z)
        void setInterpolate3(const btVector3 &v0, const btVector3 &v1, btScalar rt)
        btVector3 normalized() const
        btVector3 &normalize()
        btVector3 cross (const btVector3 &v) const
        const btScalar &getX() const
        const btScalar &getY() const
        const btScalar &getZ() const
        btScalar dot(const btVector3 &v) const
        btScalar length() const

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


    cdef cppclass btMatrix3x3:
        btVector3 operator*(btVector3)


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

        void buildOptimizedBvh()


    cdef int _DefaultFilter "btBroadphaseProxy::DefaultFilter"
    cdef int _AllFilter "btBroadphaseProxy::AllFilter"

    cdef cppclass btBroadphaseProxy:
        short int m_collisionFilterGroup
        short int m_collisionFilterMask



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
        btMatrix3x3 getBasis()


    cdef cppclass btMotionState:
        void getWorldTransform(btTransform &transform)
        void setWorldTransform(btTransform &transform)

    cdef cppclass btDefaultMotionState(btMotionState):
        btDefaultMotionState()


    cdef cppclass btAlignedObjectArray[T]:
        int size()
        T at(int)


    cdef int _CF_STATIC_OBJECT "btCollisionObject::CF_STATIC_OBJECT"
    cdef int _CF_KINEMATIC_OBJECT "btCollisionObject::CF_KINEMATIC_OBJECT"
    cdef int _CF_NO_CONTACT_RESPONSE "btCollisionObject::CF_NO_CONTACT_RESPONSE"
    cdef int _CF_CUSTOM_MATERIAL_CALLBACK "btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK"
    cdef int _CF_CHARACTER_OBJECT "btCollisionObject::CF_CHARACTER_OBJECT"
    cdef int _CF_DISABLE_VISUALIZE_OBJECT "btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT"
    cdef int _CF_DISABLE_SPU_COLLISION_PROCESSING "btCollisionObject::CF_DISABLE_SPU_COLLISION_PROCESSING"


    cdef cppclass btCollisionObject:

        btCollisionObject()

        btCollisionShape* getCollisionShape()
        void setCollisionShape(btCollisionShape*)

        btScalar getFriction()
        void setFriction(btScalar)

        void setRestitution(btScalar)
        btScalar getRestitution()

        btTransform& getWorldTransform()
        void setWorldTransform(btTransform& worldTrans)

        int getActivationState()
        void setActivationState(int newState)

        btBroadphaseProxy *getBroadphaseHandle()

        void *getUserPointer()
        void setUserPointer(void *userPointer)

        void forceActivationState (int newState) const
        void activate(bool forceActivation) const
        int getCollisionFlags () const
        void setCollisionFlags (int flags)

    cdef cppclass btRigidBody(btCollisionObject)


    cdef cppclass btActionInterface:
        pass


    cdef cppclass btCharacterControllerInterface(btActionInterface):
        void setWalkDirection(btVector3 walkDirection)

        void setVelocityForTimeInterval(
            btVector3 velocity, btScalar timeInterval)



cdef extern from "BulletCollision/CollisionShapes/btCylinderShape.h":
    cdef cppclass btCylinderShape(btConvexShape):
        btCylinderShape()
        btCylinderShape(btVector3&)
        btVector3& getHalfExtentsWithoutMargin()
        btScalar getRadius()

    cdef cppclass btCylinderShapeX(btCylinderShape):
        btCylinderShapeX(btVector3&)

    cdef cppclass btCylinderShapeZ(btCylinderShape):
        btCylinderShapeZ(btVector3&)



cdef extern from "BulletCollision/CollisionShapes/btStaticPlaneShape.h":
    cdef cppclass btStaticPlaneShape(btCollisionShape):
        btStaticPlaneShape(btVector3 &planeNormal, btScalar planeConstant)



cdef extern from "btBulletCollisionCommon.h":
    cdef cppclass btDispatcher:
        pass



cdef extern from "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h":
    cdef cppclass btOverlappingPairCallback:
        pass


    cdef cppclass btGhostPairCallback(btOverlappingPairCallback):
        pass


    cdef cppclass btOverlappingPairCache:
        void setInternalGhostPairCallback(btOverlappingPairCallback*)


    cdef cppclass btHashedOverlappingPairCache(btOverlappingPairCache):
        pass


cdef extern from "BulletCollision/CollisionDispatch/btGhostObject.h":
    cdef cppclass btPairCachingGhostObject(btCollisionObject):
        pass


cdef extern from "BulletDynamics/Character/btKinematicCharacterController.h":
    cdef cppclass btKinematicCharacterController(btCharacterControllerInterface):
        btKinematicCharacterController(
            btPairCachingGhostObject *ghostObject,
            btConvexShape *convexShape,
            btScalar stepHeight, const btVector3& up)

        void setGravity(const btVector3& gravity);
        btVector3 getGravity() const;

        void warp(btVector3 origin)



cdef extern from "btBulletCollisionCommon.h" namespace "btRigidBody":
    cdef cppclass btRigidBodyConstructionInfo:
        btRigidBodyConstructionInfo(
            btScalar mass,
            btMotionState *motionState,
            btCollisionShape *collisionShape)
        btRigidBodyConstructionInfo(
            btScalar mass,
            btMotionState *motionState,
            btCollisionShape *collisionShape,
            btVector3 localInteria)

        btScalar m_additionalAngularDampingFactor
        btScalar m_additionalAngularDampingThresholdSqr
        bool m_additionalDamping
        btScalar m_additionalDampingFactor
        btScalar m_additionalLinearDampingThresholdSqr
        btScalar m_angularDamping
        btScalar m_angularSleepingThreshold
        btCollisionShape* m_collisionShape
        btScalar m_friction
        btScalar m_linearDamping
        btScalar m_linearSleepingThreshold
        btVector3 m_localInertia
        btScalar m_mass
        btMotionState* m_motionState
        btScalar m_restitution
        btTransform m_startWorldTransform



cdef extern from "LinearMath/btIDebugDraw.h":

    cdef cppclass btIDebugDraw:
        pass

cdef extern from "LinearMath/btIDebugDraw.h" namespace "btIDebugDraw":
        cdef enum DebugDrawModes:
            DBG_NoDebug
            DBG_DrawWireframe
            DBG_DrawAabb
            DBG_DrawFeaturesText
            DBG_DrawContactPoints
            DBG_DrawText
            DBG_DrawConstraints
            DBG_DrawConstraintLimits


NO_DEBUG = DBG_NoDebug
DRAW_WIREFRAME = DBG_DrawWireframe
DRAW_AABB = DBG_DrawAabb
DRAW_FEATURES_TEXT = DBG_DrawFeaturesText
DRAW_CONTACT_POINTS = DBG_DrawContactPoints
DRAW_TEXT = DBG_DrawText
DRAW_CONSTRAINTS = DBG_DrawConstraints
DRAW_CONSTRAINT_LIMITS = DBG_DrawConstraintLimits
    
cdef extern from "btBulletCollisionCommon.h" namespace "btCollisionWorld":
    cdef cppclass LocalShapeInfo:
        int m_shapePart
        int m_triangleIndex
        
    cdef cppclass LocalRayResult:
        LocalRayResult (const btCollisionObject *collisionObject, LocalShapeInfo *localShapeInfo,
                        const btVector3 &hitNormalLocal, btScalar hitFraction)
        const btCollisionObject *m_collisionObject
        LocalShapeInfo *m_localShapeInfo
        btVector3 m_hitNormalLocal
        btScalar m_hitFraction
            
    cdef cppclass RayResultCallback:
        bool hasHit () const
        
        RayResultCallback ()
        
        bool needsCollision (btBroadphaseProxy *proxy0) const
        
        btScalar addSingleResult (LocalRayResult &rayResult, bool normalInWorldSpace)
        
        btScalar m_closestHitFraction
        
        const btCollisionObject *m_collisionObject
        
        short int m_collisionFilterGroup
        
        short int m_collisionFilterMask
        
        unsigned int m_flags

    # cdef cppclass ClosestRayResultCallback(RayResultCallback):
    #     ClosestRayResultCallback (const btVector3 &rayFromWorld, const btVector3 &rayToWorld)
    #     btScalar addSingleResult (LocalRayResult &rayResult, bool normalInWorldSpace)
    #     btVector3 m_rayFromWorld
    #     btVector3 m_rayToWorld
    #     btVector3 m_hitNormalWorld
    #     btVector3 m_hitPointWorld

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

        btVector3& normalize()
        btVector3 cross(btVector3&)
        btScalar dot(btVector3&)
        btScalar length()


        # btVector3& operator+=(btScalar&)
        # btVector3& operator*=(btScalar&)


    cdef cppclass btQuaternion:
        btQuaternion()
        btQuaternion(btScalar x, btScalar y, btScalar z, btScalar w)
        btQuaternion(btVector3 axis, btScalar angle)

        btScalar getX()
        btScalar getY()
        btScalar getZ()
        btScalar getW()

        btVector3 getAxis()
        btScalar getAngle()

        btQuaternion operator* (btQuaternion)


    cdef cppclass btBroadphaseInterface:
        btOverlappingPairCache* getOverlappingPairCache()


    cdef cppclass btAxisSweep3(btBroadphaseInterface):
        btAxisSweep3(btVector3, btVector3, unsigned short int maxHandles,
                     btOverlappingPairCache *pairCache,
                     bool disableRaycastAccelerator)


    cdef cppclass btDbvtBroadphase(btBroadphaseInterface):
        pass


    cdef cppclass btRigidBody(btCollisionObject):
        btRigidBody(btRigidBodyConstructionInfo)

        bool isInWorld()

        btScalar getInvMass()
        btVector3& getInvInertiaDiagLocal()

        btMotionState* getMotionState()

        void setAngularFactor(btScalar angFac)
        btScalar getAngularDamping()
        btScalar getAngularSleepingThreshold()

        void setLinearVelocity(btVector3 velocity)
        btVector3& getLinearVelocity()
        btScalar getLinearDamping()
        btScalar getLinearSleepingThreshold()
        void setSleepingThresholds(btScalar linear, btScalar angular)
        btScalar m_linearSleepingThreshold
        btScalar m_angularSleepingThreshold

        void applyCentralForce(btVector3 force)
        void applyForce(btVector3 force, btVector3 relativePosition)

        void applyCentralImpulse(btVector3 impulse)
        void applyImpulse(btVector3 impulse, btVector3 relativePosition)

        void setGravity(const btVector3 &acceleration)
        void setMassProps(btScalar mass, btVector3 inertia)
        void updateInertiaTensor()
        void getAabb (btVector3 &aabbMin, btVector3 &aabbMax) const
        void setMotionState (btMotionState *motionState)
        void translate (const btVector3 &v)
        const btTransform & getCenterOfMassTransform () const
        const btVector3 & getAngularVelocity () const
        void setAngularVelocity (const btVector3 &ang_vel)

    

    cdef cppclass btCollisionWorld:
        btCollisionWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)

        void setDebugDrawer(btIDebugDraw *debugDrawer)
        void debugDrawWorld()

        btDispatcher *getDispatcher()
        btBroadphaseInterface *getBroadphase()

        int getNumCollisionObjects()

        void addCollisionObject(btCollisionObject*, short int, short int)
        void removeCollisionObject(btCollisionObject*)

        btAlignedObjectArray[btCollisionObjectP]& getCollisionObjectArray()        

        void rayTest (const btVector3 &rayFromWorld, const btVector3 &rayToWorld, RayResultCallback &resultCallback) const

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
        void removeAction(btActionInterface*)

        int stepSimulation(btScalar, int, btScalar)


    cdef cppclass btDiscreteDynamicsWorld: # (btDynamicsWorld)
        btDiscreteDynamicsWorld(
            btDispatcher*, btBroadphaseInterface*,
            btConstraintSolver*, btCollisionConfiguration*)

        void addRigidBody(btRigidBody*, short, short)

        btConstraintSolver *getConstraintSolver()


cdef extern from "bulletdebugdraw.h":
    cdef cppclass PythonDebugDraw(btIDebugDraw):
        PythonDebugDraw(PyObject *debugDraw)


cdef extern from "overr.hpp":
    cdef cppclass MyMotionState(btMotionState):
        # PyObject * cyFunc
        MyMotionState(const btTransform &initialPosition)
        btTransform mworldTrans
        void setPyCallback(cyFunc)
        void getWorldTransform(btTransform &worldTrans)
        void setKinematicPos(btTransform &currentPos)
        void setWorldTransform(const btTransform &worldTrans)