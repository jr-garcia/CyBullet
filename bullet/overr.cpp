#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "Python.h"
#include "./overr.hpp"

#include <iostream>

using namespace std;

    MyMotionState::MyMotionState(const btTransform &initialPosition)
    {
	    mworldTrans = initialPosition;
    }

    MyMotionState::~MyMotionState()
    {
        Py_DECREF(this->cyFunc);
        this->cyFunc = NULL;
        cout << "destroyed\n";
    }

    void MyMotionState::setPyCallback(PyObject *cyFunc)
    {
        Py_INCREF(cyFunc);
        this->cyFunc = cyFunc;
    }

    void MyMotionState::getWorldTransform(btTransform &worldTrans) const
    {
        worldTrans = mworldTrans;
//        printf("get world overr %4.2f,%4.2f,%4.2f\n", worldTrans.getOrigin().x(), worldTrans.getOrigin().y(), worldTrans
//        .getOrigin().z());
    }

    void MyMotionState::setKinematicPos(btTransform &currentPos)
    {
//        printf("set kinematic overr %4.2f,%4.2f,%4.2f\n", currentPos.getOrigin().x(), currentPos.getOrigin().y(), currentPos
//        .getOrigin().z());
        mworldTrans = currentPos;
    }

    void MyMotionState::setWorldTransform(const btTransform &worldTrans)
    {
        mworldTrans = worldTrans;
//        this->cyFunc(pyob, worldTrans);
        if(cyFunc == NULL)
            return; // silently return

//        PyObject *argsP;
//        PyObject *argsR;
        PyObject *result;

//        PyObject *strm = PyString_FromString("_stwL");
//        PyObject *MList = PyList_New(0);

//        btVector3 pos = worldTrans.getOrigin();
//        argsP = Py_BuildValue("(ddd)", pos.x(), pos.y(), pos.z());
//
//        btQuaternion rot = worldTrans.getRotation();
//        argsR = Py_BuildValue("(dddd)", rot.getX(), rot.getY(), rot.getZ(), rot.getW());
//
//        PyList_Append(MList, argsP);
//        PyList_Append(MList, argsR);

        try
        {
//          result = PyObject_CallMethodObjArgs(pyob, strm, NULL);
            result = PyObject_CallFunctionObjArgs(cyFunc, NULL);
        }
        catch(...)
        {
        cout << "exeption.\n";
        }
        // todo: raise / cout if result == 0?

        Py_DECREF(result);
//        Py_DECREF(argsP);
//        Py_DECREF(argsR);
//        Py_DECREF(MList);
//        Py_DECREF(strm);
    }