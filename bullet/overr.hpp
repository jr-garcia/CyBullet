#ifndef overr_h__
 #define overr_h__

class MyMotionState : public btMotionState
{
protected:

    PyObject *cyFunc;

public:
    MyMotionState(const btTransform &initialPosition);
    virtual ~MyMotionState();
    btTransform mworldTrans;
    void setPyCallback(PyObject *cyFunc);
    virtual void getWorldTransform(btTransform &worldTrans) const;
    void setKinematicPos(btTransform &currentPos);
    virtual void setWorldTransform(const btTransform &worldTrans);
};

#endif  // overr_h__