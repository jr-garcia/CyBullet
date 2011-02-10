
#include "LinearMath/btIDebugDraw.h"

#include "Python.h"

class PythonDebugDraw : public btIDebugDraw {
public:
    PyObject *debugDraw;

    PythonDebugDraw(PyObject *debugDraw) {
        Py_INCREF(debugDraw);
        this->debugDraw = debugDraw;
    }

    virtual ~PythonDebugDraw() {
        Py_DECREF(this->debugDraw);
        this->debugDraw = NULL;
    }

    virtual void drawLine(const btVector3& from,
                          const btVector3& to,
                          const btVector3& color) {
        char method[] = "drawLine";
        char format[] = "(ddddddddd)";
        PyObject_CallMethod(
            this->debugDraw, &method[0], &format[0],
            from.getX(), from.getY(), from.getZ(),
            to.getX(), to.getY(), to.getZ(),
            color.getX(), color.getY(), color.getZ());
    }

    virtual void drawContactPoint(const btVector3&, const btVector3&, btScalar, int, const btVector3&) {
    }

    virtual void reportErrorWarning(const char*) {
    }

    virtual void draw3dText(const btVector3&, const char*) {
    }

    virtual void setDebugMode(int mode) {
    }

    virtual int getDebugMode() const {
        return DBG_DrawWireframe | DBG_DrawAabb | DBG_DrawContactPoints;
    }
};
