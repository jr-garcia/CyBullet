import sys

import numpy

import bullet

def memused():
    return [
        int(line.split(None)[1])
        for line in file('/proc/self/status')
        if line.startswith('VmSize')][0]



def measure(n, params):
    type, args = params[0], params[1:]
    type(*args)
    print type.__name__, memused(),
    sys.stdout.flush()
    for i in n:
        type(*args)
    print memused()



def main():
    n = xrange(2 * 10 ** 3)
    for type in [
        (bullet.DefaultMotionState,),
        (bullet.BoxShape, bullet.Vector3(1, 2, 3)),
        (bullet.BvhTriangleMeshShape,
         numpy.array([1, 2, 3], 'int32'), numpy.array([4, 5, 6], 'float32')),
        (bullet.RigidBody,),
        (bullet.CollisionDispatcher,),
        (bullet.AxisSweep3, bullet.Vector3(0, 0, 0), bullet.Vector3(2, 2, 2)),
        (bullet.SequentialImpulseConstraintSolver,),
        (bullet.DiscreteDynamicsWorld,),
        (bullet.CollisionWorld,),
                 ]:
        measure(n, type)


if __name__ == '__main__':
    main()
