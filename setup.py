
from distutils.core import Extension, setup
from Cython.Distutils import build_ext

setup(
    name="bullet",
    ext_modules=[Extension(
        "bullet",
        ["bullet/bullet.pyx"],
        libraries=["LinearMath",
                   "BulletCollision",
                   "BulletDynamics",
                   "BulletSoftBody"],
        language="c++")],
    cmdclass={'build_ext': build_ext},
    )
