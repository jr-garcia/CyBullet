from setuptools import Extension, setup
from numpy import get_include
from sys import platform

plat = platform
incl = [get_include()]
extrac = []

if 'win' in plat:
    extrac.append('/EHsc')
elif 'linux' in plat:
    incl.extend(['/usr/local/include/bullet', '/usr/include/bullet'])
    extrac.extend(['-w', '-O3'])
else:
    print('Bullet Setup.py warning: Unknown platform \'{}\''.format(plat))


setup(
    name="bullet",
    packages=["bullet"],
    ext_modules=[Extension(
        "bullet.bullet",
        ["bullet/bullet.pyx", "bullet/overr.cpp"],
        libraries=[
                "BulletSoftBody",
                "BulletDynamics",
                "BulletCollision",
                "LinearMath",
                ],
        include_dirs=incl,
        extra_compile_args=extrac,
        language="c++",
        install_requires=['numpy'])
    ])
