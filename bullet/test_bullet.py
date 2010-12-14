
from unittest import TestCase

from bullet import CollisionWorld, CollisionObject, EmptyShape

class CollisionWorldTests(TestCase):
    def test_empty(self):
        world = CollisionWorld()
        self.assertEqual(world.getNumCollisionObjects(), 0)


    def test_addCollisionObject(self):
        world = CollisionWorld()
        obj = CollisionObject()
        shape = EmptyShape()
        obj.setCollisionShape(shape)
        world.addCollisionObject(obj)
        self.assertEqual(world.getNumCollisionObjects(), 1)

