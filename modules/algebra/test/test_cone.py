import unittest
import IMP
import IMP.test
import IMP.algebra
import math

class ConeTests(IMP.test.TestCase):

    def test_cone_construction(self):
        """Check that cones on Z are constructed correctly"""
        s = IMP.algebra.Segment3D(IMP.algebra.Vector3D(0.0,0.0,0.0),
                                  IMP.algebra.Vector3D(0.0,0.0,5.0))
        cone = IMP.algebra.Cone3D(s,4.0)
        self.assertEqual((cone.get_tip()-s.get_point(0)).get_magnitude() < 0.01,True)
        self.assertEqual(cone.get_contains(IMP.algebra.Vector3D(0.0,0.0,3.0)),True)
        self.assertEqual(cone.get_contains(IMP.algebra.Vector3D(0.5,0.5,3.0)),True)

        self.assertEqual(cone.get_contains(IMP.algebra.Vector3D(1.0,1.0,-3.0)),False)


    def test_sphere_patch2(self):
        """Testing sampling a patch"""
        sphere= IMP.algebra.Sphere3D(IMP.algebra.random_vector_in_unit_box(), 10)
        n= IMP.algebra.random_vector_on_unit_sphere()
        p= IMP.algebra.random_vector_in_sphere(sphere.get_center(), sphere.get_radius())
        plane= IMP.algebra.Plane3D(p, n)
        sp = IMP.algebra.Sphere3DPatch(sphere,plane)
        bs=IMP.algebra.Sphere3D(sphere.get_center(),
                                sphere.get_radius()*1.1)
        bs.show(); print
        for v in IMP.algebra.uniform_cover(sp,3):
            v.show(); print
            self.assertEqual(bs.get_contains(v),True)

if __name__ == '__main__':
    unittest.main()
