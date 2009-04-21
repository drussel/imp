import unittest
import IMP.test
import IMP.algebra
import StringIO
import os
import math

class Vector3DTests(IMP.test.TestCase):
    def test_magnitude(self):
        """Check Vector3D magnitude"""
        v = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        self.assertEqual(v.get_squared_magnitude(), 14.0)
        self.assertAlmostEqual(v.get_magnitude(), math.sqrt(14.0), places=1)

    def test_io(self):
        """Check I/O of Vector3Ds"""
        vs1 = IMP.algebra.Vector3Ds()
        vs1.push_back(IMP.algebra.Vector3D(1.0, 2.0, 3.0))
        vs1.push_back(IMP.algebra.Vector3D(4.0, 5.0, 6.0))
        self.assertRaises(AttributeError, IMP.algebra.write_pts, vs1, None)
        self.assertRaises(AttributeError, IMP.algebra.read_pts, None)

        # Test read/write for regular files and file-like objects
        sio = StringIO.StringIO()
        IMP.algebra.write_pts(vs1, file('vectors', 'w'))
        IMP.algebra.write_pts(vs1, sio)
        sio.seek(0)
        for vs2 in (IMP.algebra.read_pts(file('vectors')),
                    IMP.algebra.read_pts(sio)):
            for v1, v2 in ((vs1[0], vs2[0]), (vs1[1], vs2[1])):
                self.assertEqual(v1[0], v2[0])
                self.assertEqual(v1[1], v2[1])
                self.assertEqual(v1[2], v2[2])
        os.unlink('vectors')

    def test_component(self):
        """Check Vector3D components"""
        v = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        self.assertEqual(v[0], 1.0)
        self.assertEqual(v[1], 2.0)
        self.assertEqual(v[2], 3.0)
        v[0] = 10.0
        self.assertEqual(v[0], 10.0)

    def test_scalar_product(self):
        """Check Vector3D scalar product"""
        v1 = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        v2 = IMP.algebra.Vector3D(10.0, 1.0, 2.0)
        self.assertInTolerance(v1.scalar_product(v2), v2.scalar_product(v1),
                               .1)
        self.assertInTolerance(v1.scalar_product(v2), v1 * v2,
                               .1)
        self.assertInTolerance(v1.scalar_product(v2), v2 * v1, .1)
        self.assertInTolerance(v1.scalar_product(v2), 18.0, .1)

    def test_vector_product(self):
        """Check Vector3D vector product"""
        v1 = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        v2 = IMP.algebra.Vector3D(10.0, 1.0, 2.0)
        v12 = IMP.algebra.vector_product(v1, v2)
        v21 = IMP.algebra.vector_product(v2, v1)
        expected_v12 = (1.0, 28.0, -19.0)
        for i in range(3):
            self.assertInTolerance(v12[i], -v21[i], .1)
            self.assertInTolerance(v12[i], expected_v12[i],.1)

    def test_difference(self):
        """Check Vector3D difference"""
        v1 = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        v2 = IMP.algebra.Vector3D(10.0, 1.0, 2.0)
        diff = v1 - v2
        v1 -= v2
        expected_diff = IMP.algebra.Vector3D(-9.0, 1.0, 1.0)
        self.assertInTolerance((diff-expected_diff).get_magnitude(),
                               0, .1)
        self.assertInTolerance((v1-expected_diff).get_magnitude(),
                               0, .1)

    def test_addition(self):
        """Check Vector3D addition"""
        v1 = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        idv1 = id(v1)
        cppobj = str(v1.this)
        v2 = IMP.algebra.Vector3D(10.0, 1.0, 2.0)
        sum = v1 + v2
        v1 += v2
        # Inplace addition should not change the Python object identity:
        self.assertEqual(id(v1), idv1)
        # The underlying C++ object pointer should be unchanged too:
        self.assertEqual(str(v1.this), cppobj)
        expected_sum = IMP.algebra.Vector3D(11.0, 3.0, 5.0)
        self.assertInTolerance((sum-expected_sum).get_magnitude(),
                               0, .1)
        self.assertInTolerance((v1-expected_sum).get_magnitude(),
                               0, .1)

    def test_scalar_multiplication(self):
        """Check Vector3D multiplication by a scalar"""
        v1 = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        idv1 = id(v1)
        s1 = 3.0
        prod = v1 * s1
        v1 *= s1
        # Inplace multiplication should not change the Python object identity:
        self.assertEqual(id(v1), idv1)
        expected_prod = (3.0, 6.0, 9.0)
        for i in range(3):
            self.assertInTolerance(prod[i], expected_prod[i],
                                   .1)
            self.assertInTolerance(v1[i], expected_prod[i],
                                   .1)

    def test_scalar_division(self):
        """Check Vector3D division by a scalar"""
        v1 = IMP.algebra.Vector3D(3.0, 6.0, 9.0)
        idv1 = id(v1)
        s1 = 3.0
        prod = v1 / s1
        v1 /= s1
        # Inplace division should not change the Python object identity:
        self.assertEqual(id(v1), idv1)
        expected_prod = (1.0, 2.0, 3.0)
        for i in range(3):
            self.assertEqual(prod[i], expected_prod[i])
            self.assertEqual(v1[i], expected_prod[i])

    def test_distance(self):
        """Check distance between two vectors"""
        v1 = IMP.algebra.Vector3D(3.0, 6.0, 9.0)
        v2 = IMP.algebra.Vector3D(1.0, 2.0, 3.0)
        self.assertEqual(IMP.algebra.squared_distance(v1, v2), 56)
        self.assertInTolerance(IMP.algebra.distance(v1, v2), 7.4833, 0.01)

    def test_generators(self):
        """Check the Vector3D generators"""
        # test calling since it is a bit non-trivial in SWIG
        v= IMP.algebra.random_vector_in_unit_sphere()
        v= IMP.algebra.random_vector_in_sphere(IMP.algebra.Vector3D(0,0,0), 1)

if __name__ == '__main__':
    unittest.main()
