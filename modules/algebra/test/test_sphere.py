import unittest
import IMP
import IMP.test
import IMP.algebra
import math

class SphereTests(IMP.test.TestCase):

    def test_sphere_construction(self):
        """Check that sphere are constructed correctly"""
        center = IMP.algebra.Vector3D(0.0,0.0,0.0)
        radius=5.0
        sph = IMP.algebra.Sphere3D(center,radius)
        self.assertEqual((sph.get_center()-center).get_magnitude() < 0.01,True)
        self.assertEqual(sph.get_radius(),radius)
        self.assertAlmostEqual(sph.get_surface_area(),math.pi*25.0*4,
                               places=1)
        self.assertAlmostEqual(sph.get_volume(),math.pi*125.0*(4.0/3),
                               places=1)

    def test_bounding_cylinder(self):
        """Check correct behavior of Sphere3D.get_bounding_cylinder"""
        center = IMP.algebra.Vector3D(0.0,0.0,0.0)
        radius=5.0
        sph = IMP.algebra.Sphere3D(center,radius)
        #print sph.get_bild_string()
        cyl = IMP.algebra.bounding_cylinder(sph)
        #print cyl.get_bild_string()
        self.assertEqual(cyl.get_radius(),radius)
        self.assertEqual(cyl.get_segment().get_length(),2.0*radius)

    def test_uniform_cover(self):
        """Check uniform cover on a sphere"""
        center = IMP.algebra.Vector3D(0.0,0.0,0.0)
        radius=20.0
        sph = IMP.algebra.Sphere3D(center,radius)
        numpts=600
        points=IMP.algebra.uniform_cover(sph,numpts)
        #check that the centroid is still the center
        sampled_centroid = IMP.algebra.Vector3D(0.0,0.0,0.0)
        self.assertEqual(len(points),numpts)
        for p in points:
            sampled_centroid = sampled_centroid + p
        sampled_centroid.show()
        sampled_centroid = sampled_centroid * (1.0/len(points))
        sampled_centroid.show()
        self.assertInTolerance((sampled_centroid-center).get_magnitude(),0,
                               4*radius/numpts**.5)


    def test_uniform_cover_not_on_000(self):
        """Check uniform cover when the the center is not on (0,0,0)"""
        center = IMP.algebra.Vector3D(4.0,5.0,-9.0)
        radius=5.0
        sph = IMP.algebra.Sphere3D(center,radius)
        points=IMP.algebra.uniform_cover(sph,400)
        #check that the centroid is still the center
        sampled_centroid = IMP.algebra.Vector3D(0.0,0.0,0.0)
        self.assertEqual(len(points),400)
        for i in range(len(points)):
            sampled_centroid = sampled_centroid + points[i]
        sampled_centroid = sampled_centroid * (1.0/len(points))
        self.assertEqual((sampled_centroid-center).get_magnitude() < 1.0,True)

    def test_sampling_of_bounding_cylinder(self):
        """Check bounding cylinder"""
        center = IMP.algebra.Vector3D(3.0,6.0,2.0)
        radius=5.0
        sph = IMP.algebra.Sphere3D(center,radius)
        cyl=IMP.algebra.bounding_cylinder(sph)
        points = IMP.algebra.grid_cover(cyl,8,8)
        sampled_centroid = IMP.algebra.Vector3D(0.0,0.0,0.0)
        self.assertEqual(len(points),8*8)
        for i in range(len(points)):
            sampled_centroid = sampled_centroid + points[i]
        sampled_centroid = sampled_centroid * (1.0/len(points))
        self.assertEqual((sampled_centroid-center).get_magnitude() < 1.0,True)

if __name__ == '__main__':
    unittest.main()
