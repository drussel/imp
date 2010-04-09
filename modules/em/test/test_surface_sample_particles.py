import IMP
import IMP.test
import sys
import IMP.em
import unittest
import os

class SampleTests(IMP.test.TestCase):
    """Tests for sampled density maps"""

    def setUp(self):
        """initialize IMP environment create particles"""
        IMP.test.TestCase.setUp(self)
        #init IMP model ( the environment)
        self.imp_model = IMP.Model()
        self.particles = IMP.Particles()
        ## -  create a set of three particles in imp
        npart = 3
        self.radius_key=IMP.FloatKey("radius")
        self.weight_key=IMP.FloatKey("weight")
        for i, (x,y,z) in enumerate(((9.0, 9.0, 9.0),
                                     (9.0, 5.0, 9.0),
                                     (5.0, 9.0, 9.0),
                                     (9.0, 9.0, 5.0),
                                     (12.0, 3.0, 3.0),
                                     (15.0, 3.0, 3.0),
                                     (12.0, 6.0, 3.0),
                                     (3.0, 12.0, 12.0))):
            p = self.create_point_particle(self.imp_model, x,y,z)
            p.add_attribute( self.radius_key, 5.0, False)
            p.add_attribute(self.weight_key, 10.0)
            p.add_attribute(IMP.IntKey("id"), i)
            p.add_attribute(IMP.IntKey("protein"), 1)
            self.particles.append(p)
        self.particle_indexes = IMP.Ints()
        for i in range(npart):
            self.particle_indexes.push_back(i)
        print "initialization done ..."

    def test_sample_map(self):
        """Check that surface sampling works"""
        resolution=1.
        voxel_size=1.
        model_map = IMP.em.SurfaceShellDensityMap(self.particles, resolution, voxel_size,self.radius_key,self.weight_key)
        for p in self.particles:
            val=model_map.get_value(IMP.core.XYZ(p).get_coordinates())
            print val
            self.assertEqual(val>9.1 and val<10.1,True)

if __name__ == '__main__':
    unittest.main()
