from IMP import *
from IMP.algebra import *
from IMP.core import *
from IMP.container import *
import IMP.test

import math

class Volume(IMP.test.TestCase):
    """Tests for angle restraints"""
    def test_volume_1(self):
        """Testing that volume restraint can separate balls"""
        m= Model()
        ps= IMP.Particles()
        IMP.set_log_level(IMP.VERBOSE)
        for i in range(0,3):
            p= Particle(m)
            v= get_random_vector_in(BoundingBox3D(Vector3D(0,0,0),
                                                  Vector3D(5,5,5)))
            d=XYZR.setup_particle(p, Sphere3D(v, 4))
            ps.append(p)
            p.set_is_optimized(FloatKey("x"), True)
            p.set_is_optimized(FloatKey("y"), True)
            p.set_is_optimized(FloatKey("z"), True)
        sc= ListSingletonContainer(ps)
        vr= VolumeRestraint(Harmonic(0,1), sc, 4**3*3.1415*4.0/3.0*len(ps))
        m.add_restraint(vr)
        mc= MonteCarlo(m)
        mc.add_mover(BallMover(sc, 4))
        mc.set_local_optimizer( ConjugateGradients(m))
        mc.set_score_threshold(.2)
        mc.optimize(1000)
        self.assert_(m.evaluate(False) < .2)
    def test_volume_2(self):
        """Testing that volume restraint can change radius"""
        m= Model()
        IMP.set_log_level(IMP.VERBOSE)
        ps= IMP.Particles()
        p= Particle(m)
        inits=Sphere3D(get_random_vector_in(BoundingBox3D(Vector3D(0,0,0),
                                                          Vector3D(5,5,5))),
                       4)
        print inits
        d=XYZR.setup_particle(p, inits)
        print d
        ps.append(p)
        d.set_coordinates_are_optimized(True)
        d.get_particle().set_is_optimized(XYZR.get_default_radius_key(), True)
        sc= ListSingletonContainer(ps)
        vr= VolumeRestraint(Harmonic(0,.001), sc, 5**3*3.1415*4.0/3.0*len(ps))
        m.add_restraint(vr)
        #c= SteepestDescent()
        #c.set_step_size(.1)
        #c.set_threshold(1)
        c= ConjugateGradients(m)
        c.set_score_threshold(.1)
        c.optimize(1000)
        print d
        self.assertAlmostEqual(d.get_radius(), 5, delta=.1)

if __name__ == '__main__':
    IMP.test.main()
