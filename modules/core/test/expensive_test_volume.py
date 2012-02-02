from IMP import *
import IMP.core
from IMP.algebra import *
from IMP.core import *
from IMP.container import *
import IMP.test

import math

class Volume(IMP.test.TestCase):
    """Tests for angle restraints"""
    def test_volume_1(self):
        """Testing that volume restraint can separate balls"""
        if not hasattr(IMP.core, 'VolumeRestraint'):
            self.skipTest('VolumeRestraint not built (no CGAL)')
        m= Model()
        ps= []
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
        sc= IMP.container.ListSingletonContainer(ps)
        vr= VolumeRestraint(Harmonic(0,1), sc, 4**3*3.1415*4.0/3.0*len(ps))
        m.add_restraint(vr)
        mc= MonteCarlo(m)
        mc.add_mover(BallMover(ps, 4))
        mc.set_score_threshold(.2)
        for i in range(5):
            try:
                mc.optimize(10)
                if m.evaluate(False) < .2:
                    break
            except IMP.ValueException:
                # Catch CG failure
                pass
        self.assertLess(m.evaluate(False), .2)
    def _test_volume_2(self):
        """Testing that volume restraint can change radius"""
        m= Model()
        IMP.set_log_level(IMP.VERBOSE)
        ps= []
        p= Particle(m)
        inits=Sphere3D(get_random_vector_in(BoundingBox3D(Vector3D(0,0,0),
                                                          Vector3D(5,5,5))),
                       4)
        print inits
        d=XYZR.setup_particle(p, inits)
        print d
        ps.append(p)
        d.set_coordinates_are_optimized(True)
        d.get_particle().set_is_optimized(XYZR.get_radius_key(), True)
        sc= ListSingletonContainer(ps)
        vr= VolumeRestraint(Harmonic(0,.001), sc, 5**3*3.1415*4.0/3.0*len(ps))
        m.add_restraint(vr)
        #c= SteepestDescent()
        #c.set_step_size(.1)
        #c.set_threshold(1)
        c= MonteCarlo(m)
        c.set_score_threshold(.1)
        c.optimize(20)
        print d
        self.assertAlmostEqual(d.get_radius(), 5, delta=.1)

if __name__ == '__main__':
    IMP.test.main()
