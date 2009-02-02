import unittest
import IMP, IMP.test
import IMP.core
import os.path

class TestBL(IMP.test.TestCase):
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.TERSE)

    def _testit(self, rk, pref):
        """Test logging to a VRML file"""
        m= IMP.Model()
        o= IMP.core.SteepestDescent()
        o.set_model(m)
        nm = pref + "vrmltest%03d.vrml"
        p0= IMP.Particle()
        m.add_particle(p0)
        d0= IMP.core.XYZDecorator.create(p0)
        p0.add_attribute(rk, 1.5, False)
        d0.set_x(0)
        d0.set_y(0)
        d0.set_z(0)

        p1= IMP.Particle()
        m.add_particle(p1)
        d1= IMP.core.XYZDecorator.create(p1)
        d1.set_x(1)
        d1.set_y(1)
        d1.set_z(1)
        pc= IMP.core.ListSingletonContainer(IMP.Particles([p0,p1]))
        a= IMP.core.VRMLLogOptimizerState(pc, nm)
        a.set_radius_key(rk)
        o.add_optimizer_state(a)
        a.update()

        os.remove(pref + "vrmltest000.vrml")

    def test_1(self):
        """Testing the VRML log"""
        self._testit(IMP.FloatKey("radius"),
                     "test1")

    def test_2(self):
        """Testing the VRML log with new attribute names"""
        self._testit(IMP.FloatKey("another_radius"),
                     "test1")
    def test_skip(self):
        """Test skipping steps in the VRML log"""
        IMP.set_log_level(IMP.TERSE)
        m= IMP.Model()
        o= IMP.core.SteepestDescent()
        o.set_model(m)
        nm = "skip" + "vrmltest%03d.vrml"
        # Possible clean up from any previous failed runs:
        try:
            os.remove("skip" + "vrmltest002.vrml")
        except OSError:
            pass
        p0= IMP.Particle()
        m.add_particle(p0)
        d0= IMP.core.XYZDecorator.create(p0)
        p0.add_attribute(IMP.FloatKey("radius"), 1.5, False)
        d0.set_x(0)
        d0.set_y(0)
        d0.set_z(0)

        p1= IMP.Particle()
        m.add_particle(p1)
        d1= IMP.core.XYZDecorator.create(p1)
        d1.set_x(1)
        d1.set_y(1)
        d1.set_z(1)
        pc= IMP.core.ListSingletonContainer(IMP.Particles([p0,p1]))
        a= IMP.core.VRMLLogOptimizerState(pc, nm)
        a.set_skip_steps(10) # kind of a hack
        r= IMP.core.DistanceRestraint(IMP.core.Harmonic(0,10), p0, p1);
        m.add_restraint(r);
        o.add_optimizer_state(a)
        o.optimize(11)

        os.remove("skip" + "vrmltest000.vrml")
        os.remove("skip" + "vrmltest001.vrml")
        self.assert_(not os.path.isfile("skip" + "vrmltest002.vrml"))

if __name__ == '__main__':
    unittest.main()
