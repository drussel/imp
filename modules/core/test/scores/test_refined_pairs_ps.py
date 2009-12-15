import unittest
import IMP
import IMP.test
import IMP.core

rk = IMP.FloatKey("radius")

class Test(IMP.test.TestCase):
    """Tests for bond refiner"""


    def test_rops(self):
        """Checking refine pairs pair score"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        pp= IMP.Particle(m)
        hpp= IMP.core.Hierarchy.setup_particle(pp)
        c=[]
        for i in range(0,10):
            p= IMP.Particle(m)
            hp= IMP.core.Hierarchy.setup_particle(p)
            hpp.add_child(hp)
            c.append(p)
        pr= IMP.core.ChildrenRefiner(IMP.core.Hierarchy.get_default_traits())
        cps= IMP.test.ConstPairScore(1)
        rps= IMP.core.RefinedPairsPairScore(pr, cps)
        ppp=IMP.ParticlePair(pp, pp)
        print type(rps)
        print type(rps.evaluate)
        da=IMP.DerivativeAccumulator();
        print type(pp)
        print type(ppp)
        self.assertEqual(rps.evaluate(pp, pp, da), 100)





if __name__ == '__main__':
    unittest.main()
