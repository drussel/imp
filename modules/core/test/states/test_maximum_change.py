import unittest
import IMP, IMP.test
import IMP.core
import os.path

class TestBL(IMP.test.TestCase):
    def test_it(self):
        """Test MaximumChangeScoreState"""
        m= IMP.Model()
        ps= self.create_particles_in_box(m, 20)
        vs=[]
        print "setting up"
        for p in ps:
            d= IMP.core.XYZDecorator(p)
            vs.append(d.get_coordinates())
        pc= IMP.core.ListParticleContainer(ps)
        print "initing"
        mc= IMP.core.MaximumChangeScoreState(pc,
                                             IMP.core.XYZDecorator.get_xyz_keys())
        m.add_score_state(mc)
        print "evaluating"
        m.evaluate(False)
        self.assertEqual(mc.get_maximum_change(), 0)
        mmax=0
        print "perturbing"
        for i in range(0,len(ps)):
            d= IMP.core.XYZDecorator(ps[i])
            v= IMP.random_vector_in_sphere()
            print v[0]
            mmax=max(mmax, float(v[0]), float(v[1]), float(v[2]))
            d.set_coordinates(d.get_coordinates()+v)
        print "evaluating again"
        m.evaluate(False)
        self.assertEqual(mmax, mc.get_maximum_change())



if __name__ == '__main__':
    unittest.main()
