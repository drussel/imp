import IMP
import IMP.test
import IMP.core
import IMP.container
import os

class CGOptimizerTests(IMP.test.TestCase):
    def test_incr(self):
        """Testing incremental scoring with Monte Carlo"""
        m= IMP.Model()
        m.set_log_level(IMP.SILENT)
        mc= IMP.core.MonteCarlo(m)
        #mc.set_log_level(IMP.VERBOSE)
        ps=[]
        bb= IMP.algebra.get_unit_bounding_box_3d()
        for i in range(0,10):
            p= IMP.Particle(m)
            d= IMP.core.XYZR.setup_particle(p)
            ps.append(d)
            d.set_coordinates(IMP.algebra.get_random_vector_in(bb))
            d.set_radius(.1)
            d.set_coordinates_are_optimized(True)
        cpc= IMP.container.ConsecutivePairContainer(ps)
        hps= IMP.core.HarmonicDistancePairScore(1,100)
        #hps.set_log_level(IMP.VERBOSE)
        r= IMP.container.PairsRestraint(hps, cpc)
        #r.set_log_level(IMP.VERBOSE)
        m.add_restraint(r)
        mc.set_use_incremental_evaluate(True)
        ms= [IMP.core.BallMover([x], 2) for x in ps]
        mv= IMP.core.SerialMover(ms)
        mc.add_mover(mv)
        mc.optimize(1000)
        print m.evaluate(False)
        self.assertLess(m.evaluate(False), 3)



if __name__ == '__main__':
    IMP.test.main()
