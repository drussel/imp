import sys
import unittest
import IMP
import IMP.test
import IMP.domino2
import IMP.core
import IMP.atom
import IMP.helper
import time

class DOMINOTests(IMP.test.TestCase):

    def test_global_min2(self):
        """Test optimization of model"""
        m= IMP.Model()
        m.set_log_level(IMP.VERBOSE)
        ps=[]
        vs=[IMP.algebra.Vector3D(0,0,0),
            IMP.algebra.Vector3D(0,0,100),
            IMP.algebra.Vector3D(100,0,0)]
        pst = IMP.domino2.ParticleStatesTable()
        for i in range(10):
            p= IMP.Particle(m)
            IMP.core.XYZR.setup_particle(p,IMP.algebra.Sphere3D(IMP.algebra.Vector3D(i,i,i), i))
            ps.append(p)
            pst.set_particle_states(p, IMP.domino2.XYZStates(vs))
        IMP.set_log_level(IMP.VERBOSE)
        cp= IMP.container.ClosePairContainer(IMP.container.ListSingletonContainer(ps), 1)
        r=IMP.container.PairsRestraint(IMP.core.DistancePairScore(IMP.core.HarmonicLowerBound(0,1)), cp);
        m.add_restraint(r)
        print "getting graph"
        dg= IMP.get_dependency_graph(m.get_score_states(),
                                     IMP.get_restraints(m.get_restraints())[0])
        dg.show()
        print "optimizing"
        doc= IMP.domino2.OptimizeContainers(m, pst)
        dg= IMP.get_dependency_graph(m.get_score_states(),
                                     IMP.get_restraints(m.get_restraints())[0])
        dg.show()
        dor= IMP.domino2.OptimizeRestraints(m, pst.get_particles())
        print "optimized restraints"
        dg= IMP.get_dependency_graph(m.get_score_states(),
                                     IMP.get_restraints(m.get_restraints())[0])
        dg.show()
        self.assert_(len(IMP.get_restraints(m.get_restraints())[0]) > 1)
        del dor
        self.assert_(len(IMP.get_restraints(m.get_restraints())[0]) == 1)
if __name__ == '__main__':
    unittest.main()
