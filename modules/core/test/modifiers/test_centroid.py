import unittest
import IMP
import IMP.test
import IMP.core
import random

class TestREFCover(IMP.test.TestCase):
    """Tests for RefinerCover"""

    def test_it(self):
        """Test cover refined"""
        m= IMP.Model()
        n= random.randrange(1,10)
        ps=IMP.Particles()
        ds=IMP.core.XYZRDecorators()
        pp= IMP.Particle()
        m.add_particle(pp)
        hd= IMP.core.HierarchyDecorator.create(pp)
        dd= IMP.core.XYZRDecorator.create(pp)
        for i in range(0,n):
            p= IMP.Particle()
            m.add_particle(p)
            d=IMP.core.XYZRDecorator.create(p)
            ps.append(p)
            ds.append(d)
            d.set_coordinates(IMP.random_vector_in_unit_box())
            d.set_radius(random.uniform(0,1))
            hd.add_child(IMP.core.HierarchyDecorator.create(p))
        r= IMP.core.ChildrenParticleRefiner(IMP.core.HierarchyDecorator.get_default_traits())
        c= IMP.core.CoverRefinedSingletonModifier(r)
        c.set_slack(.1)
        c.apply(pp)
        for d in ds:
            self.assert_(IMP.core.distance(d, dd) <= 0)
if __name__ == '__main__':
    unittest.main()
