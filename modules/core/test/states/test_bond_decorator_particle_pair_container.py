import unittest
import IMP, IMP.test
import IMP.core
import os.path

class TestBL(IMP.test.TestCase):
    def test_it(self):
        """Test bond decorator pair container"""
        m= IMP.Model()
        p0=IMP.Particle()
        p1= IMP.Particle()
        p2= IMP.Particle()
        p3= IMP.Particle()
        m.add_particle(p0)
        m.add_particle(p1)
        m.add_particle(p2)
        m.add_particle(p3)
        b0= IMP.core.BondedDecorator.create(p0)
        b1= IMP.core.BondedDecorator.create(p1)
        b3= IMP.core.BondedDecorator.create(p3)
        b= IMP.core.custom_bond(b0, b1, 1,1)

        pc= IMP.core.BondDecoratorParticlePairContainer()
        print pc.get_contains_particle_pair(IMP.ParticlePair(p0,p0))
        self.assert_(pc.get_contains_particle_pair(IMP.ParticlePair(p0,p1)))
        print 0
        self.assert_(not pc.get_contains_particle_pair(IMP.ParticlePair(p2,p0)))
        print 1
        self.assert_(not pc.get_contains_particle_pair(IMP.ParticlePair(p2,p2)))
        print 2
        self.assert_(not pc.get_contains_particle_pair(IMP.ParticlePair(p0,p0)))
        self.assert_(not pc.get_contains_particle_pair(IMP.ParticlePair(p0,p3)))


if __name__ == '__main__':
    unittest.main()
