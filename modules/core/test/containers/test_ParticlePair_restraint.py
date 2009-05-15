# NOTE: This file is generated from modules/core/tools/container_tempates/test.py
# do not edit.

import unittest
import IMP
import IMP.test
import IMP.core
import math


def singleton_cmp(a,b):
    return cmp(a.get_name(), b.get_name())

def pair_cmp(a,b):
    v0= cmp(a[0].get_name(), b[0].get_name())
    if v0 != 0: return v0
    return cmp(a[1].get_name(), b[1].get_name())

def triplet_cmp(a,b):
    v0= cmp(a[0].get_name(), b[0].get_name())
    if v0 != 0: return v0
    return pair_cmp([a[1].get_name(), a[2].get_name()],
                    [b[1].get_name(), b[2].get_name()])


def evaluate_singleton_score(f, s):
    return f.evaluate(s, None)
def evaluate_pair_score(f, s):
    return f.evaluate(s[0], s[1], None)
# This file is generated by the make-container script

class ParticlePairContainerTest(IMP.test.TestCase):
    """Tests for PairContainer related objects"""

    def create_particle(self,m):
        p= IMP.Particle(m)
        d=IMP.core.XYZ.create(p)
        d.set_coordinates(IMP.algebra.random_vector_in_unit_box())
        p.add_attribute(IMP.FloatKey("thekey"), d.get_x())
        return p

    def create_particle_pair(self,m):
        p0= IMP.Particle(m)
        p1= IMP.Particle(m)
        d0= IMP.core.XYZ.create(p0)
        d1= IMP.core.XYZ.create(p1)
        d0.set_coordinates(IMP.algebra.random_vector_in_unit_box())
        d1.set_coordinates(IMP.algebra.random_vector_in_unit_box())
        return IMP.ParticlePair(p0,p1)

    def same_particle(self, a, b):
        print str(a.get_name())+ " vs " + str(b.get_name())
        return a.get_name() == b.get_name()

    def same_particle_pair(self, a,b):
        print str(a[0].get_name())+ ", "\
            + str(a[1].get_name()) + " vs " \
            + str(b[0].get_name()) + ", "\
            + str(b[1].get_name())
        return self.same_particle(a[0], b[0]) and self.same_particle(a[1], b[1])

    def create_singleton_score(self):
        uf= IMP.core.Linear(0,1)
        return IMP.core.AttributeSingletonScore(uf,IMP.FloatKey("thekey"))

    def create_pair_score(self):
        uf= IMP.core.Linear(0,1)
        return IMP.core.DistancePairScore(uf)

    def create_singleton_restraint(self, ps,p):
        return IMP.core.SingletonRestraint(ps, p)

    def create_pair_restraint(self, ps, p):
        return IMP.core.PairRestraint(ps, p.first, p.second)


    def test_restraint(self):
        """Test the PairsRestraint"""
        m= IMP.Model()
        gs=self.create_pair_score()
        c= IMP.core.ListPairContainer()
        f=0
        for i in range(0,10):
            p=self.create_particle_pair(m)
            f= f+ evaluate_pair_score(gs, p)
            c.add_particle_pair(p)
            r= IMP.core.PairsRestraint(gs, c)
            r.set_was_owned(True)
        m.add_restraint(r)
        self.assertInTolerance(m.evaluate(False), f, .1*f)
        p=self.create_particle_pair(m)
        f= f+ evaluate_pair_score(gs, p)
        c.add_particle_pair(p)
        self.assertInTolerance(m.evaluate(False), f, .1*f)


    def test_srestraint(self):
        """Test the PairRestraint"""
        m= IMP.Model()
        gs=self.create_pair_score()
        p=self.create_particle_pair(m)
        f= evaluate_pair_score(gs, p)
        r= self.create_pair_restraint(gs, p)
        r.set_was_owned(True)
        m.add_restraint(r)
        self.assertInTolerance(m.evaluate(False), f, .1*f)

    def test_min_restraint(self):
        """Test the MinimumPairScoreRestraint"""
        m= IMP.Model()
        c= IMP.core.ListPairContainer()
        self.assertEqual(c.get_ref_count(), 1)
        for i in range(0,10):
            c.add_particle_pair(self.create_particle_pair(m))
        print c.get_number_of_particle_pairs()
        d= self.create_pair_score()
        self.assertEqual(d.get_ref_count(), 1)
        r= IMP.core.MinimumPairScoreRestraint(d, c)
        self.assertEqual(d.get_ref_count(), 2)
        self.assertEqual(c.get_ref_count(), 2)
        r.set_n(4)
        m.add_restraint(r)
        f= m.evaluate(False)
        print f
        ms= []
        print c.get_number_of_particle_pairs()
        for i in range(0,10):
            ps= c.get_particle_pair(i)
            cm= evaluate_pair_score(d, ps)
            ms.append(cm)
        print ms
        ms.sort()
        mt=0
        for i in range(0, 4):
            mt = mt+ ms[i]
        print mt
        self.assertInTolerance(mt, f, .1*f)

    def test_max_restraint(self):
        """Test the MaximumPairScoreRestraint"""
        m= IMP.Model()
        c= IMP.core.ListPairContainer()
        self.assertEqual(c.get_ref_count(), 1)
        for i in range(0,10):
            c.add_particle_pair(self.create_particle_pair(m))
        print c.get_number_of_particle_pairs()
        d= self.create_pair_score()
        self.assertEqual(d.get_ref_count(), 1)
        r= IMP.core.MaximumPairScoreRestraint(d, c)
        self.assertEqual(c.get_ref_count(), 2)
        self.assertEqual(d.get_ref_count(), 2)
        r.set_n(4)
        m.add_restraint(r)
        f= m.evaluate(False)
        print f
        ms= []
        print c.get_number_of_particle_pairs()
        for i in range(0,10):
            ps= c.get_particle_pair(i)
            cm= evaluate_pair_score(d, ps)
            ms.append(cm)
        ms.sort()
        print ms
        mt=0
        for i in range(0, 4):
            mt = mt+ ms[-i-1]
        print mt
        self.assertInTolerance(mt, f, .1*f)


    def test_container(self):
        """Test backwards compatibility on PairsRestraint"""
        m= IMP.Model()
        gs=self.create_pair_score()
        r= IMP.core.PairsRestraint(gs)
        m.add_restraint(r)

        f=0
        for i in range(0,10):
            p=self.create_particle_pair(m)
            r.add_particle_pair(p)
            f= f+evaluate_pair_score(gs, p)
        self.assertInTolerance(m.evaluate(False), f, .1*f)
        p=self.create_particle_pair(m)
        f= f+ evaluate_pair_score(gs, p)
        r.add_particle_pair(p)
        self.assertInTolerance(m.evaluate(False), f, .1*f)

    def test_filter(self):
        """Testing FilteredListPairContainer"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        c= IMP.core.FilteredListPairContainer()
        f= IMP.core.ListPairContainer()
        self.assertEqual(f.get_ref_count(), 1)
        print "add"
        f.show()
        c.add_pair_filter(f)
        self.assertEqual(f.get_ref_count(), 2)
        print "assert"
        f.show()
        print "range"
        for i in range(0,10):
            print "filter add"
            f.add_particle_pair(self.create_particle_pair(m))
        print "done 1"
        for i in range(0,10):
            c.add_particle_pair(self.create_particle_pair(m))
        print "done 2"
        for p in f.get_particle_pairs():
            print "adding one"
            c.add_particle_pair(p)
        self.assertEqual(c.get_number_of_particle_pairs(), 10)
        print "all done"

    def test_set(self):
        """Testing PairContainerSet"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        print "hi"
        c= IMP.core.PairContainerSet()
        ls=[]
        cs=[]
        for i in range(0,3):
            l= IMP.core.ListPairContainer()
            c.add_pair_container(l)
            for j in range(0,3):
                t=self.create_particle_pair(m)
                l.add_particle_pair(t)
                cs.append(t)
        for p in cs:
            self.assert_(c.get_contains_particle_pair(p))
        ret=[]
        for i in range(0, len(cs)):
            ret.append(c.get_particle_pair(i))
        ret.sort(pair_cmp)
        #print ret
        cs.sort(pair_cmp)
        #rint cs
        for i in range(0, len(ret)):
            self.assertEqual(pair_cmp(ret[i], cs[i]), 0)
        self.assertEqual(c.get_number_of_particle_pairs(), len(cs))

if __name__ == '__main__':
    unittest.main()
