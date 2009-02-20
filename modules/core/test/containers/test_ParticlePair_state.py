import unittest
import IMP
import IMP.test
import IMP.utils
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


class SingletonTestModifier(IMP.SingletonModifier):
    def __init__(self, k):
        IMP.SingletonModifier.__init__(self)
        self.k=k
    def show(self, j):
        print "Test Particle"
    def apply(self, p):
        p.add_attribute(self.k, 1)
    def get_version_info(self):
        return 1

class PairTestModifier(IMP.PairModifier):
    def __init__(self, k):
        IMP.PairModifier.__init__(self)
        self.k=k
    def show(self, j):
        print "Test Particle"
    def apply(self, p0, p1):
        p0.add_attribute(self.k, 1)
        p1.add_attribute(self.k, 1)
    def get_version_info(self):
        return 1

#class TripletTestModifier(IMP.core.TripletModifier):
#    def __init__(self, k):
#        IMP.core.TripletModifier.__init__(self)
#        self.k=k
#    def show(self, j):
#        print "Test Particle"
#    def apply(self, p0, p1, p2):
#        p0.add_attribute(self.k)
#        p1.add_attribute(self.k)
#        p2.add_attribute(self.k)

def particle_has_attribute(p, k):
    return p.has_attribute(k)

def particle_pair_has_attribute(p, k):
    return p[0].has_attribute(k) and p[1].has_attribute(k)

def particle_triplet_has_attribute(p, k):
    return p[0].has_attribute(k) and p[1].has_attribute(k) and p[2].has_attribute(k)

# This file is generated by the make-container script

class ParticlePairContainerTest(IMP.test.TestCase):
    """Tests for PairContainer related objects"""

    def create_particle(self,m):
        p= IMP.Particle()
        m.add_particle(p)
        p.add_attribute(IMP.FloatKey("thekey"), float(1))
        return p

    def create_particle_pair(self,m):
        p0= IMP.Particle()
        m.add_particle(p0)
        p1= IMP.Particle()
        m.add_particle(p1)
        d0= IMP.core.XYZDecorator.create(p0)
        d1= IMP.core.XYZDecorator.create(p1)
        d0.set_coordinates(IMP.algebra.Vector3D(0,0,1))
        d1.set_coordinates(IMP.algebra.Vector3D(0,0,0))
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

    def create_particle_score(self):
        uf= IMP.core.Linear(0,1)
        return IMP.core.AttributeSingletonScore(uf,IMP.FloatKey("thekey"))

    def create_particle_pair_score(self):
        uf= IMP.core.Linear(0,1)
        return IMP.core.DistancePairScore(uf)

    def test_set(self):
        """Testing PairsScoreState"""
        # write increment an int field
        # call evaluate and check that it is incremented
        IMP.set_log_level(IMP.VERBOSE)
        print "start"
        m= IMP.Model()
        print "hi"
        c= IMP.core.ListPairContainer()
        cs=[]
        for i in range(0,30):
            t=self.create_particle_pair(m)
            c.add_particle_pair(t)
            cs.append(t)
        print "dl"
        k= IMP.IntKey("thevalue")
        f= PairTestModifier(k)
        print "apply"
        s= IMP.core.PairsScoreState(c, f, None)
        self.assert_( not f.thisown)
        self.assert_( not c.thisown)
        m.add_score_state(s)
        print "add"
        m.evaluate(False)
        for p in cs:
            self.assert_(particle_pair_has_attribute(p, k))
        print "done"

    def test_sset(self):
        """Testing PairScoreState"""
        # write increment an int field
        # call evaluate and check that it is incremented
        IMP.set_log_level(IMP.VERBOSE)
        print "start"
        m= IMP.Model()
        print "hi"
        t=self.create_particle_pair(m)
        print "dl"
        k= IMP.IntKey("thevalue")
        f= PairTestModifier(k)
        print "apply"
        s= IMP.core.PairScoreState(f, None, t)
        self.assert_( not f.thisown)
        m.add_score_state(s)
        print "add"
        m.evaluate(False)
        self.assert_(particle_pair_has_attribute(t, k))
        print "done"

if __name__ == '__main__':
    unittest.main()
