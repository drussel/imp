import IMP
import IMP.test
import IMP.bullet
import IMP.core
import IMP.algebra
import IMP.display
import StringIO
import math

class AngleRestraintTests(IMP.test.TestCase):
    """Tests for angle restraints"""
    def count_hits(self, ps):
        ret=0
        for i in range(0, len(ps)):
            for j in range(0,i):
                pi= ps[i]
                pj= ps[j]
                di= IMP.core.XYZR(pi)
                dj= IMP.core.XYZR(pj)
                if IMP.core.get_distance(di, dj) < -.1:
                    ret=ret+1
        return ret
    def count_annulus(self, ps):
        ret=0
        for i in range(0, len(ps)):
            d= IMP.core.XYZ(ps[i]).get_coordinates().get_magnitude()
            if d <3 or d > 6:
                ret=ret+1
        return ret
    def create(self):
        m= IMP.Model()
        bb= IMP.algebra.BoundingBox3D(IMP.algebra.Vector3D(0,0,0),
                                      IMP.algebra.Vector3D(5,5,5))
        ps=[]
        for i in range(0,30):
            p= IMP.Particle(m)
            d= IMP.core.XYZR.setup_particle(p)
            d.set_coordinates(IMP.algebra.get_random_vector_in(bb))
            d.set_coordinates_are_optimized(True)
            ps.append(p)
            d.set_radius(2)
            print d
        r= IMP.core.ExcludedVolumeRestraint(IMP.container.ListSingletonContainer(ps))
        m.add_restraint(r)
        return (m, bb, ps)
    def display(self, ps, i):
        w= IMP.display.PymolWriter(self.get_tmp_file_name("col."+str(i)+".pym"))
        for i,p in enumerate(ps):
            g= IMP.core.XYZRGeometry(IMP.core.XYZR(p))
            g.set_color(IMP.display.get_display_color(i))
            w.add_geometry(g)
    def log(self, ps):
        w= IMP.display.PymolWriter()
        l= IMP.display.WriteOptimizerState(w, self.get_tmp_file_name("step.%1%.pym"))
        for i,p in enumerate(ps):
            g= IMP.core.XYZRGeometry(IMP.core.XYZR(p))
            g.set_color(IMP.display.get_display_color(i))
            l.add_geometry(g)
    def test_rco(self):
        """Test basic ResolveCollision optimization"""
        (m, bb, ps)= self.create()
        print "intesections: ", self.count_hits(ps)
        opt= IMP.bullet.ResolveCollisionsOptimizer(m)
        self.display(ps, 0)
        for i in range(0,10):
            opt.optimize(100)
            self.display(ps, i+1)
            print "intesections: ", self.count_hits(ps)
            if self.count_hits(ps) ==0:
                break
        self.display(ps, i+2)
        self.assertEqual(self.count_hits(ps), 0)
    def test_rcol(self):
        """Test local ResolveCollision optimization"""
        (m, bb, ps)= self.create()
        print "intesections: ", self.count_hits(ps)
        self.display(ps, -1)
        opt= IMP.bullet.ResolveCollisionsOptimizer(m)
        opt.set_local_stiffness(1)
        for i in range(0,10):
            self.display(ps, i)
            opt.optimize(100)
            print "intesections: ", self.count_hits(ps)
            #if self.count_hits(ps) ==0:
            #    break
        self.display(ps, i+1)
        self.assertEqual(self.count_hits(ps), 0)
    def test_rcos(self):
        """Test basic ResolveCollision optimization with springs"""
        (m, bb, ps)= self.create()
        for i in range(1,10):
            r= IMP.core.PairRestraint(IMP.core.HarmonicDistancePairScore(4,1),
                                      (ps[i-1], ps[i]))
            r.set_name(ps[i-1].get_name() + " " + ps[i].get_name())
            m.add_restraint(r)
        print "intesections:", self.count_hits(ps),"score:", m.evaluate(False)
        opt= IMP.bullet.ResolveCollisionsOptimizer(m)
        #opt.set_local_stiffness(.01)
        for i in range(0,100):
            self.display(ps, i)
            opt.optimize(100)
            print "intesections:", self.count_hits(ps), "score:", m.evaluate(False)
        self.display(ps, i+1)
        self.assertLess(m.evaluate(False), .1)
        self.assertEqual(self.count_hits(ps), 0)
    def test_rcor(self):
        """Test basic ResolveCollision optimization with restraints"""
        (m, bb, ps)= self.create()
        l= IMP.container.ListSingletonContainer(ps)
        inner= IMP.container.SingletonsRestraint(IMP.core.DistanceToSingletonScore(IMP.core.HarmonicLowerBound(3, 1), IMP.algebra.Vector3D(0,0,0)), l)
        m.add_restraint(inner)
        outer= IMP.container.SingletonsRestraint(IMP.core.DistanceToSingletonScore(IMP.core.HarmonicLowerBound(6, 1), IMP.algebra.Vector3D(0,0,0)), l)
        m.add_restraint(outer)
        print "intesections:", self.count_hits(ps),"annulus:",self.count_annulus(ps)
        opt= IMP.bullet.ResolveCollisionsOptimizer(m)
        for i in range(0,10):
            opt.optimize(100)
            print "intesections:", self.count_hits(ps), "annulus:",self.count_annulus(ps)
        self.assertEqual(self.count_hits(ps), 0)

if __name__ == '__main__':
    IMP.test.main()
