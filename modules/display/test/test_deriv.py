import IMP
import IMP.algebra
import IMP.display
import IMP.core
import IMP.test
import unittest

class TestBL(IMP.test.TestCase):
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.TERSE)

    def test_derivs(self):
        """Testing execution of derivative display support"""
        # note that there are no actual checks here at this point
        m= IMP.Model()
        pts=[IMP.algebra.Vector3D(1,0,0), IMP.algebra.Vector3D(0,1,0),
             IMP.algebra.Vector3D(-1,0,0), IMP.algebra.Vector3D(0,-1,0)]

        ps= IMP.Particles()
        for i in range(0,4):
            p= IMP.Particle(m)
            d= IMP.core.XYZ.create(p, pts[i])
            ps.append(p)
        p= IMP.Particle(m)
        d= IMP.core.XYZ.create(p)
        hd= IMP.core.Hierarchy.create(p, ps)

        for i in range(0,4):
            u= IMP.core.Harmonic(0,1)
            s= IMP.core.DistanceToSingletonScore(u, pts[(i+1)%4])
            r= IMP.core.SingletonRestraint(s, ps[i])
            m.add_restraint(r)

        m.evaluate(True)
        w= IMP.display.BildWriter()
        w.set_file_name(self.get_tmp_file_name("deriv.bild"))
        for i in range(0,4):
            w.add_geometry(IMP.display.XYZDerivativeGeometry(IMP.core.XYZ(ps[i])))
        w.set_file_name("")

        ss= IMP.core.create_rigid_body(p, ps)
        rbd= IMP.core.RigidBody(p)
        IMP.set_log_level(IMP.TERSE)
        print "eval"
        m.evaluate(True)
        w= IMP.display.BildWriter()
        w.set_file_name(self.get_tmp_file_name("qderiv.bild"))
        #oge= display.XYZRGeometryExtractor(FloatKey("hi"))
        for i in range(0,4):
            gs = IMP.display.RigidBodyDerivativeGeometry(rbd)
            w.add_geometry(gs)
            print gs
        w.set_file_name("")

if __name__ == '__main__':
    unittest.main()
