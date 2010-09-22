import IMP
import IMP.test
import IMP.core
import IMP.algebra

class DecoratorTests(IMP.test.TestCase):
    def test_xyzr(self):
        """ Testing XYZR decorators"""
        m = IMP.Model()
        pa=IMP.Particle(m)
        pb=IMP.Particle(m)
        da= IMP.core.XYZR.setup_particle(pa, IMP.FloatKey("rk_1"))
        db= IMP.core.XYZR.setup_particle(pb, IMP.FloatKey("rk_0"))
        da.set_radius(1.0)
        db.set_radius(1.5)
        da.set_coordinates(IMP.algebra.Vector3D(0,0,0))
        db.set_coordinates(IMP.algebra.Vector3D(6,6,6))
        d= IMP.core.get_distance(da, db)
        self.assertInTolerance(d, 10.3-2.5, .5)



if __name__ == '__main__':
    IMP.test.main()
