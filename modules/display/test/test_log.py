import IMP
import IMP.test
import IMP.core
import IMP.display
import os

class TestBL(IMP.test.TestCase):
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.TERSE)
    def _testopen(self, fname):
        open(fname, "r")
    def _testit(self, writer):
        m= IMP.Model()
        o= IMP.core.SteepestDescent()
        o.set_model(m)
        p0= IMP.Particle(m)
        d0= IMP.core.XYZR.setup_particle(p0)
        d0.set_radius(1.5)
        d0.set_x(0)
        d0.set_y(0)
        d0.set_z(0)

        p1= IMP.Particle(m)
        d1= IMP.core.XYZR.setup_particle(p1)
        d1.set_x(1)
        d1.set_y(1)
        d1.set_z(1)
        d1.set_radius(1)
        IMP.set_log_level(IMP.VERBOSE)
        a= IMP.display.WriteOptimizerState(writer)
        g= IMP.display.XYZRGeometry(d0)
        #ge= IMP.display.XYZRGeometryExtractor(rk)
        ps= IMP.container.ListSingletonContainer(m)
        ps.add_particle(p1)
        #a.add_geometry(ge, ps)
        a.add_geometry(g)
        o.add_optimizer_state(a)
        a.update()
        a.update()
        a.update()
        open(self.get_tmp_file_name("testbild.2.bild"), "r").read()
        self.assertRaises(IOError, self._testopen, self.get_tmp_file_name("testbild.%1%.bild"))

    def test_3(self):
        """Testing the Bild log"""
        self._testit(IMP.display.BildWriter(self.get_tmp_file_name("testbild.%1%.bild")))

if __name__ == '__main__':
    IMP.test.main()
