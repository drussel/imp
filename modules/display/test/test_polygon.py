import IMP
import IMP.test
import IMP.core
import IMP.display
import os

class TestBL(IMP.test.TestCase):
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.TERSE)
    def test_4(self):
        """Testing trivial polygon decomposition and writing"""
        IMP.set_log_level(IMP.VERBOSE)
        V=IMP.algebra.Vector3D
        o=10
        poly=[V(o,-10,-10), V(o, 10, -10), V(o, 10, 10),
              V(o,-10,10)]
        pg= IMP.display.PolygonGeometry(poly)
        pg.set_name("forward")
        w= IMP.display.PymolWriter(self.get_tmp_file_name("trivialpolygon.pym"))
        w.add_geometry(pg)
        poly.reverse()
        pg= IMP.display.PolygonGeometry(poly)
        pg.set_name("reversed")
        w.add_geometry(pg)

    def test_3(self):
        """Testing polygon decomposition and writing"""
        V=IMP.algebra.Vector3D
        o=10
        poly=[V(o,-10,-10), V(o, 10, -10), V(o, 10, 10),
              V(o,-10,10), V(o,-10,1),
              V(o,-5,1), V(o,0,5), V(o,5,0), V(o,0,-5), V(o,-5,-1),
              V(o,-10,-1)]
        pg= IMP.display.PolygonGeometry(poly)
        pg.set_name("forward")
        w= IMP.display.PymolWriter(self.get_tmp_file_name("polygon.pym"))
        w.add_geometry(pg)
        poly.reverse()
        pg= IMP.display.PolygonGeometry(poly)
        pg.set_name("reversed")
        w.add_geometry(pg)

if __name__ == '__main__':
    IMP.test.main()
