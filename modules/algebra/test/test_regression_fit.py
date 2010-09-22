import IMP
import IMP.test
import IMP.algebra

class RegressionTests(IMP.test.TestCase):
    """Test Linear and Parabolic fits"""

    def test_linear(self):
        """Check Linear Fit"""
        v1 = IMP.algebra.Vector2D(1.0, 4.0)
        v2 = IMP.algebra.Vector2D(2.0, 5.0)
        v3 = IMP.algebra.Vector2D(3.0, 6.0)
        vs= IMP.algebra.Vector2Ds()
        vs.append(v1)
        vs.append(v2)
        vs.append(v3)
        lf = IMP.algebra.LinearFit(vs)
        lf.show()
        self.assertAlmostEqual(lf.get_a(), 1.0, delta=0.01)
        self.assertAlmostEqual(lf.get_b(), 3.0, delta=0.01)

    def test_parabolic(self):
        """Check Parabolic Fit"""
        v1 = IMP.algebra.Vector2D(1.0, 9.0)
        v2 = IMP.algebra.Vector2D(2.0, 15.0)
        v3 = IMP.algebra.Vector2D(3.0, 23.0)
        vs= IMP.algebra.Vector2Ds()
        vs.append(v1)
        vs.append(v2)
        vs.append(v3)
        pf = IMP.algebra.ParabolicFit(vs)
        pf.show()
        self.assertAlmostEqual(pf.get_a(), 1.0, delta=0.01)
        self.assertAlmostEqual(pf.get_b(), 3.0, delta=0.01)
        self.assertAlmostEqual(pf.get_c(), 5.0, delta=0.01)

if __name__ == '__main__':
    IMP.test.main()
