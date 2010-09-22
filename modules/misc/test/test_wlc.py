import IMP
import IMP.test
import IMP.misc


class WLCTests(IMP.test.TestCase):
    """Tests for WLC unary function"""

    def test_wlc(self):
        """Test that the WormLikeChain values are sane"""
        wlc= IMP.misc.WormLikeChain(200, 3.4)
        self.check_unary_function_min(wlc, 0, 250, .5, 0)
        self.check_unary_function_deriv(wlc, 0, 250, .5)

        self.assert_(wlc.evaluate_with_derivative(180)[1] > 4.2)

if __name__ == '__main__':
    IMP.test.main()
