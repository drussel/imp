import unittest
import IMP
import IMP.test
import IMP.multifit
import IMP.multifit.randomize
import IMP.core


class GenericTest(IMP.test.TestCase):
    """Test the python code"""

    def test_python(self):
        """Checking module python code"""
        m = IMP.Model()
        p= IMP.Particle(m)
        d= IMP.core.XYZ.setup_particle(p)
        IMP.multifit.randomize.randomize_particle(p)
        self.assert_(IMP.multifit.get_is_xyz_particle(p))

if __name__ == '__main__':
    unittest.main()
