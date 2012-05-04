import IMP
import IMP.test
import IMP.algebra
import IMP.core
import IMP.kmeans

class DistanceTests(IMP.test.TestCase):
    """Test the symmetry restraint"""
    def test_symmetry(self):
        """Test something pointless and slow"""
        m= IMP.Model()
        for i in range(0,100000):
            p= IMP.Particle(m)

if __name__ == '__main__':
    IMP.test.main()
