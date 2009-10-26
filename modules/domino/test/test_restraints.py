import unittest
import os
import IMP
import IMP.domino
import IMP.test

class RestraintTests(IMP.test.TestCase):
    """Tests for the SimpleDiscreteRestraint"""

    def test_invalid_restraints_file(self):
        """Loading an invalid restraints file should give an error"""
        fname = "invalid"
        m = IMP.Model()
        p1 = IMP.Particle(m)
        p2 = IMP.Particle(m)

        print >> file(fname, "w"), "garbage"
        self.assertRaises(IOError, IMP.domino.SimpleDiscreteRestraint,
                          m, fname, p1, p2)
        os.unlink(fname)

if __name__ == '__main__':
    unittest.main()
