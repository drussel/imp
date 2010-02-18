import unittest
from StringIO import StringIO
import IMP
import IMP.test
import IMP.atom
import IMP.helper

class PDBReadWriteTest(IMP.test.TestCase):

    def test_read(self):
        """Check getting a residue from an atomic protein"""
        m = IMP.Model()

        #! read PDB
        mp= IMP.atom.read_pdb(self.open_input_file("input.pdb"),
                              m, IMP.atom.NonWaterPDBSelector())
        chain= mp.get_child(0)
        r= IMP.atom.get_residue(chain, 10)
        self.assert_(r != IMP.atom.Hierarchy())
        r.show()
        rr= IMP.atom.Residue(r.get_particle())
        self.assert_(rr.get_index() == 10)

    def test_chain(self):
        """Check getting a residue from a cg protein"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        p= IMP.Particle(m)
        mp=IMP.atom.create_protein(m, 10, 100)
        r = IMP.atom.get_residue(mp, 10)
        r.show()
        self.assert_(r != IMP.atom.Hierarchy())

if __name__ == '__main__':
    unittest.main()
