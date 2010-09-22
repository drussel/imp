import StringIO
import IMP
import IMP.test
import IMP.atom

class PDBReadWriteTest(IMP.test.TestCase):
    def _test_round_trip(self, name, selector):
        m= IMP.Model()
        p= IMP.atom.read_pdb(self.get_input_file_name(name),
                             m, selector)
        n1= len(IMP.atom.get_by_type(p, IMP.atom.ATOM_TYPE))
        sout= StringIO.StringIO()
        IMP.atom.write_pdb(p, sout)
        sin= StringIO.StringIO(sout.getvalue())
        p2= IMP.atom.read_pdb(sin, m, selector)
        n2= len(IMP.atom.get_by_type(p2, IMP.atom.ATOM_TYPE))
        self.assertEqual(n1, n2)
        self.assertGreater(n1, 0)
    def test_bad_read(self):
        """Check that read_pdb behaves OK on invalid files"""
        m= IMP.Model()
        self.assertRaises(IMP.IOException,
                          IMP.atom.read_pdb,
                          self.get_input_file_name("notafile.pdb"),
                          m)
        # we don't actually check if a file is a pdb or not
        # and can't conclude it is not due to not reading any atoms
        # as the selector may filter them all.
        self.assertRaises(IMP.ValueException,
                          IMP.atom.read_pdb,
                          self.open_input_file("notapdb.pdb"),
                          m)
    def test_round_trips(self):
        """Testing that we can read and write various pdbs"""
        self._test_round_trip("1d3d-protein.pdb", IMP.atom.NonWaterPDBSelector())
        self._test_round_trip("1d3d-protein.pdb", IMP.atom.NonAlternativePDBSelector())
        self._test_round_trip("1DQK.pdb", IMP.atom.NonWaterPDBSelector())
        self._test_round_trip("1z5s_A.pdb", IMP.atom.NonWaterPDBSelector())
        self._test_round_trip("input.pdb", IMP.atom.NonWaterPDBSelector())
        self._test_round_trip("protein_water.pdb", IMP.atom.NonWaterPDBSelector())
        self._test_round_trip("protein_water.pdb", IMP.atom.NonAlternativePDBSelector())
        self._test_round_trip("regression_0.pdb", IMP.atom.NonAlternativePDBSelector())
        self._test_round_trip("single_dna.pdb", IMP.atom.NonAlternativePDBSelector())
    def test_read(self):
        """Check reading a pdb with one protein"""
        m = IMP.Model()

        #! read PDB
        mp= IMP.atom.read_pdb(self.open_input_file("input.pdb"),
                              m, IMP.atom.NonWaterPDBSelector())
        self.assertEqual(m.get_number_of_particles(), 1132)
        #IMP.atom.show_molecular_hierarchy(mp)
        IMP.atom.show(mp)
        IMP.atom.add_bonds(mp)
        bds = IMP.atom.get_internal_bonds(mp)
        self.assertEqual(len(bds), 1020)
        IMP.atom.add_radii(mp)
        IMP.atom.show_molecular_hierarchy(mp)

        m2 = IMP.Model()
        mp= IMP.atom.read_pdb(self.open_input_file("input.pdb"),
                              m2, IMP.atom.CAlphaPDBSelector())
        self.assertEqual(m2.get_number_of_particles(), 260)
        IMP.atom.add_bonds(mp)
        bds = IMP.atom.get_internal_bonds(mp)
        self.assertEqual(len(bds), 0)
        # one more test for DNA
        mp = IMP.atom.read_pdb(self.open_input_file("single_dna.pdb"),
                               m, IMP.atom.NonWaterPDBSelector())
        ps = IMP.atom.get_by_type(mp, IMP.atom.ATOM_TYPE);
        self.assertEqual(len(ps), 3011)

    def test_read_het(self):
        """Check reading a pdb with one protein and a hetatm"""
        m = IMP.Model()

        #! read PDB
        mp = IMP.atom.read_pdb(self.open_input_file("1DQK.pdb"),
                               m, IMP.atom.NonWaterPDBSelector())
        ps = IMP.atom.get_by_type(mp, IMP.atom.ATOM_TYPE);
        self.assertEqual(len(ps), 4060)
        #IMP.atom.show_molecular_hierarchy(mp)
        IMP.atom.show(mp)
        IMP.atom.add_bonds(mp)
        bds = IMP.atom.get_internal_bonds(mp)
        #self.assertEqual(bds.size(), 1020)
        IMP.atom.add_radii(mp)
        IMP.atom.show_molecular_hierarchy(mp)
        # read another PDB
        mp = IMP.atom.read_pdb(self.open_input_file("1aon.pdb"),
                              m, IMP.atom.NonWaterPDBSelector())
        ps = IMP.atom.get_by_type(mp, IMP.atom.ATOM_TYPE);
        self.assertEqual(len(ps), 58870)

    def test_read_non_water(self):
        """Check that the default pdb reader skips waters"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        mp= IMP.atom.read_pdb(self.open_input_file("protein_water.pdb"),
                              m)
        a= IMP.atom.get_leaves(mp)
        IMP.atom.write_pdb(mp, self.get_tmp_file_name("water_write.pdb"))
        self.assertEqual(len(a), 13328)
    def test_read_non_hydrogen(self):
        """Check that the Hydrogen selector can identify all hydrogens"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        mp= IMP.atom.read_pdb(self.open_input_file("hydrogen.pdb"),
                              m, IMP.atom.HydrogenPDBSelector())
        a= IMP.atom.get_leaves(mp)
        self.assertEqual(len(a), 22)
    def _test_sel_logic(self):
        m= IMP.Model()
        mp= IMP.atom.read_pdb(self.open_input_file("hydrogen.pdb"),
                              m, IMP.atom.HydrogenPDBSelector())
        a= IMP.atom.get_leaves(mp)
        mpn= IMP.atom.read_pdb(self.open_input_file("hydrogen.pdb"),
                              m, IMP.atom.NotPDBSelector(IMP.atom.HydrogenPDBSelector()))
        an= IMP.atom.get_leaves(mpn)
        mpb= IMP.atom.read_pdb(self.open_input_file("hydrogen.pdb"),
                              m, IMP.atom.OrPDBSelector(IMP.atom.NotPDBSelector(IMP.atom.HydrogenPDBSelector()), IMP.atom.HydrogenPDBSelector()))
        ab= IMP.atom.get_leaves(mpb)
        self.assertEqual(len(ab), len(an)+len(a))

    def test_pyimpl(self):
        """Test PDBSelectors implemented in Python"""
        class my_selector(IMP.atom.PDBSelector):
            def __call__(self, ln):
                return ln.startswith("ATOM")

        m= IMP.Model()
        mp= IMP.atom.read_pdb(self.open_input_file("hydrogen.pdb"),
                              m, IMP.atom.ATOMPDBSelector())
        mp_py = IMP.atom.read_pdb(self.open_input_file("hydrogen.pdb"),
                                  m, my_selector())

        l= IMP.atom.get_leaves(mp)
        self.assertEqual(len(l), 24)
        l_py = IMP.atom.get_leaves(mp_py)
        self.assertEqual(len(l), len(l_py))

    def test_read_non_prob(self):
        """Check that problem lines are read properly"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        mp= IMP.atom.read_pdb(self.open_input_file("problem_lines.pdb"), m)
        a= IMP.atom.get_leaves(mp)
        self.assertEqual(len(a), 1)

if __name__ == '__main__':
    IMP.test.main()
