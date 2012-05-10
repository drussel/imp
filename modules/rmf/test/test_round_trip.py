import unittest
import IMP.rmf
import IMP.test
import RMF
from IMP.algebra import *

class GenericTest(IMP.test.TestCase):
    def _show(self, g):
        for i in range(0, g.get_number_of_children()):
            print i, g.get_child_name(i), g.get_child_is_group(i)
    def test_rt(self):
        """Test round trip"""
        m= IMP.Model()
        print "reading pdb"
        name=self.get_tmp_file_name("test_rt.rmf")
        h= IMP.atom.read_pdb(self.get_input_file_name("simple.pdb"), m,
                             IMP.atom.NonAlternativePDBSelector())
        IMP.set_log_level(IMP.SILENT)
        IMP.atom.add_bonds(h)
        print "done"
        IMP.set_log_level(IMP.VERBOSE)
        print "writing hierarchy"
        f= RMF.create_rmf_file(name)
        IMP.rmf.add_hierarchy(f, h)
        print "reopening"
        del f
        print "after closing"
        print RMF.get_open_hdf5_handle_names()
        f= RMF.open_rmf_file_read_only(name)
        print "reading"
        print f, type(f)
        h2=IMP.rmf.create_hierarchies(f, m)
        IMP.atom.show_molecular_hierarchy(h2[0])
        print "checking"
        self.assertEqual(len(h2), 1)
        self.assertEqual(len(IMP.atom.get_leaves(h)),
                         len(IMP.atom.get_leaves(h2[0])))
        self.assertAlmostEqual(IMP.atom.get_mass(h),
                               IMP.atom.get_mass(h2[0]), delta=1e-4)
        if hasattr(IMP.atom, 'get_surface_area'):
            self.assertAlmostEqual(IMP.atom.get_surface_area(h),
                                   IMP.atom.get_surface_area(h2[0]), delta=1e-4)
            self.assertAlmostEqual(IMP.atom.get_volume(h),
                                   IMP.atom.get_volume(h2[0]), delta=1e-4)


    def test_part1(self):
        """Test round trip 1"""
        m= IMP.Model()
        print "reading pdb"
        h= IMP.atom.read_pdb(self.get_input_file_name("simple.pdb"), m,
                             IMP.atom.NonAlternativePDBSelector())
        IMP.set_log_level(IMP.SILENT)
        IMP.atom.add_bonds(h)
        print "done"
        IMP.set_log_level(IMP.VERBOSE)
        print "writing hierarchy"
        name=self.get_tmp_file_name("test_rt_parts.rmf")
        f= RMF.create_rmf_file(name)
        print f, type(f)
        IMP.rmf.add_hierarchy(f, h)
        del f
        f= RMF.open_rmf_file_read_only(name)
        print "reading"
        print f, type(f)
        h2=IMP.rmf.create_hierarchies(f, m)
        self.assertEqual(len(h2), 1)
        del f

        m= IMP.Model()
        print "reopening"
        f= RMF.open_rmf_file_read_only(name)
        print "reading"
        h2=IMP.rmf.create_hierarchies(f, m)

if __name__ == '__main__':
    unittest.main()
