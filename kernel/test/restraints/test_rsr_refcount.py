import unittest
import IMP
import IMP.test

# We probably shouldn't require IMP.core to test the kernel; temporary hack
import IMP.core

class RefCountTests(IMP.test.TestCase):
    """Test refcounting of restraints"""

    def test_simple(self):
        """Check reference counting of restraints"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        r= IMP.core.ConstantRestraint(1)
        s= IMP.core.RestraintSet()
        m.add_restraint(s)
        m.add_restraint(r)
        s.add_restraint(r)
        m.evaluate(False)
        refcnt.assert_number(3)
        # Model should hold a ref to restraints, so nothing should be freed
        # until it is
        del r
        refcnt.assert_number(3)
        del s
        refcnt.assert_number(3)
        del m
        refcnt.assert_number(0)

    def test_delete_model_constructor(self):
        """Constructed Python Restraints should survive model deletion"""
        refcnt = IMP.test.RefCountChecker(self)
        m = IMP.Model()
        r = IMP.core.RestraintSet()
        m.add_restraint(r)
        self.assertEqual(r.get_ref_count(), 2)
        refcnt.assert_number(2)
        # New restraint r should not go away until we free the Python reference
        del m
        refcnt.assert_number(1)
        self.assertEqual(r.get_ref_count(), 1)
        del r
        refcnt.assert_number(0)

    def test_delete_model_iterator(self):
        """Python Restraints from iterators should survive model deletion"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        r = IMP.core.RestraintSet()
        m.add_restraint(r)
        del r
        # Now create new Python Restraint r from C++ iterator
        # This calls swig::from() internally, which is modified by template
        # specialization in our SWIG interface.
        r = m.get_restraints().__iter__().value()
        # Python reference r plus C++ reference from m
        self.assertEqual(r.get_ref_count(), 2)
        del m
        # Now only the Python reference r should survive
        self.assertEqual(r.get_ref_count(), 1)
        refcnt.assert_number(1)
        del r
        refcnt.assert_number(0)

    def test_delete_model_accessor(self):
        "Python Restraints from vector accessors should survive model deletion"
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        r = IMP.core.RestraintSet()
        m.add_restraint(r)
        del r
        # Now create new Python Restraint r from a C++ vector accessor
        # These accessors call specific methods in the SWIG wrapper which
        # are modified by typemaps in our interface.
        r = m.get_restraints()[0]
        # Python reference r plus C++ reference from m
        self.assertEqual(r.get_ref_count(), 2)
        del m
        # Now only the Python reference r should survive
        self.assertEqual(r.get_ref_count(), 1)
        refcnt.assert_number(1)
        del r
        refcnt.assert_number(0)


if __name__ == '__main__':
    unittest.main()
