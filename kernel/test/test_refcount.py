import unittest
import IMP
import IMP.test

# We probably shouldn't require IMP.core to test the kernel; temporary hack
#import IMP.core

class RefCountTests(IMP.test.TestCase):
    """Test refcounting of particles"""

    def test_simple(self):
        """Check that ref counting of particles works within python"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        refcnt.assert_number(1)
        p= IMP.Particle(m)
        refcnt.assert_number(2)
        del p
        refcnt.assert_number(2)
        del m
        # Deleting Model should delete all Particles too
        refcnt.assert_number(0)

    def test_removal(self):
        """Check that ref counting works with removing particles"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        p= IMP.Particle(m)
        refcnt.assert_number(2)
        m.remove_particle(p)
        # Particle should not disappear yet since Python still has a reference
        refcnt.assert_number(2)
        self.assert_(not p.get_is_active(), "Removed particle is still active")
        del p
        refcnt.assert_number(1)
        del m
        refcnt.assert_number(0)

    def test_delete_model_constructor(self):
        """Constructed Python Particles should survive model deletion"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        p= IMP.Particle(m)
        self.assertEqual(p.get_ref_count(), 2)
        refcnt.assert_number(2)
        # New particle p should not go away until we free the Python reference
        del m
        refcnt.assert_number(1)
        self.assertEqual(p.get_ref_count(), 1)
        del p
        refcnt.assert_number(0)

    def temporarily_disabled_test_delete_model_iterator(self):
        """Python Particles from iterators should survive model deletion"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        IMP.Particle(m)
        # Now create new Python particle p from C++ iterator
        # (not the Python IMP.Particle() constructor)
        # This calls swig::from() internally, which is modified by template
        # specialization in our SWIG interface.
        p = m.get_particles().__iter__().value()
        # Python reference p plus C++ reference from m
        self.assertEqual(p.get_ref_count(), 2)
        del m
        # Now only the Python reference p should survive
        self.assertEqual(p.get_ref_count(), 1)
        refcnt.assert_number(1)
        del p
        refcnt.assert_number(0)

    def test_delete_model_accessor(self):
        "Python Particles from vector accessors should survive model deletion"
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        IMP.Particle(m)
        # Now create new Python particle p from a C++ vector accessor
        # (front(), back(), [], etc.)
        # (not the Python IMP.Particle() constructor)
        # These accessors call specific methods in the SWIG wrapper which
        # are modified by typemaps in our interface.
        p = m.get_particles()[0]
        # Python reference p plus C++ reference from m
        self.assertEqual(p.get_ref_count(), 2)
        del m
        # Now only the Python reference p should survive
        self.assertEqual(p.get_ref_count(), 1)
        refcnt.assert_number(1)
        del p
        refcnt.assert_number(0)

    def _test_shared(self):
        """Check that ref counting works with shared particles"""
        refcnt = IMP.test.RefCountChecker(self)
        m= IMP.Model()
        p= IMP.Particle(m)
        d= IMP.core.XYZ.setup_particle(p)
        del d

        mc= IMP.core.ListSingletonContainer()
        mc.add_particle(p)
        # also have the score state now
        refcnt.assert_number(3)
        m.remove_particle(p)
        self.assertEqual(m.get_number_of_particles(), 0)
        refcnt.assert_number(3)
        del p
        refcnt.assert_number(3)
        mc.clear_particles()
        self.assertEqual(mc.get_number_of_particles(), 0)
        refcnt.assert_number(2)
        del mc
        refcnt.assert_number(1)

    def test_skip(self):
        """Check that removed particles are skipped"""
        m= IMP.Model()
        p= IMP.Particle(m)
        ps= m.get_particles()
        self.assertEqual(len(ps), 1, "Should only be 1 particle")
        m.remove_particle(p)
        ps= m.get_particles()
        self.assertEqual(len(ps), 0, "Should be no particles")


if __name__ == '__main__':
    unittest.main()
