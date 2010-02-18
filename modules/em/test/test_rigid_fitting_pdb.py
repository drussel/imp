import unittest
import sys,os
import IMP
import IMP.em
import IMP.test
import IMP.core
import IMP.atom
import IMP.helper

class ProteinRigidFittingTest(IMP.test.TestCase):
    """Class to test EM correlation restraint"""

    def load_density_map(self):
        mrw = IMP.em.MRCReaderWriter()
        self.scene = IMP.em.read_map(self.get_input_file_name("1z5s_10.mrc"), mrw)
        self.scene.get_header_writable().set_resolution(10.)
        self.scene.update_voxel_size(2.0)
        self.scene.set_origin(34.0,8.0,-92.0)
    def load_protein(self,pdb_filename):
        self.mp= IMP.atom.read_pdb(self.open_input_file(pdb_filename),
                              self.imp_model, IMP.atom.CAlphaPDBSelector())#IMP.atom.NonWaterSelector())
        IMP.atom.add_radii(self.mp)
        self.radius_key = IMP.core.XYZR.get_default_radius_key()
        self.weight_key = IMP.atom.Mass.get_mass_key()
        self.particles = IMP.core.get_leaves(self.mp)

    def setUp(self):
        """Build test model and optimizer"""
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.SILENT)
        self.imp_model = IMP.Model()
        self.load_density_map()
        self.load_protein("1z5s_A.pdb")

    def test_em_local_rigid_fitting_around_point(self):
        """Check that local rigid fitting around a point works"""
        if sys.platform == 'sunos5':
            print >> sys.stderr, "Test skipped: too slow to run on Solaris"
            return
        check = IMP.get_check_level()
        css= IMP.core.ChecksScoreState(.05)
        self.imp_model.add_score_state(css)
        try:
            # This test is super-slow, so disable checks to speed it up a little
            #IMP.set_check_level(IMP.NONE)
            #create a rigid body
            rb_p = IMP.Particle(self.imp_model)
            rb_d = IMP.core.RigidBody.setup_particle(
                                      rb_p,IMP.core.XYZs(self.particles))
            ref_trans = rb_d.get_transformation()
            fr = IMP.em.FittingSolutions()
            IMP.em.local_rigid_fitting_around_point(
                rb_d,self.radius_key, self.weight_key,
                self.scene,IMP.algebra.Vector3D(87.0856,71.7701,-56.3955),
                fr,None,
                3,5,50)

            #todo - add test that if you apply the transformation on the
            # original configuration you get the same result
            # (in rmsd and score)

            #second, test that the optimization gets you close.
            score = self.imp_model.evaluate(False)
            self.assert_(fr.get_number_of_solutions() == 3,
                         "not enough solutions returned")
            self.assert_(fr.get_score(0) < fr.get_score(1),
                         "solutions are not sorted")
            for i in xrange(3):
                print fr.get_score(i)
                self.assert_(fr.get_score(i) < 1.0, "wrong CC values")
        finally:
            IMP.set_check_level(check)
        print "things were checked " +str(css.get_number_of_checked()) + " times"

if __name__ == '__main__':
    unittest.main()
