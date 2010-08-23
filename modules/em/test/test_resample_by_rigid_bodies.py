### Test case:
# Open three proteins
# Generate density maps for each
# Create a fit restraint from the density maps
# Use evaluate() (either in model or in whatever)
# Do this in three ways: 1) as just a bunch of particles (get_leaves on all of them)
#                        2) as three rigid bodies
#                        3) as a mix
# In all three combos, check to make sure resulting RMSD is the same.
import IMP
import IMP.em
import IMP.test
import unittest

class ResamplingTest(IMP.test.TestCase):
    """Class to test transforming EM maps without resampling"""

    def load_proteins(self):
        fnames=["1atiB01.pdb","1arsA01.pdb","1ab4A02.pdb"]
        print "start load proteins"
        self.radius_key = IMP.core.XYZR.get_default_radius_key()
        self.weight_key = IMP.atom.Mass.get_mass_key()
        self.mhs=[] #3 molecular hierarchies
        self.pss=[] #3 particles
        self.mhs_copy=[] #3 copies of the molecular hierarchies used later as rigid bodies
        self.rbs_of_copy=IMP.core.RigidBodies() #3 rigid bodies
        sel=IMP.atom.CAlphaPDBSelector()
        for n,fn in enumerate(fnames):
            self.mhs.append(IMP.atom.read_pdb(self.open_input_file(fn),
                                              self.imp_model,sel))
            self.mhs_copy.append(IMP.atom.read_pdb(self.open_input_file(fn),
                                              self.imp_model,sel))

            IMP.atom.add_radii(self.mhs[n])
            IMP.atom.add_radii(self.mhs_copy[n])
            self.pss.append(IMP.Particles(IMP.core.get_leaves(self.mhs[n])))
            self.rbs_of_copy.append(IMP.atom.setup_as_rigid_body(self.mhs_copy[n]))
        print "end load proteins"
    def setUp(self):
        """Build test model and optimizer"""
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.SILENT)#VERBOSE)
        self.imp_model = IMP.Model()
        self.load_proteins()
        self.rb_refiner=IMP.core.LeavesRefiner(IMP.atom.Hierarchy.get_traits())
    def test_resample(self):
        """test resampling with and without rigid bodies"""
        #load as lots of particles, generate EM map, use it to define restraint
        print "start test resample"
        self.ps_all=IMP.Particles() #all the molecules together as one particle
        for n in xrange(3):
            self.ps_all+=self.pss[n]
        print "start test resample ==1"
        map=IMP.em.particles2density(self.ps_all,8,1.5)
        print "start test resample ==2"
        map.calcRMS()
        print "start test resample ==3"
        self.restr_ps_all=IMP.em.FitRestraint(self.ps_all,map,self.rb_refiner,self.radius_key,self.weight_key,1)
        print "start test resample ==4"
        self.restr_rb_all_fast=IMP.em.FitRestraint(self.rbs_of_copy,map,self.rb_refiner,self.radius_key,self.weight_key,1)
        print "start test resample ==5"
        self.imp_model.add_restraint(self.restr_ps_all)
        self.imp_model.add_restraint(self.restr_rb_all_fast)
        print "before eval1"
        score1=self.restr_ps_all.evaluate(False)
        print "before eval2"
        score2=self.restr_rb_all_fast.evaluate(False)
        print "evaluate ps_all before transform: ",score1
        print "evaluate rb_all before transform fast: ",score2
        self.assertInTolerance(score1,score2,
                               0.05)
        for j in range(5):
            rand_t=[]
            for i in range(3):
                rand_t.append(IMP.algebra.Transformation3D(
                    IMP.algebra.get_random_rotation_3d(),
                    IMP.algebra.get_random_vector_in(
                    IMP.algebra.BoundingBox3D(
                    IMP.algebra.Vector3D(-2.,-2.,-2.),
                    IMP.algebra.Vector3D(2.,2.,2.)))))
            for i in range(3):
                for x in IMP.core.XYZsTemp(IMP.core.get_leaves(self.mhs[i])):
                    x.set_coordinates(rand_t[i].get_transformed(x.get_coordinates()))
                IMP.core.transform(self.rbs_of_copy[i],rand_t[i])
            score1=self.restr_ps_all.evaluate(False)
            score2=self.restr_rb_all_fast.evaluate(False)
            print "evaluate ps_all after transform: ", j , " : ", score1
            print "evaluate rb_all after transform fast: ",j , " : ", score2
            self.assertInTolerance(score1,score2,
                                   0.1)
            for i in range(3):
                for x in IMP.core.XYZsTemp(IMP.core.get_leaves(self.mhs[i])):
                    x.set_coordinates(rand_t[i].get_inverse().get_transformed(x.get_coordinates()))
                IMP.core.transform(self.rbs_of_copy[i],rand_t[i].get_inverse())
        self.imp_model.remove_restraint(self.restr_ps_all)
        self.imp_model.remove_restraint(self.restr_rb_all_fast)

    def _test_resampling_derivatives(self):
        """Test derivatives with and without rigid bodies"""
        #load as lots of particles, generate EM map, use it to define restraint
        self.ps_all=IMP.Particles() #all the molecules together as one particle
        for n in xrange(3):
            self.ps_all+=self.pss[n]
        map=IMP.em.particles2density(self.ps_all,8,1.5)
        map.calcRMS()
        self.restr_ps_all=IMP.em.FitRestraint(self.ps_all,map,self.rb_refiner,self.radius_key,self.weight_key,1)
        self.restr_rb_all_fast=IMP.em.FitRestraint(self.rbs_of_copy,map,self.rb_refiner,self.radius_key,self.weight_key,1)
        self.restr_rb_all_slow=IMP.em.FitRestraint(self.rbs_of_copy,map,self.rb_refiner,self.radius_key,self.weight_key,1)
        self.imp_model.add_restraint(self.restr_ps_all)
        self.imp_model.add_restraint(self.restr_rb_all_fast)
        self.imp_model.add_restraint(self.restr_rb_all_slow)
        score1=self.restr_ps_all.evaluate(True)
        score2=self.restr_rb_all_fast.evaluate(True)
        self.assertInTolerance(score1,score2,
                               0.05)
        self.imp_model.remove_restraint(self.restr_ps_all)
        self.imp_model.remove_restraint(self.restr_rb_all_fast)

    def _test_fast_local_refinement(self):
        """test that local rigid fitting work well with rigid bodies"""
        self.ps_all=IMP.Particles() #all the molecules together as one particle
        for n in xrange(3):
            self.ps_all+=self.pss[n]
        d_map=IMP.em.particles2density(self.ps_all,8,1.5)
        d_map.calcRMS()
        fnames=["1atiB01.pdb","1arsA01.pdb","1ab4A02.pdb"]
        self.radius_key = IMP.core.XYZR.get_default_radius_key()
        self.weight_key = IMP.atom.Mass.get_mass_key()
        self.mhs=[] #3 molecular hierarchies
        self.pss=[] #3 particles
        self.mhs_copy=[] #3 copies of the molecular hierarchies used later as rigid bodies
        sel=IMP.atom.NonWaterPDBSelector()
        mh = IMP.atom.read_pdb(self.open_input_file("1atiB01.pdb"),self.imp_model,sel)
        IMP.atom.add_radii(mh)
        rb=IMP.atom.setup_as_rigid_body(mh)
        ps=IMP.Particles(IMP.core.get_leaves(IMP.atom.Hierarchy(mh)))
        score_before=IMP.em.compute_fitting_score(ps,d_map)
        rand_t = IMP.algebra.Transformation3D(
            IMP.algebra.get_random_rotation_3d(),
            IMP.algebra.get_random_vector_in(
            IMP.algebra.BoundingBox3D(
            IMP.algebra.Vector3D(-5.,-5.,-5.),
            IMP.algebra.Vector3D(5.,5.,5.))))
        IMP.core.transform(self.rbs_of_copy[0],rand_t)
        fs=IMP.em.local_rigid_fitting(rb,self.radius_key,self.weight_key,d_map,
                                      None,3,5,20,2,3,True)
        score_after=IMP.em.compute_fitting_score(ps,d_map)
        print "score_after:",score_after," score_before:",score_before
        self.assertInTolerance(score_after,score_before,0.05)
        self.assert_(fs.get_number_of_solutions()>0)
        print fs.get_score(0)
if __name__ == '__main__':
    unittest.main()
