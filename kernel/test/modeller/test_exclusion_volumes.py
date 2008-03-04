import modeller, unittest
import modeller
import modeller.optimizers
import os
import IMP
import IMP.modeller_intf
import IMP.test

radius = IMP.FloatKey("radius")

class ExclusionVolumeRestraintTests(IMP.test.TestCase):
    """Test exclusion volume restraints"""

    def setUp(self):
        """set up Modeller with exclusion volumes restraints """
        modeller.log.level(0,0,0,0,0)

        self.env = modeller.environ()
        self.env.io.atom_files_directory = '../data/'
        self.env.edat.dynamic_sphere = False
        self.env.libs.topology.read(file='$(LIB)/top_heav.lib')
        self.env.libs.parameters.read(file='$(LIB)/par.lib')

        self.imp_model = IMP.Model()
        self.particles = []
        self.restraint_sets = []
        self.rsrs = []

        self.t = self.env.edat.energy_terms
        self.t.append(IMP.modeller_intf.IMPRestraints(self.imp_model,
                                                      self.particles))

        self.modeller_model = IMP.modeller_intf.create_particles(12, self.env,
                                                                 self.imp_model,
                                                                 self.particles)
        p1 = self.particles[0]
        p1.add_attribute(radius, 1.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 1)
        p1.add_attribute(IMP.IntKey("id"), 1)

        p1 = self.particles[1]
        p1.add_attribute(radius, 1.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 1)
        p1.add_attribute(IMP.IntKey("id"), 2)

        p1 = self.particles[2]
        p1.add_attribute(radius, 1.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 1)
        p1.add_attribute(IMP.IntKey("id"), 3)

        p1 = self.particles[3]
        p1.add_attribute(radius, 1.5, False)
        p1.add_attribute(IMP.IntKey("protein"), 1)
        p1.add_attribute(IMP.IntKey("id"), 4)

        p1 = self.particles[4]
        p1.add_attribute(radius, 1.5, False)
        p1.add_attribute(IMP.IntKey("protein"), 1)
        p1.add_attribute(IMP.IntKey("id"), 5)

        p1 = self.particles[5]
        p1.add_attribute(radius, 1.5, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 6)

        p1 = self.particles[6]
        p1.add_attribute(radius, 1.5, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 7)

        p1 = self.particles[7]
        p1.add_attribute(radius, 2.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 8)

        p1 = self.particles[8]
        p1.add_attribute(radius, 2.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 9)

        p1 = self.particles[9]
        p1.add_attribute(radius, 2.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 10)

        p1 = self.particles[10]
        p1.add_attribute(radius, 2.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 11)

        p1 = self.particles[11]
        p1.add_attribute(radius, 2.0, False)
        p1.add_attribute(IMP.IntKey("protein"), 2)
        p1.add_attribute(IMP.IntKey("id"), 12)

        self.atmsel = modeller.selection(self.modeller_model)

        self.opt = modeller.optimizers.conjugate_gradients()


    def test_exclusion_volumes_one_list(self):
        """Test exclusion volumes (intra-protein).
           All particles in a single protein should be connected but should
           not be closer than allowed by their VDW radii."""
        rs = IMP.RestraintSet("one_list")
        self.restraint_sets.append(rs)
        self.imp_model.add_restraint(rs)

        # add connectivity restraints

        particle_indexes1 = IMP.Ints()
        particle_indexes2 = IMP.Ints()
        rsrs = []

        # connect 2 proteins together by two beads
        particle_indexes1.clear()
        for i in range(0, 5):
            particle_indexes1.push_back(i)
        particle_indexes2.clear()
        for i in range(5, 12):
            particle_indexes2.push_back(i)

        # connect the beads within the two proteins together
        for i in range(0, 4):
            p1 = self.particles[i]
            p2 = self.particles[i+1]
            mean = p1.get_value(radius) + p2.get_value(radius)
            sf = IMP.HarmonicUpperBound(mean, 0.1)
            rsrs.append(IMP.DistanceRestraint(sf, p1, p2))

        for i in range(5, 11):
            p1 = self.particles[i]
            p2 = self.particles[i+1]
            mean = p1.get_value(radius) + p2.get_value(radius)
            sf = IMP.HarmonicUpperBound(mean, 0.1)
            rsrs.append(IMP.DistanceRestraint(sf, p1, p2))

        # create the exclusion volume for each protein
        score_func_params_lb = IMP.BasicScoreFuncParams("harmonic_lower_bound", 0.0, 0.1)
        rsrs.append(IMP.ExclusionVolumeRestraint(self.imp_model, particle_indexes1, radius, score_func_params_lb))
        rsrs.append(IMP.ExclusionVolumeRestraint(self.imp_model, particle_indexes2, radius, score_func_params_lb))

        # add restraints
        for i in range(len(rsrs)):
            rs.add_restraint(rsrs[i])

        self.atmsel.randomize_xyz(deviation=100.0)
        new_mdl = self.opt.optimize (self.atmsel, max_iterations=100, actions=None)
        self.modeller_model.write (file='out_exclusion_volume_one_list.pdb', model_format='PDB')

        coords = self.load_coordinates('out_exclusion_volume_one_list.pdb')
        os.unlink('out_exclusion_volume_one_list.pdb')

        # check min distances for intra-protein pairs
        for i in range(0, 4):
            for j in range(i+1, 5):
                d = self.get_distance(coords[i], coords[j]) - self.particles[i].get_value(radius) - self.particles[j].get_value(radius)
                self.assert_(d > -0.05, "particles "+str(i)+" and "+str(j)+" are too close together.")

        for i in range(5, 11):
            for j in range(i+1, 12):
                d = self.get_distance(coords[i], coords[j]) - self.particles[i].get_value(radius) - self.particles[j].get_value(radius)
                self.assert_(d > -0.05, "particles "+str(i)+" and "+str(j)+" are too close together.")


    def test_exclusion_volumes_two_lists(self):
        """Test exclusion volumes (intra- and inter-protein).
           All particles in a single protein should be connected and the two
           proteins should be connected in four locations but beads should
           not be closer than allowed by their VDW radii."""
        rs = IMP.RestraintSet("two_lists")
        self.restraint_sets.append(rs)
        self.imp_model.add_restraint(rs)

        # add connectivity restraints

        particle_indexes1 = IMP.Ints()
        particle_indexes2 = IMP.Ints()
        rsrs = []

        # connect 2 proteins together by two beads
        particle_indexes1.clear()
        for i in range(0, 5):
            particle_indexes1.push_back(i)
        particle_indexes2.clear()
        for i in range(5, 12):
            particle_indexes2.push_back(i)

        # connect the beads within the two proteins together
        for i in range(0, 4):
            p1 = self.particles[i]
            p2 = self.particles[i+1]
            mean = p1.get_value(radius) + p2.get_value(radius)
            sf = IMP.HarmonicUpperBound(mean, 0.1)
            rsrs.append(IMP.DistanceRestraint(sf, p1, p2))

        for i in range(5, 11):
            p1 = self.particles[i]
            p2 = self.particles[i+1]
            mean = p1.get_value(radius) + p2.get_value(radius)
            sf = IMP.HarmonicUpperBound(mean, 0.1)
            rsrs.append(IMP.DistanceRestraint(sf, p1, p2))

        # create the exclusion volume for each protein
        score_func_params_lb = IMP.BasicScoreFuncParams("harmonic_lower_bound", 0.0, 0.1)
        rsrs.append(IMP.ExclusionVolumeRestraint(self.imp_model, particle_indexes1, radius, score_func_params_lb))
        rsrs.append(IMP.ExclusionVolumeRestraint(self.imp_model, particle_indexes2, radius, score_func_params_lb))

        # connect the beads within the two proteins together
        # get 4 distinct pairs
        num_connects = 4
        particle_reuse = False
        score_func_params_ub = IMP.BasicScoreFuncParams("harmonic_upper_bound",
                                                        0.0, 0.1)
        rsrs.append(IMP.PairConnectivityRestraint(self.imp_model, particle_indexes1, particle_indexes2, radius, score_func_params_ub, num_connects, particle_reuse))

        # create the exclusion volume for each protein
        score_func_params_lb = IMP.BasicScoreFuncParams("harmonic_lower_bound", 0.0, 0.1)
        rsrs.append(IMP.ExclusionVolumeRestraint(self.imp_model, particle_indexes1, particle_indexes2, radius, score_func_params_lb))

        # add restraints
        for i in range(len(rsrs)):
            rs.add_restraint(rsrs[i])

        self.atmsel.randomize_xyz(deviation=100.0)
        new_mdl = self.opt.optimize (self.atmsel, max_iterations=200, actions=None)
        self.modeller_model.write (file='out_exclusion_volume_two_lists.pdb', model_format='PDB')

        coords = self.load_coordinates('out_exclusion_volume_two_lists.pdb')
        os.unlink('out_exclusion_volume_two_lists.pdb')

        # check min distances for intra-protein and inter-protein pairs
        for i in range(0, 11):
            for j in range(i+1, 12):
                d = self.get_distance(coords[i], coords[j]) - self.particles[i].get_value(radius) - self.particles[j].get_value(radius)
                self.assert_(d > -0.05, "particles "+str(i)+" and "+str(j)+" are too close together.")

if __name__ == '__main__':
    unittest.main()
