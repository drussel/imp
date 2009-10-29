import sys
import unittest
import IMP
import IMP.test
import IMP.domino
import IMP.core
import IMP.atom
import IMP.helper
import time
import my_helpers

class TransformationUtilsTests(IMP.test.TestCase):

    def __set_rigid_bodies__(self):
        self.rbs=my_helpers.set_rigid_bodies(self.mhs,self.mdl)

    def __set_components__(self):
        self.mhs = my_helpers.read_components([
            self.get_input_file_name("1z5s_A.pdb"),
            self.get_input_file_name("1z5s_B.pdb"),
            self.get_input_file_name("1z5s_C.pdb"),
            self.get_input_file_name("1z5s_D.pdb")],self.mdl)

    def __set_components_copy__(self):
        self.mhs_copy = my_helpers.read_components([
            self.get_input_file_name("1z5s_A.pdb"),
            self.get_input_file_name("1z5s_B.pdb"),
            self.get_input_file_name("1z5s_C.pdb"),
            self.get_input_file_name("1z5s_D.pdb")],self.mdl)

    def setUp(self):

        """Set up model and particles"""
        IMP.test.TestCase.setUp(self)
        self.mdl = IMP.Model()
        IMP.set_check_level(IMP.NONE)
        self.__set_components__()
        self.__set_components_copy__()
        self.__set_rigid_bodies__()
        self.tu = IMP.domino.TransformationUtils(self.rbs,True)

    def test_transformation_on_rigid_bodies(self):
        '''test that move2state sets the correct transformation for rigid bodies.
           For now Model.evaluate() makes sure that the transformation actually occurs,
           in the near future will be a better solution.'''
        tu = IMP.domino.TransformationUtils(self.rbs)
        for j in range(5):
            #transform all rigid bodies
            for i in range(4):
                trans = my_helpers.create_random_transformation()
                state_p=IMP.Particle(self.mdl)
                IMP.domino.Transformation.setup_particle(state_p,trans)

                #transform the copy molecule
                xyz_copy=IMP.core.XYZs(IMP.core.get_leaves(self.mhs_copy[i]))
                for xyz in xyz_copy:
                    xyz.set_coordinates(trans.transform(xyz.get_coordinates()))


                #transform the rigid body
                xyz_orig=IMP.core.XYZs(IMP.core.get_leaves(self.mhs[i]))
                self.tu.move2state(self.rbs[i],state_p)
            self.mdl.evaluate(False) #to make sure that the rigid bodies score states are updated
            #check that the rmsd is 0
            self.assert_(IMP.atom.rmsd(xyz_copy,xyz_orig) < 0.001,
                         "the molecules are expected to have the same placement")
            #return the copy to ref for the next round
            for xyz in xyz_copy:
                xyz.set_coordinates(trans.get_inverse().transform(xyz.get_coordinates()))




if __name__ == '__main__':
    unittest.main()
