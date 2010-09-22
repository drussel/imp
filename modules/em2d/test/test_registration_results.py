import IMP
import IMP.test
import IMP.algebra
import IMP.core
import IMP.em
import IMP.em2d


class RegistrationResultTests(IMP.test.TestCase):
    def test_rotation_error(self):
        """ Test the rotation error between 2 RegistrationResults classes"""
        rr1=IMP.em2d.RegistrationResult(0.5,0.2,0.1,-1,-3)
        rr2=IMP.em2d.RegistrationResult(0.5,0.2,0.3,-1,-3)
        angle = IMP.em2d.rotation_error(rr1,rr2)
        self.assertAlmostEqual(angle,0.2, delta=0.0001)

    def test_translation_error(self):
        """ test the shift error between 2 RegistrationResults classes"""
        rr1=IMP.em2d.RegistrationResult(0.5,0.2,0.1,-1,-3)
        rr2=IMP.em2d.RegistrationResult(0.5,0.2,0.3,-8,6)
        dist = IMP.em2d.shift_error(rr1,rr2)
        self.assertAlmostEqual(dist,11.4017, delta=0.0001)

    def test_registration_quaternion(self):
        """ test back a forth of the Rotation in RegistrationResult class"""
        import random
        import math
        random.seed()
        R1 = IMP.algebra.get_random_rotation_3d()
        reg=IMP.em2d.RegistrationResult()
        reg.set_rotation(R1)
        phi=reg.get_Phi()
        theta=reg.get_Theta()
        psi=reg.get_Psi()
        R2=IMP.algebra.get_rotation_from_fixed_zyz(phi,theta,psi)
        q1=R1.get_quaternion()
        q2=R2.get_quaternion()
        for i in xrange(0,4):
            self.assertAlmostEqual(q1[i],q2[i], delta=0.001,
                   msg="Error in Registration rotation back and forth")

    def test_even_registration_results(self):
        """ Test the generation of evenly distributed RegistrationResults"""
        Regs1 = IMP.em2d.evenly_distributed_registration_results(3)
        Regs2 = IMP.em2d.read_registration_results(
                      self.get_input_file_name("1z5s-projections.params"))

        for j in xrange(0,len(Regs1)):
            q1=Regs1[j].get_rotation().get_quaternion()
            q2=Regs2[j].get_rotation().get_quaternion()
            for i in xrange(0,4):
                self.assertAlmostEqual(q1[i],q2[i], delta=0.001,
                      msg="Error in generation of evenly distributed "
                          "RegistrationResults")


if __name__ == '__main__':
    IMP.test.main()
