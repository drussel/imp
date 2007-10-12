import unittest
import IMP.utils
import IMP.test, IMP

# Class to test steepest descent optimizer in IMP
class test_sd_optimizer(IMP.test.IMPTestCase):
    """test steepest descent optimizer in IMP """

    def setUp(self):
        """set up distance restraints and create optimizer """

        self.imp_model = IMP.Model()
        self.particles = []
        self.restraint_sets = []
        self.rsrs = []

        # create particles 0 - 1
        self.particles.append(IMP.utils.XYZParticle(self.imp_model,
                                                    -43.0, 65.0, 93.0))
        self.particles.append(IMP.utils.XYZParticle(self.imp_model,
                                                    20.0, 74.0, -80.0))
        self.particles.append(IMP.utils.XYZParticle(self.imp_model,
                                                    4.0, -39.0, 26.0))

        p1 = self.particles[0]
        p1.add_float("radius", 1.0, False)
        p1 = self.particles[1]
        p1.add_float("radius", 2.0, False)
        p1 = self.particles[2]
        p1.add_float("radius", 3.0, False)

        # separate 3 particles by their radii
        score_func_params = IMP.BasicScoreFuncParams("harmonic", 0.0, 0.1)

        self.rsrs.append(IMP.DistanceRestraint(self.imp_model, self.particles[0], self.particles[1], "radius", score_func_params))
        self.rsrs.append(IMP.DistanceRestraint(self.imp_model, self.particles[1], self.particles[2], "radius", score_func_params))
        self.rsrs.append(IMP.DistanceRestraint(self.imp_model, self.particles[0], self.particles[2], "radius", score_func_params))

        # add restraints
        rs = IMP.RestraintSet("distance_rsrs")
        self.imp_model.add_restraint_set(rs)
        self.restraint_sets.append(rs)
        for i in range(len(self.rsrs)):
            rs.add_restraint(self.rsrs[i])

        self.steepest_descent = IMP.SteepestDescent()




    def test_sd_optimizer1(self):
        """ test that optimizer brings particles together """

        self.steepest_descent.optimize(self.imp_model, 50)

        for i in range(0, 2):
            for j in range(i+1, 3):
                dist = self.IMP_Distance(self.particles, i, j) - self.particles[i].get_float("radius") - self.particles[j].get_float("radius")
                self.assertAlmostEqual(0.0, dist, places=2)


    def test_sd_optimizer2(self):
        """ test that optimizer spreads particles apart """

        self.particles[0].set_x(0.0)
        self.particles[0].set_y(0.0)
        self.particles[0].set_z(0.0)

        self.particles[1].set_x(0.0)
        self.particles[1].set_y(0.0)
        self.particles[1].set_z(0.0)

        self.particles[2].set_x(0.0)
        self.particles[2].set_y(0.0)
        self.particles[2].set_z(0.0)

        self.steepest_descent.optimize(self.imp_model, 50)

        for i in range(0, 2):
            for j in range(i+1, 3):
                dist = self.IMP_Distance(self.particles, i, j) - self.particles[i].get_float("radius") - self.particles[j].get_float("radius")
                self.assertAlmostEqual(0.0, dist, places=2)


if __name__ == '__main__':
    unittest.main()
