import sys
import IMP
import IMP.test
import my_optimizer

class DOMINOTests(IMP.test.TestCase):
    def setUp(self):
        """Set up model and particles"""
        IMP.test.TestCase.setUp(self)
        IMP.set_log_level(IMP.SILENT)
        self.num_particles=5
        self.sampler = my_optimizer.my_optimizer(
                         self.get_input_file_name("simple_jt1.txt"),
                         self.get_input_file_name("simple_jt1_restraints.txt"),
                         self.num_particles)
        self.infered_score=self.sampler.optimize()
    def test_global_min(self):
        try:
            min_score2 = self.sampler.exhaustive_search()
            self.assertEqual(self.infered_score, min_score2,
                       "the minimum score as calculated by the inference " \
                       + "differs from the one calculated by the exhaustive " \
                       + "search " + str(self.infered_score) + " != " \
                       + str(min_score2))
            self.assertEqual(self.infered_score, min_score2 , "the score of the minimum configuration as calculated by the inference differs from the one calculated by the model " + str(self.infered_score) + " != " + str(min_score2))
        except NotImplementedError, detail:
            print >> sys.stderr, detail


    def test_inference_1(self):
        score = -148.600001335
        self.assertLess(abs(self.infered_score -score), 0.1 , "the score of the minimum configuration as calculated by the inference is wrong " + str(self.infered_score) + " != " + str(score))


if __name__ == '__main__':
    IMP.test.main()
