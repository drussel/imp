import sys
import unittest
import IMP
import IMP.test
import my_optimizer
IMP.set_log_level(IMP.SILENT)
class DOMINOTests(IMP.test.TestCase):
    def setUp(self):
        """Set up model and particles"""
        IMP.test.TestCase.setUp(self)
        self.sampler = my_optimizer.my_optimizer(
                          self.get_input_file_name("simple_jt2.txt"),
                          self.get_input_file_name("simple_jt2_restraints.txt"),
                          8)
        self.infered_score = self.sampler.optimize()
    def test_global_min(self):
        try:
            min_score2 = self.sampler.exhaustive_search()
            print "=min_score2 " + str(min_score2)
            self.assert_(abs(self.infered_score - min_score2) < 0.1 , "the score of the minimum configuration as calculated by the inference differs from the one calculated by the model " + str(self.infered_score) + " != " + str(min_score2))
        except NotImplementedError, detail:
            print >> sys.stderr, detail

    def test_inference_2(self):
        score = -13.7 #this is the value of the exhaustive search
        self.assert_( abs(self.infered_score -score) < 0.1 , "the score of the minimum configuration as calculated by the inference is wrong " + str(self.infered_score) + " != " + str(score))

if __name__ == '__main__':
    unittest.main()
