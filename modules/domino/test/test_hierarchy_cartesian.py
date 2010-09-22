import sys
import IMP
import IMP.test
import IMP.domino
import IMP.core
import IMP.atom
import IMP.helper
import time

class DOMINOTests(IMP.test.TestCase):

    #set representation and scoring
    def __set_representation__(self):
        #load three proteins
        self.particles = IMP.Particles() #the particles to be optimized
        self.h_particles = []  #their molecular hierarchy decorator
        self.states=[]
        for s in ['1','2','3']:
            mp = IMP.atom.read_pdb(
                self.get_input_file_name('prot'+s+'.pdb'), self.imp_model)
            self.h_particles.append(mp)
            rb_p = IMP.Particle(self.imp_model)
            IMP.atom.setup_as_rigid_body(mp)
            mp.get_particle().add_attribute(IMP.domino.node_name_key(),"prot"+s)
            self.particles.append(mp.get_particle())

    def __set_restraints__(self):
        rsrs=[]
        self.opt_score = 0.0
        for i in range(2):
            ub = IMP.core.HarmonicUpperBound(1.0, 0.1)
            ss= IMP.core.DistancePairScore(ub)
            r= IMP.core.ConnectivityRestraint(ss)
            r.set_log_level(IMP.SILENT)
            ps = IMP.Particles()
            ps_refined=[]
            for j in range(2):
                ps_refined.append(IMP.core.get_leaves(self.h_particles[i+j]))
                ps.append(self.particles[i+j])
            print "ps_refined lenght is : " + str(len(ps_refined))
            for e in ps_refined:
                r.add_particles(e)
            self.imp_model.add_restraint(r)
            self.d_opt.add_restraint(r,ps)
#             beg  = time.time()
            self.opt_score = self.opt_score + r.evaluate(False)
#             end = time.time()
#             dt    = end - beg
#             print 'connectivity restraint calculation took %9.6f Seconds' % (dt)

    def __set_optimizer__(self):
        jt_filename = self.get_input_file_name("hierarchy_jt.txt")
        self.jt = IMP.domino.JunctionTree()
        IMP.domino.read_junction_tree(jt_filename,self.jt)
        self.re=IMP.domino.RestraintEvaluator(self.sampler)
        self.d_opt = IMP.domino.DominoOptimizer(self.jt,self.imp_model,self.re)

    def __set_discrete_sampling_space__(self):
        self.ps_cont = IMP.container.ListSingletonContainer(self.particles,"space")
        self.m_discrete_set = IMP.domino.TransformationMappedDiscreteSet(self.ps_cont)
        #set 4 optinal centroids for each of the particles
        for j,p in enumerate(self.particles):
            for i in range(3):
                new_p=IMP.Particle(self.imp_model)
                if (i==j):
                    IMP.domino.Transformation.setup_particle(new_p,
                      IMP.algebra.get_identity_transformation_3d())
                else:
                    IMP.domino.Transformation.setup_particle(new_p,
                        IMP.algebra.Transformation3D(
                        IMP.algebra.get_rotation_from_fixed_xyz( 0.3*i+0.5*j,i+j*j,i*i+1.2*j),
                        IMP.algebra.Vector3D(30*i+5*j,
                                             8*i*i+12*j,
                                             12*i+j*j)))

                self.m_discrete_set.add_state(new_p)
                self.m_discrete_set.add_mapped_state(p,new_p)
        self.sampler = IMP.domino.TransformationCartesianProductSampler(self.m_discrete_set,self.particles,True)


    def setUp(self):

        """Set up model and particles"""
        IMP.test.TestCase.setUp(self)
        self.imp_model = IMP.Model()
        IMP.set_check_level(IMP.USAGE_AND_INTERNAL)
        self.__set_representation__()
        self.__set_discrete_sampling_space__()
        self.__set_optimizer__()
        self.__set_restraints__()


    def test_global_min(self):
        """
        Test that the global minimum is achieved
        """
        self.d_opt.set_sampling_space(self.sampler)
        num_sol=5
        self.d_opt.set_number_of_solutions(num_sol)
        print self.d_opt.optimize(1)
        rg = self.d_opt.get_graph()
        print "OPT SCORE ::::::::::::; " + str(self.opt_score)
        scores=[self.opt_score,39.3363,65.1026,75.7, 91.8]
        for i in range(num_sol):
            score_inf = rg.get_opt_combination(i).get_total_score()
            self.assert_( abs(score_inf -scores[i]) < 0.2 ,
                          "the score of the minimum configuration as calculated by the inference is wrong " + str(score_inf) + " != " + str(scores[i]))

if __name__ == '__main__':
    IMP.test.main()
