import IMP
import IMP.test
import IMP.domino
import IMP.core
import IMP.atom
import random

class TrivialParticleStates(IMP.domino.ParticleStates):
    def __init__(self, n):
        IMP.domino.ParticleStates.__init__(self)
        self.n=n
    def get_number_of_particle_states(self):
        return self.n
    def load_state(self, i, p):
        pass
    def do_show(self, stream):
        pass


class DOMINOTests(IMP.test.TestCase):
    def _setup_round_trip(self):
        m= IMP.Model()
        ps=[]
        for i in range(0,8):
            ps.append(IMP.Particle(m))
        assignments=[]
        for i in range(0,20):
            ss=[random.randint(0,30) for i in range(0,8)]
            assignments.append(IMP.domino.Assignment(ss))
        return ps, IMP.domino.Subset(ps), assignments, m
    def _test_out(self, container, assignments):
        container.set_was_used(True)
        for a in assignments:
            container.add_assignment(a)
        self.assertEqual(container.get_number_of_assignments(),len(assignments))
    def _test_in(self, container, assignments):
        container.set_was_used(True)
        for i, a in enumerate(assignments):
            ac= container.get_assignment(i)
            print ac
            self.assertEqual(ac, a)
        self.assertEqual(len(assignments), container.get_number_of_assignments())
    def test_hdf5(self):
        """Testing default subset states writing to an hdf5 data set"""
        (ps, ss, ass, m)= self._setup_round_trip()
        try:
            import RMF
        except:
            self.skipTest("no RMF found")
        name= self.get_tmp_file_name("round_trip.hdf5")
        h5= RMF.create_hdf5_file(name)
        pss= IMP.domino.WriteHDF5AssignmentContainer(h5, ss, ps, "assignments")
        pss.set_cache_size(16)
        self._test_out(pss, ass)
        del pss
        ds= h5.get_child_index_data_set_2d("assignments")
        iss= IMP.domino.ReadHDF5AssignmentContainer(ds, ss, ps,
                                                     "in assignments")
        iss.set_cache_size(16)
        self._test_in(iss, ass)

    def test_binary(self):
        """Testing default subset states writing to an binary data set"""
        (ps, ss, ass, m)= self._setup_round_trip()
        name= self.get_tmp_file_name("round_trip.assignments")
        pss= IMP.domino.WriteAssignmentContainer(name, ss, ps, "assignments")
        pss.set_cache_size(16)
        self._test_out(pss, ass)
        del pss
        iss= IMP.domino.ReadAssignmentContainer(name, ss, ps, "in assignments")
        iss.set_cache_size(516)
        self._test_in(iss, ass)


    def test_sample(self):
        """Testing default sample container"""
        sac= IMP.domino.SampleAssignmentContainer(10)
        for i in range(0,25):
            ass= IMP.domino.Assignment([i])
            sac.add_assignment(ass)
        self.assertEqual(sac.get_number_of_assignments(), 10)
        print sac.get_assignments()

    def test_cluster(self):
        """Testing the cluster container"""
        m= IMP.Model()
        IMP.set_log_level(IMP.VERBOSE)
        ps= [IMP.Particle(m) for i in range(0,3)]
        s= IMP.domino.Subset(ps)
        pst= IMP.domino.ParticleStatesTable()
        ik= IMP.IntKey("hi")
        na=40
        iss= IMP.domino.IndexStates(na, ik)
        for p in ps:
            p.add_attribute(ik, 1)
            pst.set_particle_states(p, iss)
        nc=50
        cac= IMP.domino.ClusteredAssignmentContainer(nc, s, pst)
        cac.set_log_level(IMP.VERBOSE)
        for i in range(0,na):
            print i
            for j in range(0,na):
                for k in range(0,na):
                    ass=IMP.domino.Assignment([i,j,k])
                    cac.add_assignment(ass)
        print cac.get_assignments()
        self.assertLess(len(cac.get_assignments()), nc)
        print cac.get_r()
        self.assertLess(cac.get_r(), 20)

    def test_heap_container(self):
        """Testing heap sample container"""

        #create particles
        m= IMP.Model()
        m.set_log_level(IMP.SILENT)
        ps=[]
        for i in range(0,3):
            p= IMP.Particle(m)
            d= IMP.core.XYZR.setup_particle(p,IMP.algebra.Sphere3D(IMP.algebra.Vector3D(0,0,0),5))
            ps.append(p)

        pts=[IMP.algebra.Vector3D(0,0,0),
             IMP.algebra.Vector3D(1,0,0),
             IMP.algebra.Vector3D(2,0,0),
             IMP.algebra.Vector3D(3,0,0)]
        particle_state= IMP.domino.XYZStates(pts)
        pst= IMP.domino.ParticleStatesTable()
        for p in ps:
            pst.set_particle_states(p, particle_state)
        m.add_restraint(IMP.core.DistanceRestraint(IMP.core.Harmonic(1,1), ps[0], ps[1]))
        m.add_restraint(IMP.core.DistanceRestraint(IMP.core.Harmonic(1,1), ps[1], ps[2]))
        print 5
        sampler= IMP.domino.DominoSampler(m, pst)
        rssft= IMP.domino.RestraintScoreSubsetFilterTable(m, pst)
        s=IMP.domino.Subset(pst.get_particles())
        rssf=rssft.get_subset_filter(s,[])
        assignments=sampler.get_sample_assignments(s);
        print "number of assignments:",len(assignments)
        scores=[]
        for i in range(len(assignments)):
            print assignments[i],rssf.get_score(assignments[i])
            scores.append(rssf.get_score(assignments[i]))
        scores.sort()
        hac= IMP.domino.HeapAssignmentContainer(10,rssf)
        for a in assignments:
            hac.add_assignment(a)
        self.assertEqual(hac.get_number_of_assignments(), 10)
        #check that you have the top 10
        print "top ten"
        for a in hac.get_assignments():
            self.assertLess(rssf.get_score(a),scores[9]+0.01)
            print a,rssf.get_score(a)



if __name__ == '__main__':
    IMP.test.main()
