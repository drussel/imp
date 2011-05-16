import IMP
import IMP.test
import IMP.core
import IMP.algebra
import IMP.atom


class RBDTests(IMP.test.TestCase):
    """Tests for RigidBody function"""
    def _add_rb_restraints(self, rbd):
        #intentionally kept trivial to ensure convergence
        r= IMP.algebra.get_identity_rotation_3d()
        t= IMP.algebra.get_random_vector_in(IMP.algebra.get_unit_sphere_3d())
        tr= IMP.algebra.Transformation3D(r,t)
        mbs= rbd.get_members()
        m= rbd.get_particle().get_model()
        for b in mbs:
            mb= IMP.core.RigidMember(b.get_particle())
            lc= mb.get_internal_coordinates()
            lct= tr.get_transformed(lc)
            dt= IMP.core.DistanceToSingletonScore(IMP.core.Harmonic(0,1), lct)
            r= IMP.core.SingletonRestraint(dt, mb.get_particle())
            m.add_restraint(r)
    def _create_hierarchy(self, m, n=10):
        rd= IMP.core.XYZ.setup_particle(IMP.Particle(m),
                                         IMP.algebra.get_random_vector_in(IMP.algebra.get_unit_bounding_box_3d()))
        hd= IMP.core.Hierarchy.setup_particle(rd.get_particle())
        for i in range(0,n):
            crd= IMP.core.XYZ.setup_particle(IMP.Particle(m),
                                              IMP.algebra.get_random_vector_in(IMP.algebra.get_unit_bounding_box_3d()))
            chd= IMP.core.Hierarchy.setup_particle(crd.get_particle())
            hd.add_child(chd)
        return rd.get_particle()

    def test_create_one(self):
        """Testing create_rigid_body"""
        count=1
        success=0
        for i in range(0, count):
            m= IMP.Model()
            IMP.set_log_level(IMP.SILENT)
            print "creating"
            p= self._create_hierarchy(m)
            print "created", p
            print "wrapping"
            h=IMP.core.Hierarchy(p)
            print "getting children"
            children=h.get_children()
            print "wrapping children"
            cs=IMP.core.XYZs(children)
            print "setting up"
            rbd=IMP.core.RigidBody.setup_particle(p, cs)
            print "set up"
            p.show()
            rbd.set_coordinates_are_optimized(True)
            self. _add_rb_restraints(rbd)
            cg= IMP.core.ConjugateGradients()
            cg.set_model(m)
            print "Initial score is " + str(m.evaluate(False))
            cg.optimize(1000)
            if m.evaluate(False) < .1:
                success=success+1
        self.assertGreater(success, count/2)


    def test_create_one_from_pdb(self):
        """Testing create_rigid_bodies"""
        m= IMP.Model()
        h= IMP.atom.read_pdb(self.get_input_file_name("input.pdb"), m)
        print "done reading"
        rb= IMP.atom.create_rigid_body(h)
        rb.set_coordinates_are_optimized(True)
        print "done setting up"
        ls= IMP.core.get_leaves(h)
        keypts= [ls[0], ls[-1], ls[len(ls)/3], ls[len(ls)/3*2]]
        tr= IMP.algebra.Transformation3D(IMP.algebra.get_random_rotation_3d(),
                                         IMP.algebra.get_random_vector_in(IMP.algebra.BoundingBox3D(IMP.algebra.Vector3D(0,0,0), IMP.algebra.Vector3D(500, 500, 500))))
        for p in keypts:
            mp= IMP.core.RigidMember(p.get_particle())
            ic= mp.get_internal_coordinates()
            nic= tr.get_transformed(ic)
            dt= IMP.core.DistanceToSingletonScore(IMP.core.Harmonic(0,1), nic)
            r= IMP.core.SingletonRestraint(dt, p.get_particle())
            m.add_restraint(r)
        cg= IMP.core.ConjugateGradients()
        cg.set_model(m)
        cg.optimize(100)
        ntr= rb.get_reference_frame().get_transformation_to()
        print ntr
        print tr
        self.assertLess((ntr.get_translation()- tr.get_translation()).get_magnitude(), 2)

    # test one with snap and non-snap
    # test setting things to be optimized or not

if __name__ == '__main__':
    IMP.test.main()
