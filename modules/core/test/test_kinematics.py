import IMP
import IMP.test
import IMP.core
import IMP.algebra

class Test(IMP.test.TestCase):
    """Tests for kinematic structure"""

    def test_basic_kinematics(self):
        """Checking basic construction of kinematic structure"""
        IMP.set_log_level(IMP.VERBOSE)
        m= IMP.Model()
        p0= IMP.Particle(m)
        p1= IMP.Particle(m)
        p2= IMP.Particle(m)
        tr0 = IMP.algebra.Transformation3D( [0,0,0] )
        tr1 = IMP.algebra.Transformation3D( [0,0,1] )
        tr2 = IMP.algebra.Transformation3D( [0,0,2] )
        ref0 = IMP.algebra.ReferenceFrame3D( tr0 )
        ref1 = IMP.algebra.ReferenceFrame3D( tr1 )
        ref2 = IMP.algebra.ReferenceFrame3D( tr2 )
        rb0 = IMP.core.RigidBody.setup_particle(p0, ref0)
        rb1 = IMP.core.RigidBody.setup_particle(p1, ref1)
        rb2 = IMP.core.RigidBody.setup_particle(p2, ref2)
        print rb0.get_coordinates()
        print rb1.get_coordinates()
        print rb2.get_coordinates()
        print IMP.algebra.get_distance(rb0.get_coordinates(),rb1.get_coordinates())
        kf = IMP.core.KinematicForest(m)
        pj01 = IMP.core.PrismaticJoint(rb0, rb1)
        print pj01
        pj12 = IMP.core.PrismaticJoint(rb1, rb2)
        print pj12
        kf.add_edge(pj01)
        kf.add_edge(pj12)
        print kf
        print "lengths before set coords safe"
        print pj01.get_length()
        print pj12.get_length()
        kf.set_coordinates_safe(rb1,[0,0,0.5])
        print "lengths after set coords safe"
        print pj01.get_length()
        print pj12.get_length()
        pj01.set_length(10.0)
        print "lengths after set_length(10.0) of pj01"
        print pj01.get_length()
        print pj12.get_length()


if __name__ == '__main__':
    IMP.test.main()
