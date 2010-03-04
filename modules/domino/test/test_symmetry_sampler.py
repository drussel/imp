import sys
import unittest
import IMP
import IMP.test
import IMP.domino
import IMP.core
import IMP.atom
import IMP.algebra
import math

def get_diameter(xyzs):
    max_squared_dist=0.0;
    for i in range(0, len(xyzs)):
        xyzi= xyzs[i].get_coordinates()
        for j in range(0,i):
            xyzj= xyzs[j].get_coordinates()
            curr_squared_dist =IMP.algebra.get_squared_distance(xyz1,xyz2);
            if curr_squared_dist > max_squared_dist:
                p1 = xyzs[i]
                p2 = xyzs[j]
                max_squared_dist = curr_squared_dist
    return IMP.algebra.Segment3D(p1.get_coordinates(),
                                 p2.get_coordinates())

class DOMINOTests(IMP.test.TestCase):
    def __set_particles(self):
        #after the sampler you need to check that the centroids are rotated the same
        self.m = IMP.Model()
        self.ps = IMP.Particles()
        self.prots=[]
        #create three Hierarchy particels:
        self.prots.append(IMP.atom.read_pdb(self.get_input_file_name("symm1.pdb"),
                                    self.m))
        self.prots[0].get_particle().add_attribute(IMP.domino.node_name_key(),str(0))
        self.ps.append(self.prots[0].get_particle())
        self.ref = IMP.atom.read_pdb(self.get_input_file_name("symm1.pdb"),
                                     self.m)

        self.prots.append(IMP.atom.read_pdb(self.get_input_file_name("symm2.pdb"),
                                         self.m))
        self.prots[1].get_particle().add_attribute(IMP.domino.node_name_key(),str(1))
        self.ps.append(self.prots[1].get_particle())
        self.prots.append(IMP.atom.read_pdb(self.get_input_file_name("symm3.pdb"),
                                    self.m))
        self.prots[2].get_particle().add_attribute(IMP.domino.node_name_key(),str(2))
        self.ps.append(self.prots[2].get_particle())


    def __set_sampling_space__(self):
        #set 10 transformations on a patch of a sphere
        self.rt = IMP.domino.TransformationDiscreteSet()
        self.rt.set_model(self.m)
        max_d1 = get_diameter(IMP.core.XYZsTemp(IMP.core.get_leaves(self.ref)))
        max_d=IMP.algebra.Segment3D(max_d1.get_point(1),max_d1.get_point(0))
        #print ".dot " + str(max_d.get_point(0)[0]) + " " + str(max_d.get_point(0)[1]) + " " + str(max_d.get_point(0)[2])
        #print ".dot " +str(max_d.get_point(1)[0]) + " " + str(max_d.get_point(1)[1]) + " " + str(max_d.get_point(1)[2])
        cone = IMP.algebra.Cone3D(max_d,max_d.get_length())
        print ".cone " + str(max_d.get_point(1)[0]) + " " + str(max_d.get_point(1)[1]) + " " + str(max_d.get_point(1)[2]) +" " + str(max_d.get_point(0)[0]) + " " + str(max_d.get_point(0)[1]) + " " + str(max_d.get_point(0)[2]) + " " + str(max_d.get_length())
        pln = cone.get_base_plane()
        patch = IMP.algebra.SpherePatch3D(IMP.algebra.Sphere3D(cone.get_tip(),
                                                               (cone.get_radius()**2\
                                                                    +cone.get_height()**2)**.5),
                                          pln.get_opposite())
        print ".dot " +str(pln.get_point_on_plane()[0]) + " " + str(pln.get_point_on_plane()[1]) + " " + str(pln.get_point_on_plane()[2])
        #sss = cone.get_bounding_sphere()
        #print ".sphere " + str(sss.get_center()[0]) + " " + str(sss.get_center()[1]) + " "+ str(sss.get_center()[2]) + " " + str(sss.get_radius())
        #print ".sphere " + str(patch.get_sphere().get_center()[0]) + " " + str(patch.get_sphere().get_center()[1]) + " "+ str(patch.get_sphere().get_center()[2]) + " " + str(patch.get_sphere().get_radius())
        zero_vec = IMP.algebra.Vector3D(0.0,0.0,0.0)
        rots = IMP.algebra.get_uniform_surface_cover(patch,10)
        for rot in rots:
            print ".dot " + str(rot[0]) + " " + str(rot[1]) + " " + str(rot[2])
            r = IMP.algebra.get_rotation_taking_first_to_second(max_d.get_point(1),rot)
            t = IMP.algebra.Transformation3D(r,zero_vec)
            self.rt.add_transformation(t)
        #write a function to find a bounding cylinder
#         self.cyl = IMP.algebra.Cylinder3D(IMP.algebra.Vector3D(0.0,0.0,-50.0),
#                                           IMP.algebra.Vector3D(0.0,0.0,-90.0),5.0)
        self.cyl = IMP.algebra.Cylinder3D(IMP.algebra.Segment3D(IMP.algebra.Vector3D(0.0,0.0,0.0),
                                          IMP.algebra.Vector3D(0.0,0.0,1.0)),5.0)

        self.sampler = IMP.domino.SymmetrySampler(self.ps,self.rt,self.cyl)
    def set_restraint_graph(self):
        jt_filename = self.get_input_file_name("simple_jt3.txt")
        self.jt = IMP.domino.JunctionTree()
        IMP.domino.read_junction_tree(jt_filename,self.jt)
        self.re=IMP.domino.RestraintEvaluator(self.sampler)
        self.d_opt = IMP.domino.DominoOptimizer(self.jt,self.m,self.re)
        self.d_opt.set_sampling_space(self.sampler)

    def setUp(self):
        """Set up model and particles"""
        IMP.set_log_level(IMP.VERBOSE)
        IMP.test.TestCase.setUp(self)
        self.__set_particles()
        self.__set_sampling_space__()
        self.set_restraint_graph()

    def test_global_min(self):
        """
        Test that the sampler created symmetric sampling space
        """
        r = self.d_opt.get_graph()
        n = r.get_node(self.ps)
        cendtroids=[IMP.algebra.Vector3D(0.0,0.0,0.0),
                    IMP.algebra.Vector3D(0.0,0.0,0.0),
                    IMP.algebra.Vector3D(0.0,0.0,0.0),]
        for i in range(5):
            state = n.get_state(i)
            self.sampler.move2state(state)
            for j,p in enumerate(self.ps):
                cendtroids[j] = cendtroids[j]+IMP.core.get_centroid(IMP.core.XYZsTemp(IMP.core.get_leaves(self.prots[j])))

        rot120 = IMP.algebra.Transformation3D(
            IMP.algebra.get_rotation_in_radians_about_axis(self.cyl.get_segment().get_direction(),
                                                        2.*math.pi/3))
        for j in range(2):
            self.assertAlmostEqual(IMP.algebra.get_distance(rot120.get_transformed(cendtroids[j]),cendtroids[j+1]),0.0,2)
if __name__ == '__main__':
    unittest.main()
