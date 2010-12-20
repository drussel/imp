import IMP
import IMP.test
import IMP.core
import IMP.atom

class DecoratorTests(IMP.test.TestCase):
    def test_hierarchy(self):
        """Check that the hierarchy works fine"""
        m = IMP.Model()
        pp=IMP.Particle(m)
        ppd= IMP.core.Hierarchy.setup_particle(pp)
        pp.set_name("root")
        for i in range(0,10):
            pc=IMP.Particle(m)
            pcd= IMP.core.Hierarchy.setup_particle(pc)
            ppd.add_child(pcd)
            pc.set_name(str("child "+str(i)))

        self.assertEqual(ppd.get_number_of_children(), 10,
                         "Number of children is not as expected")
    def test_alt_hierarchy(self):
        """Check that an alternate hierarchy works fine"""
        t= IMP.core.HierarchyTraits("my_hierarchy")
        m = IMP.Model()
        pp=IMP.Particle(m)
        ppd= IMP.core.Hierarchy.setup_particle(pp, t)
        pp.set_name("root")
        for i in range(0,10):
            pc=IMP.Particle(m)
            pcd= IMP.core.Hierarchy.setup_particle(pc, t)
            ppd.add_child(pcd)
            pc.set_name(str("child "+str(i)))

        self.assertEqual(ppd.get_number_of_children(), 10,
                         "Number of children is not as expected")
    def test_conversion(self):
        """Test conversion from atom.hierarchy"""
        m= IMP.Model()
        p =IMP.Particle(m)
        h= IMP.atom.Hierarchy.setup_particle(p)
        c= IMP.core.get_leaves(h)

if __name__ == '__main__':
    IMP.test.main()
