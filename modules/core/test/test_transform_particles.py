import unittest
import IMP
import IMP.core
import IMP.algebra
import IMP.test

class ParticleTransformationTests(IMP.test.TestCase):
    """Test particle transformations"""
    def test_transformation(self):
        """Test the TransformationFunction class"""
        imp_model = IMP.Model()
        particles = IMP.core.create_xyzr_particles(imp_model, 4, 1.0);

        coords= [x.get_coordinates() for x in particles]
        r = IMP.algebra.get_rotation_from_fixed_xyz(0.2,0.8,-0.4)
        t=IMP.algebra.Transformation3D(r,IMP.algebra.Vector3D(20.0,-12.4,18.6))
        print "create transform"
        tf=IMP.core.Transform(t)
        tf.set_was_used(True)
        for p in particles:
            print "applying to "+str(p)
            r = tf.apply(p.get_particle())
        for i in range(0,len(particles)):
            v = particles[i].get_coordinates()
            self.assertInTolerance((v-t.get_transformed(coords[i])).get_magnitude(), 0, 0.01)

    def test_transformation2(self):
        """Test the TransformationFunction class with map"""
        imp_model = IMP.Model()
        particles = IMP.core.create_xyzr_particles(imp_model, 4, 1.0);

        coords= [x.get_coordinates() for x in particles]
        r = IMP.algebra.get_rotation_from_fixed_xyz(0.2,0.8,-0.4)
        t=IMP.algebra.Transformation3D(r,IMP.algebra.Vector3D(20.0,-12.4,18.6))
        tf=IMP.core.Transform(t)
        tf.set_was_used(True)
        map( IMP.SingletonFunctor(tf), [x.get_particle() for x in particles])
        for i in range(0,len(particles)):
            v = particles[i].get_coordinates()
            self.assertInTolerance((v-t.get_transformed(coords[i])).get_magnitude(), 0, 0.01)

if __name__ == '__main__':
    unittest.main()
