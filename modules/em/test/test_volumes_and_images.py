import unittest
import IMP.test
import IMP.em
import os

class VolumeTest(IMP.test.TestCase):

    def test_image(self):
        """Check image reading and writing"""
        img = IMP.em.Image()
        rw = IMP.em.SpiderImageReaderWriter(
                       self.get_input_file_name("flipY-nup84-0.spi"),
                       False,False,True)
        img.read(self.get_input_file_name("flipY-nup84-0.spi"), rw)
        img.write("test_image.spi",rw)
        img2 = IMP.em.Image()
        img2.read("test_image.spi",rw)
        for j in xrange(0,img.get_data().get_number_of_rows()):
            for i in xrange(0,img.get_data().get_number_of_columns()):
                self.assertEqual(img.get_data()[i,j],img2.get_data()[i,j])
        # Cleanup
        os.unlink('test_image.spi')

    def test_em_maps(self):
        """Check volume reading and writing"""
        # Read in Xmipp format
        rw1 = IMP.em.SpiderMapReaderWriter(
                        self.get_input_file_name("media_mon_iter3.xmp"),
                        False,False,True)
        rw2 = IMP.em.MRCReaderWriter()
        m = IMP.em.DensityMap()
        m.Read(self.get_input_file_name("media_mon_iter3.xmp"), rw1)
        # Compare against known voxel values to make sure we're reading the
        # file correctly
        self.assertInTolerance(m.get_value(m.xyz_ind2voxel(24,28,25)),
                               0.04647, 0.001)
        self.assertInTolerance(m.get_value(m.xyz_ind2voxel(23,29,25)),
                               0.03346, 0.001)
        m.Write("test.mrc",rw2)
        m.Write("test.xmp",rw1)
        m2 = IMP.em.DensityMap()
        m2.Read("test.xmp",rw1)
        # Check that the two maps have the same values
        for k in xrange(0,m.get_header().nz):
            for j in xrange(0,m.get_header().ny):
                for i in xrange(0,m.get_header().nx):
                    self.assertEqual(m.get_value(m.xyz_ind2voxel(i,j,k)),
                                    m2.get_value(m.xyz_ind2voxel(i,j,k)))
        # Cleanup
        os.unlink('test.mrc')
        os.unlink('test.xmp')

if __name__ == '__main__':
    unittest.main()
