import unittest
import RMF
import shutil

class GenericTest(RMF.TestCase):
    def _show(self, g):
        for i in range(0, g.get_number_of_children()):
            print i, g.get_child_name(i), g.get_child_is_group(i)
    """Test the python code"""
    def test_perturbed(self):
        """Test copying an rmf file"""
        nm= self.get_input_file_name("sink.rmf")
        onm= self.get_tmp_file_name("sink_out.rmf")
        f= RMF.open_rmf_file_read_only(nm)
        of= RMF.create_rmf_file(onm)
        RMF.copy_structure(f, of)
        nf= f.get_number_of_frames()
        for i in range(0, nf):
            RMF.copy_frame(f, of, i, i)

        self.assertTrue(RMF.get_equal_structure(f, of, True))
        for i in range(0, nf):
            self.assertTrue(RMF.get_equal_frame(f, of, i, i, True))

if __name__ == '__main__':
    unittest.main()
