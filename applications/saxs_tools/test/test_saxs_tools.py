import IMP.test
import sys
import os
import re

class SAXSToolsTest(IMP.test.ApplicationTestCase):
    def test_rg(self):
        """Simple test of SAXS profile application"""
        print self.get_input_file_name('6lyz.pdb')
        print self.get_input_file_name('lyzexp.dat')
        p = self.run_application('rg',
                                 [self.get_input_file_name('6lyz.pdb'),
                                  self.get_input_file_name('lyzexp.dat')])
        out, err = p.communicate()
        sys.stderr.write(err)
        self.assertApplicationExitedCleanly(p.returncode, err)
        m = re.search('lyzexp.dat Rg= ([\d\.]+)', out)
        self.assertNotEqual(m, None, msg="Rg value output not found in " + str(out))
        self.assertAlmostEqual(float(m.group(1)), 15.14, delta=0.1)
        m = re.search('6lyz.pdb Rg= ([\d\.]+)', out)
        self.assertNotEqual(m, None, msg="Rg value output not found in " + str(out))
        self.assertAlmostEqual(float(m.group(1)), 13.966, delta=0.1)


if __name__ == '__main__':
    IMP.test.main()
