import unittest
import os
from modeller import *
import IMP
import IMP.test
import IMP.core
import IMP.modeller

class ModellerRestraintsTests(IMP.test.TestCase):

    def assertSimilarModellerIMPScores(self, modeller_model, imp_model):
        """Assert that Modeller and IMP give the same score"""
        modeller_energy = selection(modeller_model).energy()[0]
        imp_score = imp_model.evaluate(True)
        self.assertInTolerance(modeller_energy, imp_score, 0.1)

    def test_read_static_restraints(self):
        """Check loading of Modeller static restraints"""
        e = environ()
        e.edat.dynamic_sphere = False
        e.libs.topology.read('${LIB}/top_heav.lib')
        e.libs.parameters.read('${LIB}/par.lib')
        modmodel = model(e)
        modmodel.build_sequence('GGCC')

        m = IMP.Model()
        loader = IMP.modeller.ModelLoader(modmodel)
        protein = loader.load_atoms(m)

        at = modmodel.atoms
        restraints = []
        # Typical distance restraints or stereochemical bonds:
        r = forms.gaussian(feature=features.distance(at[0], at[1]),
                           mean=1.54, stdev=0.1, group=physical.xy_distance)
        restraints.append(r)
        r = forms.lower_bound(feature=features.distance(at[0], at[1]),
                              mean=10.0, stdev=0.1, group=physical.xy_distance)
        restraints.append(r)
        r = forms.upper_bound(feature=features.distance(at[0], at[1]),
                              mean=10.0, stdev=0.1, group=physical.xy_distance)
        restraints.append(r)

        # Typical stereochemical angle restraint:
        r = forms.gaussian(feature=features.angle(at[0], at[1], at[2]),
                           mean=1.92, stdev=0.07, group=physical.xy_distance)
        restraints.append(r)

        # Typical stereochemical improper dihedral restraint:
        r = forms.gaussian(feature=features.dihedral(at[0], at[1], at[2],
                                                     at[3]),
                           mean=3.14, stdev=0.1, group=physical.xy_distance)
        restraints.append(r)

        # Typical stereochemical dihedral restraint:
        r = forms.cosine(feature=features.dihedral(at[0], at[1], at[2],
                                                   at[3]),
                         group=physical.xy_distance,
                         phase=0.0, force=2.5, period=2)
        restraints.append(r)

        # Typical splined restraint:
        r = forms.spline(feature=features.distance(at[0], at[1]), open=True,
                         low=1.0, high=5.0, delta=1.0,
                         group=physical.xy_distance,
                         lowderiv=0.0, highderiv=0.0,
                         values=[100.0, 200.0, 300.0, 200.0, 100.0])
        restraints.append(r)

        # Test forms.factor
        r = forms.factor(feature=features.angle(at[0], at[1], at[2]),
                          factor=100.0, group=physical.xy_distance)
        restraints.append(r)

        # Test periodic splined restraint:
        r = forms.spline(feature=features.dihedral(at[0], at[1], at[2], at[3]),
                         open=False, low=0.0, high=6.2832, delta=1.2566,
                         group=physical.xy_distance,
                         lowderiv=0.0, highderiv=0.0,
                         values=[100.0, 200.0, 300.0, 400.0, 300.0])
        restraints.append(r)

        for r in restraints:
            modmodel.restraints.clear()
            modmodel.restraints.add(r)

            rset = IMP.RestraintSet()
            m.add_restraint(rset)
            for rsr in loader.load_static_restraints():
                rset.add_restraint(rsr)
            self.assertSimilarModellerIMPScores(modmodel, m)
            rset.set_weight(0)

    def test_rsr_file_read(self):
        """Check reading of arbitrary Modeller restraint files"""
        e = environ()
        e.edat.dynamic_sphere = False
        e.libs.topology.read('${LIB}/top_heav.lib')
        e.libs.parameters.read('${LIB}/par.lib')
        modmodel = model(e)
        modmodel.build_sequence('GGCC')
        open('test.rsr', 'w').write('MODELLER5 VERSION: MODELLER FORMAT\n'
                                    'R    3   1   1   1   2   2   0     3'
                                    '     2       1.5380    0.0364')
        modmodel.restraints.append('test.rsr')
        # Deprecated interface
        m = IMP.Model()
        loader = IMP.modeller.ModelLoader(modmodel)
        protein = loader.load_atoms(m)
        r = IMP.modeller.load_restraints_file('test.rsr', protein)
        self.assert_(isinstance(r, list))
        for rsr in r:
            m.add_restraint(rsr)
        self.assertSimilarModellerIMPScores(modmodel, m)

        # Need atoms before loading restraints
        m = IMP.Model()
        loader = IMP.modeller.ModelLoader(modmodel)
        self.assertRaises(ValueError, loader.load_static_restraints_file,
                          'test.rsr')
        # New interface
        protein = loader.load_atoms(m)
        for rsr in loader.load_static_restraints_file('test.rsr'):
            m.add_restraint(rsr)
        self.assertSimilarModellerIMPScores(modmodel, m)
        os.unlink('test.rsr')


if __name__ == '__main__':
    unittest.main()
