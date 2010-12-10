import IMP
import IMP.test
import IMP.core
import IMP.atom
import IMP.algebra

class CHARMMParametersTests(IMP.test.TestCase):
    """Test the CHARMMParameters class"""

    def test_bond_parameters(self):
        """Check extraction of bond parameters"""
        p = IMP.atom.CHARMMParameters(IMP.atom.get_data_path('top.lib'),
                                      IMP.atom.get_data_path('par.lib'))
        self.assertRaises(IndexError, p.get_bond_parameters, 'garbage', 'CT2')
        for types in [('CT1', 'CT2'), ('CT2', 'CT1')]:
            bond = p.get_bond_parameters(*types)
            self.assertAlmostEqual(bond.force_constant, 222.500, delta=1e-4)
            self.assertAlmostEqual(bond.ideal, 1.5380, delta=1e-5)

        self.assertRaises(IndexError, p.get_angle_parameters,
                          'garbage', 'CT2', 'CT3')
        for types in [('OM', 'CM', 'FE'), ('FE', 'CM', 'OM')]:
            bond = p.get_angle_parameters(*types)
            self.assertAlmostEqual(bond.force_constant, 35.000, delta=1e-4)
            self.assertAlmostEqual(bond.ideal, 180.0000, delta=1e-5)
        self.assertRaises(IndexError, p.get_angle_parameters, 'OM', 'FE', 'CM')

        self.assertRaises(IndexError, p.get_dihedral_parameters,
                          'garbage', 'C', 'CT2', 'CT3')
        # Check multiple dihedrals
        bonds = p.get_dihedral_parameters('CP1', 'C', 'N', 'CP1')
        self.assertEqual(len(bonds), 2)
        self.assertAlmostEqual(bonds[0].force_constant, 2.7500, delta=1e-4)
        self.assertEqual(bonds[0].multiplicity, 2)
        self.assertAlmostEqual(bonds[0].ideal, 180.00, delta=1e-5)
        self.assertAlmostEqual(bonds[1].force_constant, 0.3000, delta=1e-4)
        self.assertEqual(bonds[1].multiplicity, 4)
        self.assertAlmostEqual(bonds[1].ideal, 0.00, delta=1e-5)

        # Check wildcards
        bonds = p.get_dihedral_parameters('OM', 'CT1', 'NH3', 'FE')
        self.assertEqual(len(bonds), 1)
        self.assertAlmostEqual(bonds[0].force_constant, 0.1000, delta=1e-4)
        self.assertEqual(bonds[0].multiplicity, 3)
        self.assertAlmostEqual(bonds[0].ideal, 0.00, delta=1e-5)

        self.assertRaises(IndexError, p.get_improper_parameters,
                          'garbage', 'C', 'CT2', 'CT3')
        for types in [('CPB', 'CPA', 'NPH', 'CPA'),
                      ('CPA', 'NPH', 'CPA', 'CPB')]:
            bond = p.get_improper_parameters(*types)
            self.assertAlmostEqual(bond.force_constant, 20.800, delta=1e-4)
            self.assertEqual(bond.multiplicity, 0)
            self.assertAlmostEqual(bond.ideal, 0.0000, delta=1e-5)
        # Check wildcards
        for types in [('CPB', 'CPA', 'NPH', 'C'),
                      ('CPB', 'CT2', 'CT3', 'C')]:
            bond = p.get_improper_parameters(*types)
            self.assertAlmostEqual(bond.force_constant, 90.000, delta=1e-4)
            self.assertEqual(bond.multiplicity, 0)
            self.assertAlmostEqual(bond.ideal, 0.0000, delta=1e-5)
        self.assertRaises(IndexError, p.get_improper_parameters,
                          'NPH', 'CPA', 'CPB', 'CPA')

if __name__ == '__main__':
    IMP.test.main()
