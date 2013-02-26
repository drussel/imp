## \example modules/rotamer/examples/rotamer_pdb.py
## rotamer_pdb.py is a script demonstrating the usage of RotamerCalculator and RotamerLibrary.
## It reads a PDB file and a rotamer library file, and tries to rotate the atoms based on the most
## probable chi angles from the rotamer library. Then it saves the rotated atoms to a specified output
## PDB file.
##
## Usage:
##
## `python rotamer_pdb.py -i <input>.pdb -l <rotamer_library>.lib -o <output>.pdb`
##
## Example (the result will be saved into transformed_1z5s_A.pdb):
##
## `../../../tools/imppy.sh python rotamer_pdb.py -i ../../atom/test/input/1z5s_A.pdb \
##   -l /path/to/ALL.bbdep.rotamers.lib -o transformed_1z5s_A.pdb`
##

#!/usr/bin/python

import IMP
import IMP.core
import IMP.atom
import IMP.algebra
import IMP.rotamer


def transform(input_pdb, input_lib, output_pdb):
    # read rotamer library
    rl = IMP.rotamer.RotamerLibrary()
    rl.read_library_file(input_lib)
    rc = IMP.rotamer.RotamerCalculator(rl)

    # read the original PDB
    m = IMP.Model()
    orig_h = IMP.atom.read_pdb(input_pdb, m)
    mh = IMP.atom.get_by_type(orig_h, IMP.atom.RESIDUE_TYPE)

    # transform...
    hps = IMP.core.HarmonicDistancePairScore(1, 100)
    rc.transform(orig_h, hps, 0.9, 1e-6, 6)

    # save the rotated atoms to output PDB
    IMP.atom.write_pdb(orig_h, output_pdb)


if __name__ == '__main__':

    import sys
    import optparse

    P = optparse.OptionParser()
    P.add_option('--input_pdb', '-i', action='store', type='string',
        help='input PDB file (required)')
    P.add_option('--input_lib', '-l', action='store', type='string',
        help='input rotamer library file (required)')
    P.add_option('--output_pdb', '-o', action='store', type='string',
        help='output PDB file (required)')
    P.add_option('--verbose', '-v', action='store_true',
        help='show more messages')
    opts, args = P.parse_args()
    if not opts.input_pdb:
        print '--input_pdb is required'
        sys.exit(1)
    if not opts.output_pdb:
        print '--output_pdb is required'
        sys.exit(1)
    if not opts.input_lib:
        print '--input_lib is required'
        sys.exit(1)
    if opts.verbose:
        IMP.base.set_log_level(IMP.base.VERBOSE)
    else:
        IMP.base.set_log_level(IMP.base.SILENT)
    transform(opts.input_pdb, opts.input_lib, opts.output_pdb)
