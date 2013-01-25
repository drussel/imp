## \example rmf/pdb.py
## Write a PDB to an hdf5 file.
##

import IMP.atom
import IMP.rmf
import RMF
m= IMP.Model()

# Create a new IMP.atom.Hierarchy from the contents of the pdb file
h= IMP.atom.read_pdb(IMP.rmf.get_example_path("simple.pdb"), m)

# find the name for a temporary file to use to for writing the hdf5 file
tfn=IMP.create_temporary_file_name("pdb", ".rmf")

print "File name is", tfn

# open the temporary file, clearing any existing contents
rh = RMF.create_rmf_file(tfn)

# add the hierarchy to the file
IMP.rmf.add_hierarchies(rh, [h])

# add the current configuration to the file as frame 0
IMP.rmf.save_frame(rh, 0)

# change a coordinate
IMP.core.XYZ(IMP.atom.get_leaves(h)[0]).set_x(0)

# add the new configuration to the file as frame 1
IMP.rmf.save_frame(rh, 1)

# close the file
del rh

# reopen it, don't clear the file when opening it
rh= RMF.open_rmf_file(tfn)

# hps is a list with one element which is a copy of h
hps= IMP.rmf.create_hierarchies(rh, m)

IMP.atom.show_molecular_hierarchy(hps[0])

# load the second configuration into hps
IMP.rmf.load_frame(rh, 0)

print "Try running hdf5_display or hdf5_show on", tfn
