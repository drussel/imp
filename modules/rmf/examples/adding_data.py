import IMP.atom
import IMP.rmf
m= IMP.Model()

# get the key with a given name from an rmf file, adding it if needed
def get_key(root, name, per_frame, category):
    if root.get_has_float_key(category, name):
        return root.get_float_key(category, name)
    else:
        return root.add_float_key(category, name, per_frame)

# Create a new IMP.atom.Hierarchy from the contents of the pdb file
h= IMP.atom.read_pdb(IMP.rmf.get_example_path("simple.pdb"), m)

# find the name for a temporary file to use to for writing the hdf5 file
tfn=IMP.create_temporary_file_name("pdb", ".rmf")

print "File name is", tfn

# open the temporary file, clearing any existing contents
rh = IMP.rmf.create_rmf_file(tfn)

my_kc= rh.add_category("my data");

# add the hierarchy to the file
IMP.rmf.add_hierarchy(rh, h)

# change a coordinate
IMP.core.XYZ(IMP.atom.get_leaves(h)[0]).set_x(0)

# add the new configuration to the file as frame 1
IMP.rmf.save_frame(rh, 1, h)

# create my key
my_key= get_key(rh, "my score", True, my_kc)

# make up scores
rh.set_value(my_key, 3, 0)
rh.set_value(my_key, 5, 1)
