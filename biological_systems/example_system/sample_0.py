import IMP
import IMP.atom
import IMP.container
import IMP.display
import IMP.statistics
import IMP.example
import IMP.system
from IMP.example_system_local import *
import parameters
import setup
import os.path

display_restraints=[]


import sys
print "hi"
print sys.argv

if not IMP.has_netcdfcpp:
    print >> sys.stderr, "This script requires IMP to be built with NetCDF"
    sys.exit(0)

# find acceptable conformations of the model
def get_conformations(m, gs, n):
    sampler= IMP.core.MCCGSampler(m)

    sampler.set_bounding_box(parameters.bb)
    # magic numbers, experiment with them and make them large enough for things to work
    sampler.set_number_of_conjugate_gradient_steps(100)
    sampler.set_number_of_monte_carlo_steps(10)
    sampler.set_number_of_attempts(n)
    sampler.set_save_rejected_configurations(True)
    # We don't care to see the output from the sampler
    sampler.set_log_level(IMP.SILENT)

    # return the IMP.ConfigurationSet storing all the found configurations that
    # meet the various restraint maximum scores.
    cs= sampler.get_sample()
    # Look at the rejected minimal conformations to understand how the restraints
    # are failing
    print "rejected", \
          sampler.get_rejected_configurations().get_number_of_configurations(), "solutions"
    m.set_gather_statistics(True)
    for i in range(0, sampler.get_rejected_configurations().get_number_of_configurations()):
        sampler.get_rejected_configurations().load_configuration(i)
        m.evaluate(False)
        # show the score for each restraint to see which restraints were causing the
        # conformation to be rejected
        m.show_restraint_score_statistics()
        w= IMP.display.PymolWriter(IMP.system.get_output_path("rejected.%d.pym"%i))
        for g in gs:
            w.add_geometry(g)
    return cs

(i, n) =IMP.system.get_sample_parameters()


# now do the actual work
(m,all)= setup.create_representation()
IMP.atom.show_molecular_hierarchy(all)
setup.create_restraints(m, all)


gs= setup.create_geometry(all)
IMP.set_log_level(IMP.SILENT)
cs= get_conformations(m, gs, 2000.0/n)
IMP.set_log_level(IMP.TERSE)

print "found", cs.get_number_of_configurations(), "solutions"
for i in range(0, cs.get_number_of_configurations()):
    cs.load_configuration(i)
    w= IMP.display.PymolWriter(IMP.system.get_output_path("struct.%d.pym"%i))
    for g in gs:
        w.add_geometry(g)
keys=IMP.core.XYZ.get_xyz_keys()
IMP.write_configuration_set(cs, IMP.atom.get_leaves(all), keys,
                            IMP.system.get_output_path("configurations_"+str(i)+".bimp"))
