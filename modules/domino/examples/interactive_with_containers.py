import IMP.domino
import IMP.algebra
import IMP
import RMF

m = IMP.Model()

# create some particles
ps=[IMP.core.XYZ.setup_particle(IMP.Particle(m)) for x in range(0,3)]

s= IMP.core.HarmonicDistancePairScore(1, 1)
lpc= IMP.container.ListPairContainer([(ps[i[0]], ps[i[1]]) for i in [(0,1), (1,2)]])
print [(p[0].get_name(), p[1].get_name()) for p in lpc.get_particle_pairs()]
r= IMP.container.PairsRestraint(s, lpc)
m.add_restraint(r)
r.set_maximum_score(.1)

space= IMP.domino.XYZStates([IMP.algebra.Vector3D(i, 0, 0) for i in range(0,6)])

pst= IMP.domino.ParticleStatesTable()
for p in ps:
    pst.set_particle_states(p, space)

m.set_log_level(IMP.SILENT)

# make sure to break up the
mt= IMP.domino.get_merge_tree(m.get_root_restraint_set(), pst)
try:
    IMP.show_graphviz(mt)
except:
    print "Unable to display graph using 'dot'"

ors= IMP.domino.OptimizeRestraints(m.get_root_restraint_set(), pst)
filters=[IMP.domino.RestraintScoreSubsetFilterTable(m, pst),
         IMP.domino.ExclusionSubsetFilterTable(pst)]
leaf_table=IMP.domino.BranchAndBoundAssignmentsTable(pst, filters)

# create a database to store the results
name=IMP.create_temporary_file_name("assignments", ".hdf5")
root= RMF.create_hdf5_file(name)

# recurse down the tree getting the assignments and printing them
def get_assignments(vertex):
    on= mt.get_out_neighbors(vertex)
    ss= mt.get_vertex_name(vertex)
    print "computing assignments for", ss
    ssn= str(ss)
    dataset= root.add_child_index_data_set_2d(ssn)
    dataset.set_size([0, len(ss)])
    mine= IMP.domino.HDF5AssignmentContainer(dataset, ss, pst.get_particles(), ssn)
    if len(on)==0:
        # we have a leaf
        IMP.domino.load_leaf_assignments(ss, leaf_table, mine)
    else:
        # recurse on the two children
        (ss0, a0)= get_assignments(on[0])
        (ss1, a1)= get_assignments(on[1])
        IMP.domino.load_merged_assignments(ss0, a0, ss1, a1, filters, mine)
    print ss, mine.get_number_of_assignments()
    return (ss, mine)

# the root is the last vetex
all=get_assignments(mt.get_vertices()[-1])
all[1].set_was_used(True)

print 'try: h5dump', name
