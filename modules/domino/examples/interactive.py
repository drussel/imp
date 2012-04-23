import IMP.domino
import IMP.algebra
import IMP.container
import IMP

m = IMP.Model()

# create some particles
ps=[IMP.core.XYZ.setup_particle(IMP.Particle(m)) for x in range(0,3)]

s= IMP.core.HarmonicDistancePairScore(1, 1)
lpc= IMP.container.ListPairContainer([(ps[i[0]], ps[i[1]]) for i in [(0,1), (1,2)]])
print [(p[0].get_name(), p[1].get_name()) for p in lpc.get_particle_pairs()]
r= IMP.container.PairsRestraint(s, lpc)
r.set_model(m)
r.set_maximum_score(.1)

space= IMP.domino.XYZStates([IMP.algebra.Vector3D(i, 0, 0) for i in range(0,6)])

pst= IMP.domino.ParticleStatesTable()
for p in ps:
    pst.set_particle_states(p, space)

m.set_log_level(IMP.SILENT)

# make sure to break up the
mt= IMP.domino.get_merge_tree([r], pst)
try:
    IMP.show_graphviz(mt)
except:
    print "Unable to display graph using 'dot'"

ds= IMP.domino.DominoSampler(m, pst)
# use the default setup for filters
ds.set_scoring_function([r])
ds.set_merge_tree(mt)
ds.set_log_level(IMP.SILENT)

# recurse down the tree getting the assignments and printing them
def get_assignments(vertex):
    on= mt.get_out_neighbors(vertex)
    if len(on)==0:
        # we have a leaf
        ret= ds.get_vertex_assignments(vertex)
    else:
        # recurse on the two children
        if on[0] > on[1]:
            # the get_vertex_assignment methods expects the children in sorted order
            on=[on[1], on[0]]
        a0= get_assignments(on[0])
        a1= get_assignments(on[1])
        ret= ds.get_vertex_assignments(vertex, a0, a1)
    print mt.get_vertex_name(vertex), [str(r) for r in ret]
    return ret

# the root is the last vetex
get_assignments(mt.get_vertices()[-1])


schedule=[]
# we could instead decompose the tree into independent sets of jobs
def schedule_job(vertex):
    # first figure out the maximum phase for my children, then add me to the
    # next higher phase
    on= mt.get_out_neighbors(vertex)
    max_child_time=-1
    for n in on:
        max_child_time=max(schedule_job(n), max_child_time)
    my_time=max_child_time+1
    while len(schedule) < my_time+1:
        schedule.append([])
    # add myself to the next free phase
    schedule[my_time].append(vertex)
    return my_time

schedule_job(mt.get_vertices()[-1])
print "The merging can be scheduled as", schedule
