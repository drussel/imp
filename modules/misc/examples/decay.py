import IMP.atom
import IMP.container
import IMP.misc
import IMP.display
import IMP.example
import IMP.rmf
import RMF
import random

if IMP.build=="debug":
    np=5
    nb=5
    prob=.5
    period=10
    steps=10
else:
    np=50
    nb=150
    prob=.1
    period=10
    steps=10000

m= IMP.Model()

bb= IMP.algebra.BoundingBox3D(IMP.algebra.Vector3D(0,0,0),
                              IMP.algebra.Vector3D(100,100,100))
ps=[]
for i in range(0,np):
    p= IMP.Particle(m)
    d= IMP.core.XYZR.setup_particle(p)
    d.set_coordinates(IMP.algebra.get_random_vector_in(bb))
    d.set_radius(10)
    d.set_coordinates_are_optimized(True)
    IMP.atom.Hierarchy.setup_particle(p)
    IMP.atom.Diffusion.setup_particle(p)
    IMP.atom.Mass.setup_particle(p, 1)
    ps.append(p)
    IMP.display.Colored.setup_particle(p, IMP.display.get_display_color(i))

bds=[]
for i in range(0,nb):
    pp= random.sample(ps, 2)
    if pp not in bds and [pp[1], pp[0]] not in bds:
        bds.append(pp)

cf= IMP.core.CoinFlipPairPredicate(prob)
dos= IMP.misc.DecayPairContainerOptimizerState(cf, bds, "decay")
dos.set_period(period)

# create restraints
rs=[]
box_score=IMP.core.BoundingBox3DSingletonScore(IMP.core.HarmonicUpperBound(0,10), bb)
rs.append(IMP.container.SingletonsRestraint(box_score, ps, "box"))
bond_score=IMP.core.HarmonicUpperBoundSphereDistancePairScore(0,10)
rs.append(IMP.container.PairsRestraint(bond_score,
                                       dos.get_output_container(),
                                       "bonds"))
ev= IMP.core.ExcludedVolumeRestraint(ps, 10,10)
# equilibrate
print "equilibrating"
IMP.set_log_level(IMP.PROGRESS)
IMP.example.optimize_balls(ps, rs)

# set up simulator
bd= IMP.atom.BrownianDynamics(m)
bd.set_maximum_time_step(1000)
bd.set_scoring_function(rs+[ev])
bd.add_optimizer_state(dos)

# set up display
fn=IMP.base.create_temporary_file_name("decay", ".rmf")
rmf= RMF.create_rmf_file(fn)
# silliness that will go away
bd.get_scoring_function().evaluate(False)
IMP.rmf.add_hierarchies(rmf, ps)
IMP.rmf.add_restraints(rmf, rs+[ev])
g= IMP.display.BoundingBoxGeometry(bb)
IMP.rmf.add_geometries(rmf, [g])
os= IMP.rmf.SaveOptimizerState(rmf)
bd.add_optimizer_state(os)

# actually optimize things
print "running"
bd.optimize(steps)

print "see", fn
