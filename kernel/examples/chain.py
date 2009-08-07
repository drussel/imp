import IMP
import IMP.core
import random
import IMP.display

# This example constructs a set of particles which are restrained
# to form a chain via bonds between successive particles. In addition
# the head and the tail of the chain are restrained to be close to one
# another.

#IMP.set_log_level(IMP.VERBOSE)
m= IMP.Model()
# The particles in the chain
chain= IMP.core.ListSingletonContainer(IMP.core.create_xyzr_particles(m, 20, 1.0))

# create a bond between successive particles
IMP.atom.Bonded.setup_particle(chain.get_particle(0))
bonds= IMP.core.ListSingletonContainer()
for i in range(1, chain.get_number_of_particles()):
    bp= IMP.atom.Bonded.decorate_particle(chain.get_particle(i-1))
    bpr= IMP.atom.Bonded.setup_particle(chain.get_particle(i))
    b= IMP.atom.custom_bond(bp, bpr, 1.5, 10)
    bonds.add_particle(b.get_particle())

# If you want to inspect the particles
# Notice that each bond is a particle
for p in m.get_particles():
    p.show()

# Set up the nonbonded list
nbl= IMP.core.ClosePairsScoreState(chain)
m.add_score_state(nbl)
# Exclude bonds from closest pairs
fl= nbl.get_close_pairs_container()
bpc=IMP.atom.BondPairFilter()
nbl.add_close_pair_filter(bpc)

# Set up excluded volume
ps= IMP.core.SphereDistancePairScore(IMP.core.HarmonicLowerBound(0,1))
evr= IMP.core.PairsRestraint(ps, fl)
m.add_restraint(evr)

# Restraint for bonds
bss= IMP.atom.BondSingletonScore(IMP.core.Harmonic(0,1))
br= IMP.core.SingletonsRestraint(bss, bonds)
m.add_restraint(br)

# Tie the ends of the chain
# We cound have used a bond instead
p= IMP.ParticlePair(chain.get_particle(0), chain.get_particle(chain.get_number_of_particles()-1))
pps= IMP.core.ListPairContainer()
pps.add_particle_pair(p)
cr= IMP.core.PairsRestraint(
           IMP.core.SphereDistancePairScore(IMP.core.Harmonic(3,1)), pps)
m.add_restraint(cr)

# Set up optimizer
o= IMP.core.ConjugateGradients()
o.set_model(m)

# Write the progression of states as the system is optimized to
# the files state.000.vrml, state.001.vrml etc.
vrml= IMP.display.LogOptimizerState(IMP.display.VRMLWriter(), "state.%03d.vrml")
for p in chain.get_particles():
    vrml.add_geometry(IMP.display.XYZRGeometry(IMP.core.XYZR(p)))
vrml.set_skip_steps(100)
o.add_optimizer_state(vrml)

# We probably don't need this many steps
o.optimize(1000)
