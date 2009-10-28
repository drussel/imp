import IMP
import IMP.core
import IMP.atom

# This example addes a restraint on nonbonded interactions
# after excluding a set of bonded interactions.

m= IMP.Model()
# The set of particles
ps = IMP.core.ListSingletonContainer(IMP.core.create_xyzr_particles(m, 20, 1.0))

# create a bond between two particles
bd0= IMP.atom.Bonded.setup_particle(ps.get_particle(0))
bd1= IMP.atom.Bonded.setup_particle(ps.get_particle(1))
IMP.atom.custom_bond(bd0, bd1, 2.0)

# Set up the nonbonded list for all pairs at are touching
# and let things move 3 before updating the list
nbl= IMP.core.ClosePairContainer(ps, 0.0, 3.0)
nbl.add_pair_filter(IMP.atom.BondedPairFilter())

# Set up excluded volume
sdps= IMP.core.SphereDistancePairScore(IMP.core.HarmonicLowerBound(0,1))
evr= IMP.core.PairsRestraint(sdps, nbl)
m.add_restraint(evr)

# Set up optimizer
o= IMP.core.ConjugateGradients()
o.set_model(m)

o.optimize(1000)
