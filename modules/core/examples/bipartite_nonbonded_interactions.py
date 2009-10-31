import IMP
import IMP.core

# This example addes a restraint on bipartite nonbonded interactions
# after excluding a set of bonded interactions.

m= IMP.Model()
# The set of particles
ps0 = IMP.core.ListSingletonContainer(IMP.core.create_xyzr_particles(m, 20, 1.0))
ps1 = IMP.core.ListSingletonContainer(IMP.core.create_xyzr_particles(m, 20, 2.0))

# Set up the nonbonded list
nbl= IMP.core.CloseBipartitePairContainer(ps0, ps1,0,1)

# Set up excluded volume
ps= IMP.core.SphereDistancePairScore(IMP.core.HarmonicLowerBound(0,1))
evr= IMP.core.PairsRestraint(ps, nbl)
m.add_restraint(evr)

# Set up optimizer
o= IMP.core.ConjugateGradients()
o.set_model(m)

o.optimize(1000)
