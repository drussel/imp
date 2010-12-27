import IMP
import IMP.core
import IMP.algebra
import IMP.atom

m= IMP.Model()

# Put the parent particles for each molecule
hs=[]

# create the molecules
for i in range(0,10):
    p=IMP.Particle(m)
    d= IMP.atom.Hierarchy.setup_particle(p)
    for j in range(0,5):
        p=IMP.Particle(m)
        cd= IMP.atom.Fragment.setup_particle(p)
        d.add_child(cd)
        xd= IMP.core.XYZR.setup_particle(p, IMP.algebra.Sphere3D(IMP.algebra.Vector3D(3*i,j,0), 1))
    hs.append(p)

ps= IMP.core.SphereDistancePairScore(IMP.core.HarmonicUpperBound(0,1))
cps= IMP.core.ChildrenRefiner(IMP.atom.Hierarchy.get_traits())
lrps = IMP.core.KClosePairsPairScore(ps, cps, 1)
cr = IMP.core.ConnectivityRestraint(lrps)
cr.set_particles(hs)
m.add_restraint(cr)

m.evaluate(False)
