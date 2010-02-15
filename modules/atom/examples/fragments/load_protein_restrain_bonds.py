prot= IMP.atom.read_pdb("some.pdb")
bds= IMP.core.get_internal_bonds(prot)
ps= IMP.Particles()
for b in bds:
    ps.append(b.get_particle())
bl= IMP.container.ListSingletonContainer(ps)
h= IMP.core.Harmonic(0,1)
bs= IMP.core.BondSingletonScore(h)
br= IMP.core.SingletonsRestraint(bs, bl)
m.add_restraint(br)
