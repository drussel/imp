import IMP
import IMP.core
import IMP.atom

m= IMP.Model()
prot= IMP.atom.read_pdb('examples/simple_examples/single_protein.pdb', m)
res= IMP.atom.get_by_type(prot, IMP.atom.Hierarchy.RESIDUE)
rc= IMP.core.ListSingletonContainer(res)
for p in res:
    IMP.core.XYZR.create(p)
mtr=IMP.atom.Hierarchy.get_traits()
pr= IMP.core.ChildrenRefiner(mtr)
IMP.core.create_covers(rc, pr)
m.evaluate(False)
