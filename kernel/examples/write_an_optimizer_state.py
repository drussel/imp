import IMP

# an optimizer state which prints out model statistics.
class MyOptimizerState(IMP.OptimizerState):
    def __init__(self):
        IMP.OptimizerState.__init__(self)
    def update(self):
        self.get_optimizer().get_model().show_restraint_score_statistics()
    def do_show(self, stream):
        print >> stream, ps

# some code to create and evaluate it
k= IMP.FloatKey("a key")
m= IMP.Model()
# we don't have any real restraints in the kernel
r0=IMP._ConstRestraint(1)
r0.set_name("restraint 0")
m.add_restraint(r0)
r1=IMP._ConstRestraint(2)
r1.set_name("restraint 1")
m.add_restraint(r1)

os= MyOptimizerState()
# we don't have any optimizers either
co= IMP._ConstOptimizer(m)
co.add_optimizer_state(os)
m.set_gather_statistics(True)
# so we only see the statistics
IMP.set_log_level(IMP.SILENT)
print co.optimize(100)
