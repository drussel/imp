import IMP

# a restraint which checks if particles are sorted in
# increasing order on k.
class MyRestraint(IMP.Restraint):
    # take the list of particles and the key to use
    def __init__(self, ps, k):
        IMP.Restraint.__init__(self)
        self.ps=ps
        self.k=k
    def unprotected_evaluate(self, da):
        score=0
        for i in range(1, len(self.ps)):
            p0= self.ps[i-1]
            p1= self.ps[i]
            if p0.get_value(k) > p1.get_value(k):
                diff=(p0.get_value(k)-p1.get_value(k))
                score= score+diff
                p0.add_to_derivative(k, -1, da)
                p1.add_to_derivative(k, 1, da)
            else:
                if IMP.get_log_level() >= IMP.TERSE:
                    print p0.get_name(), "and", p1.get_name(), " are ok"
        return score
    def get_input_particles(self):
        return self.ps
    def get_input_containers(self):
        return []

# some code to create and evaluate it
k= IMP.FloatKey("a key")
m= IMP.Model()
ps=[]
for i in range(0,10):
    p = IMP.Particle(m)
    p.add_attribute(k, i)
    ps.append(p)
r= MyRestraint(ps, k)
m.add_restraint(r)
#IMP.set_log_level(IMP.TERSE)
print r.evaluate(True)
