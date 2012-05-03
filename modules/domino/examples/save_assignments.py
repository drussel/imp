import IMP.domino
import RMF

# create a model and some particles, they are just used as markers here
m= IMP.Model()
ps= [IMP.Particle(m) for i in range(0,10)]

# create a subset with a few of the particles
ss= IMP.domino.Subset([ps[3], ps[5], ps[7]])

file_name= IMP.create_temporary_file_name("assignments", ".hdf5")

print "File name is", file_name

# create a list of assignments
wcn=IMP.create_temporary_file_name("assignments", ".asn")
asl=IMP.domino.WriteAssignmentContainer(wcn, ss, ps, "writer")
written=[]
for i in range(0,5):
    for j in range(0,5):
        for k in range(0,5):
            a=IMP.domino.Assignment([i,j,k])
            written.append(a)
            asl.add_assignment(a)

del asl

# to check, we can read it back immediately
back_asl= IMP.domino.ReadAssignmentContainer(wcn, ss, ps, "reader")


if back_asl.get_assignments()==written:
    print "They match!"
else:
    print "They don't match :-("
    raise 1


# More interestingly, we can create a new model and read back the assignments for that
mp= IMP.Model()
psp= [IMP.Particle(mp) for i in range(0,10)]

# create a subset with a few of the particles
ssp= IMP.domino.Subset([psp[3], psp[5], psp[7]])

print "Note the subsets are differently ordered (most of the time): ", ss, ssp

back_aslp= IMP.domino.ReadAssignmentContainer(wcn, ssp, psp, "reader2")

# however, the states are demuxed so they match the particles
print [str(a) for a in back_aslp.get_assignments()]
