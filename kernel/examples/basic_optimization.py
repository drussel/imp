import IMP.example
import IMP.statistics

(m,c)=IMP.example.create_model_and_particles()
ps= IMP.core.DistancePairScore(IMP.core.HarmonicLowerBound(1,1))
r= IMP.container.PairsRestraint(ps, IMP.container.ClosePairContainer(c, 2.0))
m.add_restraint(r)
# we don't want to see lots of log messages about restraint evaluation
m.set_log_level(IMP.WARNING)

# the container (c) stores a list of particles, which are alse XYZR particles
# we can construct a list of all the decorated particles
xyzrs= IMP.core.XYZRsTemp(c.get_particles())

s= IMP.core.MCCGSampler(m)
s.set_number_of_attempts(10)
# but we do want something to watch
s.set_log_level(IMP.TERSE)
# find some configurations which move the particles far apart
configs= s.get_sample();
for i in range(0, configs.get_number_of_configurations()):
    configs.set_configuration(i)
    # print out the sphere containing the point set
    # - Why? - Why not?
    sphere= IMP.core.get_enclosing_sphere(xyzrs)
    print sphere

# cluster the solutions based on their coordinates
e= IMP.statistics.ConfigurationSetXYZEmbedding(configs, c)

# of course, this doesn't return anything of interest since the points are
# randomly distributed, but, again, why not?
clustering = IMP.statistics.get_lloyds_kmeans(e, 3, 1000)
for i in range(0,clustering.get_number_of_clusters()):
    # load the configuration for a central point
    configs.set_configuration(clustering.get_cluster_representative(i))
    seg= IMP.core.get_diameter(IMP.core.XYZsTemp(c.get_particles()))
    print (seg.get_point(0)-seg.get_point(1)).get_magnitude()
