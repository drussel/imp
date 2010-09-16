# Assume p is a RigidBody Particle
rbd= IMP.core.RigidBody(p)
translation=IMP.algebra.get_random_vector_in(IMP.algebra.Vector3D(0,0,0),
                                                 10.0)
# we don't yet have python code to generate a nearby rotation
rotation= IMP.algebra.get_random_rotation_3d()
transformation= IMP.algebra.Transformation3D(rotation, translation)
# note, this overwrites the existing position
# The True is to transform the members now rather than wait for a
# score state
rbd.set_transformation(transformation)
# to instead perturb the existing transformation instead do
rbd.set_transformation(IMP.algebra.compose(rbd.get_transformation(),
                                                           transformation))
