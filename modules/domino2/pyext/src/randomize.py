# python files placed in this src directory are automatically added to the module.
# The function below can be accessed as IMP.domino2.randomize.randomize_particles().

import IMP.core

def randomize_particle(p):
    d= IMP.core.XYZ.decorate_particle(p)
    d.set_coordinates(IMP.algebra.get_random_vector_in(IMP.algebra.get_unit_bounding_box_3d()))
