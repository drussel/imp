/**
 *  \file BoxSweepClosePairsFinder.cpp
 *  \brief Test all pairs.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/base_types.h>
#include "IMP/core/BoxSweepClosePairsFinder.h"
#include "IMP/core/XYZDecorator.h"

#ifdef IMP_USE_CGAL

/* compile the CGAL code with NDEBUG since it doesn't have the
   same level of control over errors as IMP
*/
#ifndef NDEBUG
#define NDEBUG
#endif

#include <CGAL/box_intersection_d.h>
#include <vector>


IMPCORE_BEGIN_NAMESPACE


namespace {
struct NBLBbox
{
  XYZDecorator d_;
  typedef Float NT;
  typedef void * ID;
  Float r_;
  NBLBbox(){}
  NBLBbox(Particle *p,
                  Float r): d_(p),
                            r_(r){}
  static unsigned int dimension() {return 3;}
  void *id() const {return d_.get_particle();}
  NT min_coord(unsigned int i) const {
    return d_.get_coordinate(i)-r_;
  }
  NT max_coord(unsigned int i) const {
    return d_.get_coordinate(i)+r_;
  }
  // make it so I can reused the callback provide by NBLSS
  operator Particle*() const {return d_.get_particle();}
};

static void copy_particles_to_boxes(const SingletonContainer *ps,
                                    FloatKey rk, Float distance,
                                    std::vector<NBLBbox> &boxes)
{
  boxes.resize(ps->get_number_of_particles());
  for (unsigned int i=0; i< ps->get_number_of_particles(); ++i) {
    Particle *p= ps->get_particle(i);

    Float r= distance/2.0;
    if (rk != FloatKey() && p->has_attribute(rk)) {
      r+= p->get_value(rk);
    }
    boxes[i]=NBLBbox(p, r);
  }
}

struct AddToList {
  FilteredListPairContainer *out_;
  AddToList(FilteredListPairContainer *out): out_(out){}
  void operator()(const NBLBbox &a, const NBLBbox &b) {
    if (squared_distance(XYZDecorator(a).get_coordinates(),
                         XYZDecorator(b).get_coordinates())
        < square(a.r_ + b.r_)) {
      out_->add_particle_pair(ParticlePair(a,b));
    }
  }
};

}


BoxSweepClosePairsFinder::BoxSweepClosePairsFinder(){}

BoxSweepClosePairsFinder::~BoxSweepClosePairsFinder(){}

void BoxSweepClosePairsFinder
::add_close_pairs(SingletonContainer *ca,
                  SingletonContainer *cb,
                  FilteredListPairContainer *out) const {
  std::vector<NBLBbox> boxes0, boxes1;
  copy_particles_to_boxes(ca, get_radius_key(), get_distance(), boxes0);
  copy_particles_to_boxes(cb, get_radius_key(), get_distance(), boxes1);

  FilteredListPairContainerEditor e(out);

  CGAL::box_intersection_d( boxes0.begin(), boxes0.end(),
                            boxes1.begin(), boxes1.end(), AddToList(out));
}

void BoxSweepClosePairsFinder
::add_close_pairs(SingletonContainer *c,
                  FilteredListPairContainer *out) const {
  std::vector<NBLBbox> boxes;
  copy_particles_to_boxes(c, get_radius_key(), get_distance(), boxes);

  FilteredListPairContainerEditor e(out);

  CGAL::box_self_intersection_d( boxes.begin(), boxes.end(), AddToList(out));

}


IMPCORE_END_NAMESPACE
#endif /* IMP_USE_CGAL */
