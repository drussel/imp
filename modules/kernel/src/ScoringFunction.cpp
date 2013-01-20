/**
 *  \file Model.cpp \brief Storage of a model, its restraints,
 *                         constraints and particles.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/ScoringFunction.h"
#include "IMP/Model.h"
#include "IMP/internal/evaluate_utility.h"
#include "IMP/internal/scoring_functions.h"
#include "IMP/scoring_function_macros.h"
#include "IMP/internal/utility.h"
#include "IMP/generic.h"
#include "IMP/utility.h"



IMP_BEGIN_NAMESPACE

// in namespace so it can be made a friend.
class NullScoringFunction: public ScoringFunction {
public:
  NullScoringFunction() {}
  IMP_SCORING_FUNCTION(NullScoringFunction);
};
void
NullScoringFunction::do_add_score_and_derivatives(ScoreAccumulator ,
                                         const ScoreStatesTemp &) {
}
Restraints NullScoringFunction::create_restraints() const {
  return Restraints();
}
ScoreStatesTemp
NullScoringFunction
::get_required_score_states(const DependencyGraph &,
                            const DependencyGraphVertexIndex&) const {
  return ScoreStatesTemp();
}
void NullScoringFunction::do_show(std::ostream &) const {
}

ScoringFunction::ScoringFunction(Model *m,
                                 std::string name): ModelObject(m, name){
}
double ScoringFunction::evaluate_if_good(bool derivatives) {
  IMP_OBJECT_LOG;
  set_was_used(true);
  ensure_dependencies();
  es_.score=0;
  es_.good=true;
  const ScoreAccumulator sa=get_score_accumulator_if_good(derivatives);
  do_add_score_and_derivatives(sa, ss_);
  return es_.score;
}
double ScoringFunction::evaluate(bool derivatives) {
  IMP_OBJECT_LOG;
  set_was_used(true);
  ensure_dependencies();
  es_.score=0;
  es_.good=true;
  const ScoreAccumulator sa=get_score_accumulator(derivatives);
  do_add_score_and_derivatives(sa, ss_);
  return es_.score;
}
double ScoringFunction::evaluate_if_below(bool derivatives, double max) {
  IMP_OBJECT_LOG;
  set_was_used(true);
  ensure_dependencies();
  es_.score=0;
  es_.good=true;
  const ScoreAccumulator sa=get_score_accumulator_if_below(derivatives,
                                                     max);
  do_add_score_and_derivatives(sa, ss_);
  return es_.score;
}
void
ScoringFunction::do_update_dependencies(const DependencyGraph &dg,
                                     const DependencyGraphVertexIndex &index) {
  // can't check here as create_restraints can cause a loop
  // but we must make sure they are ordered
  ss_= get_update_order(get_required_score_states(dg, index));
}

ScoreStatesTemp
ScoringFunction::get_required_score_states(const DependencyGraph &dg,
                                           const DependencyGraphVertexIndex &i)
    const {
  Restraints restraints=create_restraints();
  ModelObjectsTemp rs= get_model()->get_optimized_particles()\
    + ModelObjectsTemp(restraints.begin(), restraints.end());
  IMP_INTERNAL_CHECK(!get_model() || get_model()->get_has_dependencies(),
                     "ScoringFunctions where create_restraints() creates "
                     << "new restraints must implement their own"
                     << " get_required_score_states()");
  return IMP::get_required_score_states(rs , dg, i);
}

void
ScoringFunction::clear_caches() {
  get_model()->clear_caches();
}

ScoringFunction*
ScoringFunctionAdaptor::get(const RestraintsTemp &sf) {
  if (!sf.empty()) {
    return new internal::RestraintsScoringFunction(sf);
  } else {
    return new NullScoringFunction();
  }}

ScoringFunction*
ScoringFunctionAdaptor::get(const Restraints&sf) {
  if (!sf.empty()) {
    return new internal::RestraintsScoringFunction(sf);
  } else {
    return new NullScoringFunction();
  }
}
ScoringFunction* ScoringFunctionAdaptor::get(Model *sf) {
  return sf->create_scoring_function();
}
ScoringFunction* ScoringFunctionAdaptor::get(Restraint *sf) {
  return sf->create_scoring_function();
}

namespace {
  unsigned int sf_num_children(Restraint*r) {
    RestraintSet *rs= dynamic_cast<RestraintSet*>(r);
    if (rs) return rs->get_number_of_restraints();
    else return 0;
  }
}


void show_restraint_hierarchy(ScoringFunctionAdaptor r, std::ostream &out) {
  Restraints cur= r->create_restraints();
  for (unsigned int i=0; i< cur.size(); ++i) {
      Restraint*r= cur[i];
      RestraintSet *rs=dynamic_cast<RestraintSet*>(r);
       if (!rs) {
         IMP_PRINT_TREE(out, Restraint*, r, 0,
                        dynamic_cast<RestraintSet*>(n)->get_restraint,
                        out << Showable(n)
                        << " " << n->get_maximum_score() << " "
                        << n->get_weight() );
       } else {
         IMP_PRINT_TREE(out, Restraint*, rs, sf_num_children(n),
                        dynamic_cast<RestraintSet*>(n)->get_restraint,
                        out << Showable(n)
                        << " " << n->get_maximum_score() << " "
                        << n->get_weight() );
       }
  }
}

namespace {
ScoringFunctions create_decomposition(Restraint *r, double w, double max) {
  if (!r) return ScoringFunctions();
  RestraintSet* rs= dynamic_cast<RestraintSet*>(r);
  if (rs) {
    ScoringFunctions ret;
    for (RestraintSet::RestraintIterator it=rs->restraints_begin();
         it != rs->restraints_end(); ++it) {
      ret= ret+create_decomposition(*it, w* rs->get_weight(),
                               std::min(max, rs->get_maximum_score()));
    }
    return ret;
  } else {
    return ScoringFunctions(1, r->create_scoring_function(w, max));
  }
}

ScoringFunctions
create_decomposition_into_scoring_functions(const RestraintsTemp &sf) {
  ScoringFunctions ret;
  for (unsigned int i=0; i< sf.size(); ++i) {
    base::Pointer<Restraint> r= sf[i]->create_decomposition();
    ret= ret+ create_decomposition(r, 1.0, NO_MAX);
  }
  return ret;
}


}

ScoringFunctions create_decomposition(ScoringFunction *sf) {
  ScoringFunctions ret;
  ret= create_decomposition_into_scoring_functions(sf->create_restraints());
  return ret;
}
IMP_END_NAMESPACE
