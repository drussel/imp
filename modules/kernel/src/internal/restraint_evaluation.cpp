/**
 *  \file static.cpp   \brief all static data for module.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/restraint_evaluation.h"
#include "IMP/ScoringFunction.h"
#include "IMP/Model.h"
#include "IMP/container_base.h"
#include "IMP/ScoreAccumulator.h"
#include <boost/scoped_ptr.hpp>
#include <boost/timer.hpp>
#include "IMP/ModelObject.h"

#include "IMP/internal/evaluate_utility.h"
#include "IMP/internal/utility.h"
#include <numeric>

IMP_BEGIN_INTERNAL_NAMESPACE

namespace {

void before_protected_evaluate(Model *m,
                               const ScoreStatesTemp &states,
                               bool derivative) {
  IMP_CHECK_OBJECT(m);
  m->before_evaluate(states);
  if (derivative) {
    m->zero_derivatives();
  }
}
void after_protected_evaluate(Model *m,
                              const ScoreStatesTemp &states,
                              bool derivative) {
  m->after_evaluate(states, derivative);
  // validate derivatives
  IMP_IF_CHECK(base::USAGE_AND_INTERNAL) {
    if (derivative) {
      m->validate_computed_derivatives();
    }
  }
  ++m->eval_count_;
  m->first_call_=false;
}

template <class RS>
void do_evaluate_one(IMP::ScoreAccumulator sa,
                            RS* restraint,
                            Model *m) {
#if IMP_BUILD < IMP_FAST && !defined(_OPENMP)
  if (m->first_call_) {
    internal::SFResetBitset rbr(m->Masks::read_mask_, true);
    internal::SFResetBitset rbw(m->Masks::write_mask_, true);
    internal::SFResetBitset rbar(m->Masks::add_remove_mask_, true);
    internal::SFResetBitset rbrd(m->Masks::read_derivatives_mask_, true);
    internal::SFResetBitset rbwd(m->Masks::write_derivatives_mask_, true);
    m->Masks::write_mask_.reset();
    m->Masks::add_remove_mask_.reset();
    m->Masks::read_derivatives_mask_.reset();
    IMP_SF_SET_ONLY(m->Masks::read_mask_, restraint->get_inputs()
                    );
    IMP_SF_SET_ONLY(m->Masks::write_derivatives_mask_,
                    restraint->get_inputs()
                    );
    IMP_SF_SET_ONLY(m->Masks::read_derivatives_mask_,
                    restraint->get_inputs()
                    );
    IMP_CHECK_OBJECT(restraint);
    restraint->add_score_and_derivatives(sa);;
  } else {
    IMP_CHECK_OBJECT(restraint);
    restraint->add_score_and_derivatives(sa);;
  }
#else
  IMP_UNUSED(m);
  restraint->add_score_and_derivatives(sa);
#endif
}

template <class RS>
void protected_evaluate_many(IMP::ScoreAccumulator sa,
                             const RS &restraints,
                             const ScoreStatesTemp &states,
                             Model *m) {
  before_protected_evaluate(m, states, sa.get_derivative_accumulator());
  internal::SFSetIt<IMP::internal::Stage>
    reset(&m->cur_stage_, internal::EVALUATING);
  {
    for (unsigned int i=0; i<restraints.size(); ++i) {
      do_evaluate_one(sa, restraints[i].get(), m);
    }
#pragma omp taskwait
#pragma omp flush
  }
  after_protected_evaluate(m, states, sa.get_derivative_accumulator());
}

template <class RS>
 void unprotected_evaluate_one(IMP::ScoreAccumulator sa,
                                 RS* restraint,
                                 Model *m) {
  internal::SFSetIt<IMP::internal::Stage>
    reset(&m->cur_stage_, internal::EVALUATING);
  do_evaluate_one(sa, restraint, m);
}


template <class RS>
 void protected_evaluate_one(IMP::ScoreAccumulator sa,
                               RS* restraint,
                               const ScoreStatesTemp &states,
                               Model *m) {
  before_protected_evaluate(m, states, sa.get_derivative_accumulator());
  {
    unprotected_evaluate_one(sa, restraint, m);
#pragma omp taskwait
#pragma omp flush
  }
  after_protected_evaluate(m, states, sa.get_derivative_accumulator());
}

}

void protected_evaluate(IMP::ScoreAccumulator sa,
                        Restraint* restraint,
                        const ScoreStatesTemp &states,
                        Model *m) {
  protected_evaluate_one<Restraint>(sa, restraint, states, m);
}

void protected_evaluate(IMP::ScoreAccumulator sa,
                        const RestraintsTemp &restraints,
                        const ScoreStatesTemp &states,
                        Model *m) {
  protected_evaluate_many<RestraintsTemp>(sa, restraints, states, m);
}

void protected_evaluate(IMP::ScoreAccumulator sa,
                        const Restraints &restraints,
                        const ScoreStatesTemp &states,
                        Model *m) {
  protected_evaluate_many<Restraints>(sa, restraints, states, m);
}

IMP_END_INTERNAL_NAMESPACE
