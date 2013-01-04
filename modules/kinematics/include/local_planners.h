/**
 * \file local_planners \brief
 *
 * Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPKINEMATICS_LOCAL_PLANNERS_H
#define IMPKINEMATICS_LOCAL_PLANNERS_H

#include "kinematics_config.h"
#include "DOFValues.h"
#include "directional_DOFs.h"
#include "DOFsSampler.h"

IMPKINEMATICS_BEGIN_NAMESPACE

/* general interface */
class IMPKINEMATICSEXPORT LocalPlanner{
public:
  LocalPlanner(Model* model, DOFsSampler* sampler) :
    model_(model), sampler_(sampler) {}

  virtual std::vector<DOFValues> plan(DOFValues q_near,
                                      DOFValues q_rand) = 0;

  bool is_valid(const DOFValues& values) {
    sampler_->apply(values);
    double score = model_->evaluate(false);
    std::cerr << "score = " << score << std::endl;
    return true; //(model_->evaluate(false)>0)?false:true; //_if_below(); //???
  }

protected:
  Model* model_;
  DOFsSampler* sampler_;
};


/*
  Local planner implementation that samples conformations on a path
  between two nodes
*/
class IMPKINEMATICSEXPORT PathLocalPlanner : public LocalPlanner {
public:
  // default path sampling is linear
  PathLocalPlanner(Model* model, DOFsSampler* sampler,
                   DirectionalDOF* directional_dof,
                   int save_step_interval = 1) :
    LocalPlanner(model, sampler),
    d_(directional_dof),
    save_step_interval_(save_step_interval) {}

protected:

  // plan a path of intermediate nodes with a properly calculated
  // step size from existing node q_from until the valid node that is
  // found closest to q_rand (inclusive)
  virtual std::vector<DOFValues> plan(DOFValues q_from, DOFValues q_rand);

private:
  DirectionalDOF* d_;
  unsigned int save_step_interval_; // if 0, save only last valid node
};

IMPKINEMATICS_END_NAMESPACE

#endif /* IMPKINEMATICS_LOCAL_PLANNERS_H */
