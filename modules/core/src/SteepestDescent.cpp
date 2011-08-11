/**
 *  \file SteepestDescent.cpp \brief Simple steepest descent optimizer.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/SteepestDescent.h>

#include <IMP/log.h>
#include <IMP/Model.h>

IMPCORE_BEGIN_NAMESPACE

//! Constructor
SteepestDescent::SteepestDescent(Model *m) : step_size_(0.01), threshold_(0.)
{
  if (m) set_model(m);
}
void SteepestDescent::do_show(std::ostream &) const {
}


double SteepestDescent::do_optimize(unsigned int max_steps)
{
  Floats temp_vals;
  Floats temp_derivs;
  Float last_score, new_score = 0.0;

  // set up the indexes


  FloatIndexes float_indexes=get_optimized_attributes();
  int opt_var_cnt = float_indexes.size();

  Float current_step_size = step_size_;

  // ... and space for the old values
  temp_vals.resize(opt_var_cnt);
  temp_derivs.resize(opt_var_cnt);

  for (unsigned int step = 0; step < max_steps; step++) {
    IMP_LOG(VERBOSE, "=== Step " << step << " ===");
    // model.show(std::cout);
    int cnt = 0;

    // evaluate the last model state
    last_score = evaluate(true);
    IMP_LOG(VERBOSE, "start score: " << last_score);

    // store the old values
    for (int i = 0; i < opt_var_cnt; i++) {
      FloatIndex fi = float_indexes[i];

      temp_vals[i] = get_value(fi);
      temp_derivs[i] = get_derivative(fi);
    }

    bool done = false;
    bool not_changing = false;
    // try to increase step_size if we are moving to slowly down gradient
    bool move_bigger;
    while (!done && (cnt < 200)) {
      cnt++;
      move_bigger = true;

      // try new values based on moving down the gradient at the current
      // step size
      for (int i = 0; i < opt_var_cnt; i++) {
        IMP_LOG(VERBOSE, i << " move: " << temp_vals[i] << " new: "
                << temp_vals[i] - temp_derivs[i] * current_step_size << "  "
                << temp_derivs[i]);

        set_value(float_indexes[i], temp_vals[i] - temp_derivs[i]
                                    * current_step_size);
      }

      // check the new model
      new_score = evaluate(false);
      IMP_LOG(VERBOSE, "last score: " << last_score << "  new score: "
              << new_score << "  step size: " << current_step_size);

      // if the score is less than the threshold, we're done
      if (new_score <= threshold_) {
        update_states();
        return new_score;
      }

      // if the score got better, we'll take it
      if (new_score < last_score) {
        done = true;
        update_states();
        if (move_bigger)
          current_step_size *= 1.4;
      }

      // if the score is the same, keep going one more time
      else if (std::abs(new_score- last_score) < .0000001) {
        if (not_changing)
          done = true;

        current_step_size *= 0.9;
        not_changing = true;
      }

      // if the score got worse, our step size was too big
      // ... make it smaller, and try again
      else {
        not_changing = false;
        move_bigger = false;
        current_step_size *= 0.71;
      }

      if (current_step_size <.00000001) {
        update_states();
        return new_score;
      }
    }
    update_states();
  }

  return new_score;
}

IMPCORE_END_NAMESPACE
