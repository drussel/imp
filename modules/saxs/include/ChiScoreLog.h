/**
 * \file IMP/saxs/ChiScoreLog.h \brief scoring with log intensity
 *
 * Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPSAXS_CHI_SCORE_LOG_H
#define IMPSAXS_CHI_SCORE_LOG_H

#include "saxs_config.h"
#include "Profile.h"

IMPSAXS_BEGIN_NAMESPACE

class IMPSAXSEXPORT ChiScoreLog {
 public:
  Float compute_score(const Profile& exp_profile,
                      const Profile& model_profile,
                      bool use_offset = false) const;

  Float compute_scale_factor(const Profile& exp_profile,
                             const Profile& model_profile,
                             Float offset = 0.0) const;

  Float compute_offset(const Profile& exp_profile,
                       const Profile& model_profile) const {
    // not implemented as no straightforward solution to the equations
    return 0.0;
  }

};

IMPSAXS_END_NAMESPACE

#endif /* IMPSAXS_CHI_SCORE_LOG_H */
