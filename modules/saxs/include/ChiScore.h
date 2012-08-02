/**
 * \file ChiScore.h \brief Basic chi score implementation
 *
 * Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPSAXS_CHI_SCORE_H
#define IMPSAXS_CHI_SCORE_H

#include "saxs_config.h"
#include "Profile.h"

IMPSAXS_BEGIN_NAMESPACE

class IMPSAXSEXPORT ChiScore {
 public:
  Float compute_score(const Profile& exp_profile,
                      const Profile& model_profile,
                      bool use_offset = false) const;

  Float compute_scale_factor(const Profile& exp_profile,
                             const Profile& model_profile,
                             Float offset = 0.0) const;

  Float compute_offset(const Profile& exp_profile,
                       const Profile& model_profile) const;
};

IMPSAXS_END_NAMESPACE

#endif /* IMPSAXS_CHI_SCORE_H */
