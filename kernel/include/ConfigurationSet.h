/**
 *  \file ConfigurationSet.h
 *  \brief Store a set of configurations of the model.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_CONFIGURATION_SET_H
#define IMP_CONFIGURATION_SET_H

#include "kernel_config.h"
#include "Object.h"
#include "Pointer.h"
#include "Model.h"
#include "Configuration.h"
#include "internal/OwnerPointer.h"
#include "OptimizerState.h"
#include "FailureHandler.h"
#include "internal/utility.h"
#include "compatibility/map.h"
#include <set>

IMP_BEGIN_NAMESPACE


//! A class to store a set of configurations of a model
/** The class maintains a set of structures found by a Sampler
    protocol and allows them to be queried and loaded and saved.
    \pythonexample{basic_optimization}
*/
class IMPEXPORT ConfigurationSet: public Object
{
  mutable Pointer<Model> model_;
  Pointer<Configuration> base_;
  VectorOfRefCounted<Configuration*> configurations_;
 public:
  ConfigurationSet(Model *m, std::string name="ConfigurationSet %1%");
  //! Save the current configuration of the Model
  void save_configuration();
  unsigned int get_number_of_configurations() const;
  //! Load the ith configuration into the Model
  /** Passing -1 returns it to the base configuration. */
  void load_configuration(int i) const;
  void remove_configuration(unsigned int i);
  Model *get_model() const {
    return model_;
  }
  IMP_OBJECT(ConfigurationSet);
};

IMP_OBJECTS(ConfigurationSet,ConfigurationSets);

IMP_MODEL_SAVE(SaveToConfigurationSet,
               (ConfigurationSet *cs, std::string file_name),
               mutable internal::OwnerPointer<ConfigurationSet> cs_;,
               cs_=cs;,
               ,
               {
                 IMP_LOG(TERSE, "Saving to configuration set "
                         << file_name << std::endl);
                 cs_->save_configuration();
               });


IMP_END_NAMESPACE

#endif  /* IMP_CONFIGURATION_SET_H */
