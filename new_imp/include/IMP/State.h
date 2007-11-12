/**
 *  \file State.h   \brief Shared state.
 *
 *  Copyright 2007 Sali Lab. All rights reserved.
 *
 */

#ifndef __IMP_STATE_H
#define __IMP_STATE_H

#include <iostream>

#include "IMP_config.h"
#include "ModelData.h"
#include "boost/noncopyable.h"

namespace IMP
{
class Model;

//! Shared state.
class IMPDLLEXPORT State : public boost::noncopyable
{
  friend class Model;
  void set_model(Model* model);

public:
  State(std::string name=std::string());
  virtual ~State();

  // return the score for this restraint or set of restraints
  // ... given the current state of the model
  virtual void update() = 0;

  virtual void show(std::ostream& out = std::cout) const;
  virtual std::string version() const {
    return "unknown";
  }
  virtual std::string last_modified_by() const {
    return "unknown";
  }
  //! Get the name of the restraint
  const std::string& get_name() const {
    return name_;
  }
  //! Set the name of the restraint
  void set_name(const std::string &name) {
    name_=name;
  }

  //! return the stored model data
  Model *get_model() const {
    return model_;
  }
protected:
  // all of the particle data
  Model* model_;

  std::string name_;
};


inline std::ostream &operator<<(std::ostream &out, const State &s)
{
  s.show(out);
  return out;
}

} // namespace IMP

#endif  /* __IMP_STATE_H */
