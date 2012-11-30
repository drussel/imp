/**
 *  \file IMP/rmf/simple_links.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPRMF_SIMPLE_LINKS_H
#define IMPRMF_SIMPLE_LINKS_H

#include "rmf_config.h"
#include "links.h"
#include "associations.h"
#include <IMP/base/Object.h>
#include <IMP/base/Pointer.h>
#include <IMP/base/object_macros.h>
#include <IMP/base/log_macros.h>
#include <RMF/SetCurrentFrame.h>
#include <RMF/names.h>

IMPRMF_BEGIN_NAMESPACE

template <class O>
class SimpleLoadLink: public LoadLink {
  base::Vector<base::Pointer<O> > os_;
  RMF::NodeIDs nhs_;
  IMP_PROTECTED_METHOD(virtual void, do_load_one, ( RMF::NodeConstHandle nh,
                                                    O *o),
                       ,=0);
  IMP_PROTECTED_METHOD(void,
                       do_load,
                       (RMF::FileConstHandle fh),,
                       {
                         for (unsigned int i=0; i< os_.size();  ++i) {
                           do_load_one(fh.get_node_from_id(nhs_[i]), os_[i]);
                         }
                       });
  IMP_PROTECTED_METHOD(virtual void, do_add_link, (O *, RMF::NodeConstHandle),
                       ,{});
  IMP_PROTECTED_METHOD(void, add_link,(O *o, RMF::NodeConstHandle nh), ,
                       {
                         os_.push_back(o);
                         nhs_.push_back(nh.get_id());
                         set_association(nh, o, true);
                       });
  IMP_PROTECTED_METHOD(virtual bool, get_is,(RMF::NodeConstHandle nh),
                       const,=0);
  IMP_PROTECTED_METHOD(virtual O*, do_create,
                       (RMF::NodeConstHandle nh), ,=0);
  IMP_PROTECTED_CONSTRUCTOR(SimpleLoadLink, (std::string name),
                            : LoadLink(name){});
public:
  /** Create all the entities under the passed root.*/
  base::Vector<base::Pointer<O> > create(RMF::NodeConstHandle rt) {
    IMP_OBJECT_LOG;
    RMF::SetCurrentFrame sf(rt.get_file(), 0);
    RMF::NodeConstHandles ch= rt.get_children();
    base::Vector<base::Pointer<O> > ret;
    for (unsigned int i=0; i< ch.size(); ++i) {
      IMP_LOG(VERBOSE, "Checking " << ch[i] << std::endl);
      if (get_is(ch[i])) {
        IMP_LOG(VERBOSE, "Adding " << ch[i] << std::endl);
        base::Pointer<O> o=do_create(ch[i]);
        add_link(o, ch[i]);
        ret.push_back(o);
        o->set_was_used(true);
      }
    }
    return ret;
  }
  void link(RMF::NodeConstHandle rt,
            const base::Vector<base::Pointer<O> > &ps) {
    IMP_OBJECT_LOG;
    RMF::SetCurrentFrame sf(rt.get_file(), 0);
    set_was_used(true);
    RMF::NodeConstHandles ch= rt.get_children();
    int links=0;
    for (unsigned int i=0; i< ch.size(); ++i) {
      IMP_LOG(VERBOSE, "Checking " << ch[i] << std::endl);
      if (get_is(ch[i])) {
        IMP_LOG(VERBOSE, "Linking " << ch[i] << std::endl);
        if (ps.size() <= static_cast<unsigned int>(links)) {
          IMP_THROW("There are too many matching hierarchies in the rmf to "
                    << "link against " << ps, ValueException);
        }
        add_link(ps[links], ch[i]);
        ps[links]->set_was_used(true);
        do_add_link(ps[links], ch[i]);
        ++links;
      }
    }
    IMP_USAGE_CHECK(os_.size()==nhs_.size(),
                    "Didn't find enough matching things.");
    IMP_USAGE_CHECK(links==static_cast<int>(ps.size()),
                    "Didn't find enough matching things. Found "
                    << links << " wanted " << ps.size());
  }

};


template <class O>
class SimpleSaveLink: public SaveLink {
  base::Vector<base::Pointer<O> > os_;
  RMF::NodeIDs nhs_;

  IMP_PROTECTED_METHOD(virtual void,  do_save_one, (O *o,
                                                    RMF::NodeHandle nh),,=0);
  IMP_PROTECTED_METHOD(void, do_save, (RMF::FileHandle fh),,
                       {
                         for (unsigned int i=0; i< os_.size();  ++i) {
                           os_[i]->set_was_used(true);
                           IMP_LOG(VERBOSE, "Saving " << Showable(os_[i])
                                   << std::endl);
                           do_save_one(os_[i], fh.get_node_from_id(nhs_[i]));
                         }
                       });
  IMP_PROTECTED_METHOD(virtual void, do_add, (O *o, RMF::NodeHandle c),,
                       {
                         add_link(o, c);
                       });
  IMP_PROTECTED_METHOD(virtual RMF::NodeType, get_type, (O*o), const, =0);
  IMP_PROTECTED_METHOD(void, add_link,(O *o, RMF::NodeConstHandle nh),,
                       {
                         os_.push_back(o);
                         nhs_.push_back(nh.get_id());
                         set_association(nh, o, true);
                       });
  IMP_PROTECTED_CONSTRUCTOR(SimpleSaveLink,(std::string name),
                            : SaveLink(name) {});
 public:
  void add(RMF::NodeHandle parent,
           const base::Vector<base::Pointer<O> > &os) {
    IMP_OBJECT_LOG;
    RMF::SetCurrentFrame sf(parent.get_file(), RMF::ALL_FRAMES);
    for (unsigned int i=0; i< os.size(); ++i) {
      std::string nicename= RMF::get_as_node_name(os[i]->get_name());
      RMF::NodeHandle c= parent.add_child(nicename,
                                          get_type(os[i]));
      do_add(os[i], c);
      os[i]->set_was_used(true);
    }
  }
};

IMPRMF_END_NAMESPACE

#endif /* IMPRMF_SIMPLE_LINKS_H */
