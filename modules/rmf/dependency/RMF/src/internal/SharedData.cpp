/**
 *  \file RMF/Category.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include <RMF/internal/SharedData.h>
#include <RMF/NodeHandle.h>
#include <RMF/Validator.h>
#include <RMF/internal/set.h>
#include <RMF/HDF5File.h>
#include <boost/filesystem/path.hpp>
#include <RMF/internal/HDF5SharedData.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/version.hpp>

namespace RMF {
  namespace internal {


    namespace {
      map<std::string, SharedData *> cache;
      map<SharedData*, std::string> reverse_cache;

    struct CacheCheck {
      ~CacheCheck() {
        if (!cache.empty()) {
          std::cerr << "Not all open objects were properly close before the rmf"
                    << " module was unloaded. This is a bad thing."
                    << std::endl;
          for (map<std::string, SharedData *>::const_iterator it=cache.begin();
               it != cache.end(); ++it) {
            std::cerr << it->first << std::endl;
          }
        }
      }
    };
    CacheCheck checker;
    }
    SharedData::SharedData(std::string path): valid_(11111), cur_frame_(0),
                                              path_(path){
    };
    SharedData::~SharedData() {
      valid_=-66666;
      // check for an exception in the constructor
      if (reverse_cache.find(this) != reverse_cache.end()) {
        std::string name= reverse_cache.find(this)->second;
        cache.erase(name);
        reverse_cache.erase(this);
      }
    }

    void SharedData::audit_key_name(std::string name) const {
      if (name.empty()) {
        RMF_THROW("Empty key name", UsageException);
      }
      static const char *illegal="\\:=()[]{}\"'";
      const char *cur=illegal;
      while (*cur != '\0') {
        if (name.find(*cur) != std::string::npos) {
          RMF_THROW(get_error_message("Key names can't contain ",
                                          *cur), UsageException);
        }
        ++cur;
      }
      if (name.find("  ") != std::string::npos) {
        RMF_THROW("Key names can't contain two consecutive spaces",
                      UsageException);
      }
    }

    void SharedData::audit_node_name(std::string name) const {
      if (name.empty()) {
        RMF_THROW("Empty key name", UsageException);
      }
      static const char *illegal="\"";
      const char *cur=illegal;
      while (*cur != '\0') {
        if (name.find(*cur) != std::string::npos) {
          RMF_THROW(get_error_message("Node names names can't contain \"",
                                          *cur,
                                          "\", but \"", name, "\" does."),
                        UsageException);
        }
        ++cur;
      }
    }

    std::string SharedData::get_file_name() const {
#if BOOST_VERSION >= 104600
      return boost::filesystem::path(path_).filename().string();
#else
      return boost::filesystem::path(path_).filename();
#endif
    }

  void SharedData::validate(std::ostream &out) const {
    Creators cs= get_validators();
    for (unsigned int i=0; i< cs.size(); ++i) {
      boost::scoped_ptr<Validator>
          ptr(cs[i]->create(FileHandle(const_cast<SharedData*>(this))));
      ptr->write_errors(out);
    }
  }

    // throws RMF::IOException if couldn't create file or unsupported file
    // format
    SharedData* create_shared_data(std::string path, bool create) {
      SharedData *ret;
      if (cache.find(path) != cache.end()) {
        return cache.find(path)->second;
      }
      if (boost::algorithm::ends_with(path, ".rmf")) {
        ret= new HDF5SharedData(path, create, false);
      } else {
        RMF_THROW("Don't know how to open file", IOException);
      }
      cache[path]=ret;
      reverse_cache[ret]=path;
      return ret;
    }

    SharedData* create_read_only_shared_data(std::string path) {
      SharedData *ret;
      if (cache.find(path) != cache.end()) {
        return cache.find(path)->second;
      }
      if (boost::algorithm::ends_with(path, ".rmf")) {
        ret= new HDF5SharedData(path, false, true);
      } else {
        RMF_THROW("Don't know how to open file", IOException);
      }
      cache[path]=ret;
      reverse_cache[ret]=path;
      return ret;
    }

  } // namespace internal
} /* namespace RMF */