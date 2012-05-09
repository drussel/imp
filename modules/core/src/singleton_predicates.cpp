/**
 *  \file SingletonPredicate.cpp  \brief Define SingletonPredicate
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/core/singleton_predicates.h>
#include <IMP/base/random.h>

IMPCORE_BEGIN_NAMESPACE

ConstantSingletonPredicate::ConstantSingletonPredicate(int v,
                                                 std::string name):
    SingletonPredicate(name), v_(v){}

UnorderedTypeSingletonPredicate::UnorderedTypeSingletonPredicate(
                                                 std::string name):
    SingletonPredicate(name){}

OrderedTypeSingletonPredicate::OrderedTypeSingletonPredicate(
                                                 std::string name):
    SingletonPredicate(name){}


AllSameSingletonPredicate::AllSameSingletonPredicate(
    std::string name):
    SingletonPredicate(name){}

CoinFlipSingletonPredicate::CoinFlipSingletonPredicate( double p,
    std::string name):
  SingletonPredicate(name), p_(p){}


IMPCORE_END_NAMESPACE
