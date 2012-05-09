/**
 *  \file TripletPredicate.cpp  \brief Define TripletPredicate
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/core/triplet_predicates.h>
#include <IMP/base/random.h>

IMPCORE_BEGIN_NAMESPACE

ConstantTripletPredicate::ConstantTripletPredicate(int v,
                                                 std::string name):
    TripletPredicate(name), v_(v){}

UnorderedTypeTripletPredicate::UnorderedTypeTripletPredicate(
                                                 std::string name):
    TripletPredicate(name){}

OrderedTypeTripletPredicate::OrderedTypeTripletPredicate(
                                                 std::string name):
    TripletPredicate(name){}


AllSameTripletPredicate::AllSameTripletPredicate(
    std::string name):
    TripletPredicate(name){}

CoinFlipTripletPredicate::CoinFlipTripletPredicate( double p,
    std::string name):
  TripletPredicate(name), p_(p){}


IMPCORE_END_NAMESPACE
