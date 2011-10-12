/**
 *  \file GaussianProcessInterpolationRestraint.cpp  
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#include <IMP/isd/GaussianProcessInterpolationRestraint.h>
#include <IMP/macros.h>
#include <IMP/Object.h>
#include <IMP/constants.h>
#include <math.h>
#include <IMP/algebra/internal/tnt_array2d.h>
#include <IMP/algebra/internal/jama_lu.h>
#include <boost/scoped_ptr.hpp>

IMPISD_BEGIN_NAMESPACE

GaussianProcessInterpolationRestraint::GaussianProcessInterpolationRestraint(
        GaussianProcessInterpolation *gpi) : gpi_(gpi)
{
    //number of observation points
    IMP_LOG(TERSE, "GPIR: init" << std::endl);
    M_ = gpi_->n_obs_.size();
    //number of repetitions
    int N=gpi_->n_obs_[0];
    //check if the number of repetitions is the same for every point
    IMP_IF_CHECK(USAGE) {
        for (unsigned i=1; i<M_; i++)
            IMP_USAGE_CHECK(N == gpi_->n_obs_[i], 
                "must have the same number of repetitions for each point!");
    }
    // build multivariate normal with 
    // mean : prior mean
    // covariance : prior covariance
    // observed at : the original observations
    IMP_LOG(TERSE, "GPIR: multivariate normal()" << std::endl);
    //args are: sample mean, jacobian, true mean, 
    // nobs, sample variance, true variance
    mvn_ = new MultivariateFNormalSufficient(gpi_->get_I(), 1.0, gpi_->get_m(),
            N, gpi_->get_S(), gpi_->get_W());
    IMP_LOG(TERSE, "GPIR: done init" << std::endl);
}

void GaussianProcessInterpolationRestraint::update_mean_and_covariance()
{
    IMP_LOG(TERSE, "GPIR: update_mean_and_covariance" << std::endl);
    gpi_->update_flags_covariance();
    gpi_->update_flags_mean();
    if (!(gpi_->flag_m_gpir_))  // gpi says that our m is not up to date
    {
        mvn_->set_FM(gpi_->get_m());
        gpi_->flag_m_gpir_ = true;
        IMP_LOG(TERSE, " updated mean");
    }
    if (!(gpi_->flag_W_gpir_)) 
    {
        mvn_->set_Sigma(gpi_->get_W());
        gpi_->flag_W_gpir_ = true;
        IMP_LOG(TERSE, " updated covariance");
    }
    IMP_LOG(TERSE, std::endl);
}

double GaussianProcessInterpolationRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
    //check if the functions have changed
    const_cast<GaussianProcessInterpolationRestraint*>(this)->
        update_mean_and_covariance();
    
    double ene = mvn_->evaluate();

    if (accum)
    {
        VectorXd dmv = mvn_->evaluate_derivative_FM();
        //derivatives for mean particles
        MatrixXd dmean = gpi_->mean_function_->get_derivative_matrix(gpi_->x_);
        VectorXd meanprod = dmv.transpose()*dmean;
        unsigned npart=meanprod.cols(); //should be 2 for Linear1DFunction
        for (unsigned i=0; i<meanprod.cols(); i++)
            gpi_->mean_function_->add_to_particle_derivative(i, meanprod(i), 
                    accum);
        //derivatives for covariance particles
        MatrixXd dmvS = mvn_->evaluate_derivative_Sigma();
        npart = gpi_->covariance_function_->get_number_of_particles();
        for (unsigned i=0; i<npart; i++)
        {
            MatrixXd dcov = 
                gpi_->covariance_function_->get_derivative_matrix(i, gpi_->x_);
            double val = (dmvS.transpose()*dcov).trace();
            gpi_->covariance_function_->add_to_particle_derivative(i, val,
                    accum);
        }
    }
    return ene;
}

ParticlesTemp GaussianProcessInterpolationRestraint::get_input_particles() const
{
    ParticlesTemp ret;
    ParticlesTemp ret1 = gpi_->mean_function_->get_input_particles();
    ret.insert(ret.end(),ret1.begin(),ret1.end());
    ParticlesTemp ret2 = gpi_->covariance_function_->get_input_particles();
    ret.insert(ret.end(),ret2.begin(),ret2.end());
    return ret;
}
    
ContainersTemp GaussianProcessInterpolationRestraint::get_input_containers() 
    const
{
    ContainersTemp ret;
    ContainersTemp ret1 = gpi_->mean_function_->get_input_containers();
    ret.insert(ret.end(),ret1.begin(),ret1.end());
    ContainersTemp ret2 = gpi_->covariance_function_->get_input_containers();
    ret.insert(ret.end(),ret2.begin(),ret2.end());
    return ret;
}
    
void GaussianProcessInterpolationRestraint::do_show(std::ostream& out) const
{
    out << "GaussianProcessInterpolationRestraint on " 
        << get_input_particles().size() << " particles" << std::endl;
}

IMPISD_END_NAMESPACE

