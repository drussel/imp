#!/usr/bin/env python

#general imports
from numpy import *
from random import uniform


#imp general
import IMP

#our project
from IMP.isd import *

#unit testing framework
import IMP.test

class MockFunc:
    def __init__(self, setval, evaluate, evalargs=(1,), update=None):
        self.__set = setval
        self.__eval = evaluate
        self.__update = update
        self.__evalargs = evalargs

    def set_evalargs(self, evalargs):
        self.__evalargs = evalargs

    def __call__(self, value):
        self.__set(value)
        if self.__update:
            self.__update()
        return self.__eval(*self.__evalargs)

class TestGaussianProcessInterpolationNumericallyNoSigma(IMP.test.TestCase):
    """test of the GPI with more data points, using numerical derivative
    estimation. Sigma is not optimized (uses different internal loop
    structures).
    """

    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.TERSE)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        data=open(self.get_input_file_name('lyzexp_gpir.dat')).readlines()
        data=[map(float,d.split()) for d in data]
        self.q=[[i[0]] for i in data]
        self.I=[i[1] for i in data]
        self.err=[i[2] for i in data]
        self.N=10
        self.G = Scale.setup_particle(IMP.Particle(self.m), 3.0)
        self.G.set_nuisance_is_optimized(True)
        self.Rg = Scale.setup_particle(IMP.Particle(self.m),  10.0)
        self.Rg.set_nuisance_is_optimized(True)
        #put d=15 so we don't use the porod region
        self.d = Scale.setup_particle(IMP.Particle(self.m),  15.0)
        self.d.set_nuisance_is_optimized(False)
        self.s = Scale.setup_particle(IMP.Particle(self.m),  0.0)
        self.s.set_nuisance_is_optimized(False)
        self.A = Scale.setup_particle(IMP.Particle(self.m),  0.0)
        self.A.set_nuisance_is_optimized(False)
        self.mean = GeneralizedGuinierPorodFunction(
                self.G,self.Rg,self.d,self.s, self.A)
        self.tau = Switching.setup_particle(IMP.Particle(self.m), 1.0)
        self.tau.set_nuisance_is_optimized(True)
        self.lam = Scale.setup_particle(IMP.Particle(self.m), 1.)
        self.lam.set_nuisance_is_optimized(True)
        self.sig = Scale.setup_particle(IMP.Particle(self.m), 1.0)
        self.sig.set_nuisance_is_optimized(False)
        self.cov = Covariance1DFunction(self.tau, self.lam, 2.0)
        self.gpi = IMP.isd.GaussianProcessInterpolation(self.q, self.I,
                self.err, self.N, self.mean, self.cov, self.sig)
        self.particles=[self.G,self.Rg,self.d,self.s,self.sig,self.tau,self.lam]

    def testCovDerivNumericG(self):
        """
        test the derivatives of the gpi numerically for G
        """
        pnum=0
        values=linspace(1,10)
        pos=0.1
        particle=self.particles[pnum]
        PFunc = MockFunc(particle.set_nuisance,
                self.gpi.get_posterior_covariance,
                ([pos],[pos]))
        for val in values:
            particle.set_nuisance(val)
            ene=self.gpi.get_posterior_covariance([pos],[pos])
            observed = self.gpi.get_posterior_covariance_derivative([pos],
                    False)[pnum]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovDerivNumericRg(self):
        """
        test the derivatives of the gpi numerically for Rg
        """
        pnum=1
        values=linspace(1,10)
        pos=0.1
        particle=self.particles[pnum]
        PFunc = MockFunc(particle.set_nuisance,
                self.gpi.get_posterior_covariance,
                ([pos],[pos]))
        for val in values:
            particle.set_nuisance(val)
            ene=self.gpi.get_posterior_covariance([pos],[pos])
            observed = self.gpi.get_posterior_covariance_derivative([pos],
                    False)[pnum]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovDerivNumericTau(self):
        """
        test the derivatives of the gpi numerically for Tau
        """
        pnum=5
        values=linspace(1,10)
        pos=0.1
        particle=self.particles[pnum]
        PFunc = MockFunc(particle.set_nuisance,
                self.gpi.get_posterior_covariance,
                ([pos],[pos]))
        for val in values:
            particle.set_nuisance(val)
            ene=self.gpi.get_posterior_covariance([pos],[pos])
            observed = self.gpi.get_posterior_covariance_derivative([pos],
                    False)[pnum-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovDerivNumericLambda(self):
        """
        test the derivatives of the gpi numerically for Lambda
        """
        pnum=6
        values=linspace(.1,1)
        pos=0.1
        particle=self.particles[pnum]
        PFunc = MockFunc(particle.set_nuisance,
                self.gpi.get_posterior_covariance,
                ([pos],[pos]))
        for val in values:
            particle.set_nuisance(val)
            ene=self.gpi.get_posterior_covariance([pos],[pos])
            observed = self.gpi.get_posterior_covariance_derivative([pos],
                    False)[pnum-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)


    def testCovHessianNumericGG(self):
        """
        test the hessian of the gpi numerically for G and G
        """
        pn1=0
        pn2=0
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovHessianNumericGRg(self):
        """
        test the hessian of the gpi numerically for G and Rg
        """
        pn1=0
        pn2=1
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovHessianNumericGTau(self):
        """
        test the hessian of the gpi numerically for G and Tau
        """
        pn1=0
        pn2=5
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovHessianNumericGLambda(self):
        """
        test the hessian of the gpi numerically for G and Lambda
        """
        pn1=0
        pn2=6
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)


    def testCovHessianNumericRgRg(self):
        """
        test the hessian of the gpi numerically for Rg and Rg
        """
        pn1=1
        pn2=1
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovHessianNumericRgTau(self):
        """
        test the hessian of the gpi numerically for Rg and Tau
        """
        pn1=1
        pn2=5
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)

    def testCovHessianNumericRgLambda(self):
        """
        test the hessian of the gpi numerically for Rg and Lambda
        """
        pn1=1
        pn2=6
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(expected,observed,delta=1e-3)


    def testCovHessianNumericTauTau(self):
        """
        test the hessian of the gpi numerically for Tau and Tau
        """
        pn1=5
        pn2=5
        values=linspace(.1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1-3][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(observed,expected,delta=1e-3)

    def testCovHessianNumericTauLambda(self):
        """
        test the hessian of the gpi numerically for Tau and Lambda
        """
        pn1=6
        pn2=5
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1-3][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(observed/expected,1,delta=1e-2)


    def testCovHessianNumericLambdaLambda(self):
        """
        test the hessian of the gpi numerically for Lambda and Lambda
        """
        pn1=6
        pn2=6
        values=linspace(1,10)
        pos=0.1
        p1=self.particles[pn1]
        p2=self.particles[pn2]
        PFunc = MockFunc(p1.set_nuisance,
                lambda a:self.gpi.get_posterior_covariance_derivative(a,
                    False)[pn2-3], ([pos],))
        for val in values:
            p1.set_nuisance(val)
            observed = self.gpi.get_posterior_covariance_hessian([pos],
                    False)[pn1-3][pn2-3]
            expected = IMP.test.numerical_derivative(PFunc, val, 0.01)
            self.assertAlmostEqual(observed/expected,1,delta=1e-2)


if __name__ == '__main__':
    IMP.test.main()
