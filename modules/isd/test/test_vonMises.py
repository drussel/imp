#!/usr/bin/env python

#general imports
from numpy import *
from scipy.special import i0,i1
from random import uniform


#imp general
import IMP

#our project
from IMP.isd import vonMises

#unit testing framework
import IMP.test

class TestvonMises(IMP.test.TestCase):

    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)

    def testEvaluate(self):
        "tests vonMises.evaluate"
        for i in xrange(100):
            randno = [uniform(-4*pi,4*pi), uniform(-pi,pi),
                    uniform(0.1,100)]
            fn=vonMises(*randno)
            self.assertAlmostEqual(fn.evaluate(),
                    log(2*pi*i0(randno[2])) - randno[2]*cos(randno[0]-randno[1]),
                    delta=0.001)

    def testEvaluateDX(self):
        "tests vonMises.evaluate_derivative_x"
        for i in xrange(100):
            randno = [uniform(-4*pi,4*pi), uniform(-pi,pi),
                    uniform(0.1,100)]
            fn=vonMises(*randno)
            self.assertAlmostEqual(fn.evaluate_derivative_x(),
                    randno[2]*sin(randno[0]-randno[1]),
                    delta=0.001)

    def testEvaluateDMu(self):
        "tests vonMises.evaluate_derivative_mu"
        for i in xrange(100):
            randno = [uniform(-4*pi,4*pi), uniform(-pi,pi),
                    uniform(0.1,100)]
            fn=vonMises(*randno)
            self.assertAlmostEqual(fn.evaluate_derivative_mu(),
                    -randno[2]*sin(randno[0]-randno[1]),
                    delta=0.001)

    def testEvaluateDKappa(self):
        "tests vonMises.evaluate_derivative_kappa"
        for i in xrange(100):
            randno = [uniform(-4*pi,4*pi), uniform(-pi,pi),
                    uniform(0.1,100)]
            fn=vonMises(*randno)
            self.assertAlmostEqual(fn.evaluate_derivative_kappa(),
                    i1(randno[2])/i0(randno[2]) - cos(randno[0]-randno[1]),
                    delta=0.001)

    def testDensity(self):
        "tests vonMises.density"
        for i in xrange(100):
            randno = [uniform(-4*pi,4*pi), uniform(-pi,pi),
                    uniform(0.1,100)]
            fn=vonMises(*randno)
            self.assertAlmostEqual(fn.density(),
                    exp(randno[2]*cos(randno[0]-randno[1]))/(2*pi*i0(randno[2])),
                    delta=0.001)

if __name__ == '__main__':
    IMP.test.main()
