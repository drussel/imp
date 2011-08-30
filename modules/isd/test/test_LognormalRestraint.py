#!/usr/bin/env python

#general imports
from numpy import *


#imp general
import IMP

#our project
from IMP.isd import Nuisance,Scale,LognormalRestraint

#unit testing framework
import IMP.test

class TestLognormalRestraintSimple3(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = Scale.setup_particle(IMP.Particle(self.m), 2.0)
        self.mu = Nuisance.setup_particle(IMP.Particle(self.m), 1.0)
        self.x = Nuisance.setup_particle(IMP.Particle(self.m), 2.0)
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_mu(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(mu/x)/(mu*sigma**2)

    def deriv_x(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(x/mu)/(x*sigma**2) + 1./x

    def deriv_sigma(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return -log(x/mu)**2/sigma**3 + 1./sigma

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdx(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.x).get_nuisance_derivative(),
                    self.deriv_x(*self.all))

    def testdmu(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.mu).get_nuisance_derivative(),
                    self.deriv_mu(*self.all))

    def testdsigma(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(
                    Nuisance(self.sigma).get_nuisance_derivative(),
                    self.deriv_sigma(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

    def testFail(self):
        dummy=IMP.Particle(self.m)
        self.assertRaises(IMP.UsageException, LognormalRestraint, dummy, self.all[1], self.all[2])
        self.assertRaises(IMP.UsageException, LognormalRestraint, self.all[0], dummy, self.all[2])
        self.assertRaises(IMP.UsageException, LognormalRestraint, self.all[0], self.all[1], dummy)

class TestLognormalRestraintSimple21(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = 2.0
        self.mu = Nuisance.setup_particle(IMP.Particle(self.m), 1.0)
        self.x = Nuisance.setup_particle(IMP.Particle(self.m), 2.0)
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_mu(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(mu/x)/(mu*sigma**2)

    def deriv_x(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(x/mu)/(x*sigma**2) + 1./x

    def deriv_sigma(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return -log(x/mu)**2/sigma**3 + 1./sigma

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdx(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.x).get_nuisance_derivative(),
                    self.deriv_x(*self.all))

    def testdmu(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.mu).get_nuisance_derivative(),
                    self.deriv_mu(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

class TestLognormalRestraintSimple22(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = Scale.setup_particle(IMP.Particle(self.m), 2.0)
        self.mu = 1.0
        self.x = Nuisance.setup_particle(IMP.Particle(self.m), 2.0)
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_mu(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(mu/x)/(mu*sigma**2)

    def deriv_x(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(x/mu)/(x*sigma**2) + 1./x

    def deriv_sigma(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return -log(x/mu)**2/sigma**3 + 1./sigma

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdx(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.x).get_nuisance_derivative(),
                    self.deriv_x(*self.all))

    def testdsigma(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(
                    Nuisance(self.sigma).get_nuisance_derivative(),
                    self.deriv_sigma(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

class TestLognormalRestraintSimple23(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = Scale.setup_particle(IMP.Particle(self.m), 2.0)
        self.mu = Nuisance.setup_particle(IMP.Particle(self.m), 1.0)
        self.x = 2.0
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_mu(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(mu/x)/(mu*sigma**2)

    def deriv_x(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(x/mu)/(x*sigma**2) + 1./x

    def deriv_sigma(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return -log(x/mu)**2/sigma**3 + 1./sigma

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdmu(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.mu).get_nuisance_derivative(),
                    self.deriv_mu(*self.all))

    def testdsigma(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(
                    Nuisance(self.sigma).get_nuisance_derivative(),
                    self.deriv_sigma(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

class TestLognormalRestraintSimple11(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = Scale.setup_particle(IMP.Particle(self.m), 2.0)
        self.mu = 1.0
        self.x = 2.0
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_mu(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(mu/x)/(mu*sigma**2)

    def deriv_x(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(x/mu)/(x*sigma**2) + 1./x

    def deriv_sigma(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return -log(x/mu)**2/sigma**3 + 1./sigma

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdsigma(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(
                    Nuisance(self.sigma).get_nuisance_derivative(),
                    self.deriv_sigma(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

class TestLognormalRestraintSimple12(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = 2.0
        self.mu = Nuisance.setup_particle(IMP.Particle(self.m), 1.0)
        self.x = 2.0
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_mu(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(mu/x)/(mu*sigma**2)

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdmu(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.mu).get_nuisance_derivative(),
                    self.deriv_mu(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

class TestLognormalRestraintSimple13(IMP.test.TestCase):
    "simple test cases to check if LognormalRestraint works"
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        #IMP.set_log_level(IMP.MEMORY)
        IMP.set_log_level(0)
        self.m = IMP.Model()
        self.sigma = 2.0
        self.mu = 1.0
        self.x = Nuisance.setup_particle(IMP.Particle(self.m), 2.0)
        self.locations=[self.x, self.mu]
        self.all = self.locations+[self.sigma]
        self.DA = IMP.DerivativeAccumulator()

    def get_value(self, p):
        try:
            v = Nuisance(p).get_nuisance()
        except:
            v = p
        return v

    def change_value(self, p, min=0.1, max=100):
        try:
            n = Nuisance(p)
        except:
            return
        n.set_nuisance(random.uniform(min,max))

    def normal_p(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 1/(sqrt(2*pi)*sigma*x) * exp(-1/(2*sigma**2)*log(x/mu)**2)

    def normal_e(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return 0.5*log(2*pi) + log(sigma*x) + 1/(2*sigma**2)*log(x/mu)**2

    def deriv_x(self, x, mu, sigma):
        (x,mu,sigma) = map(self.get_value, [x,mu,sigma])
        return log(x/mu)/(x*sigma**2) + 1./x

    def testE(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            e=self.m.evaluate(False)
            self.assertAlmostEqual(e, self.normal_e(*self.all))

    def testdx(self):
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.m.evaluate(True)
            self.assertAlmostEqual(Nuisance(self.x).get_nuisance_derivative(),
                    self.deriv_x(*self.all))

    def testSanityPE(self):
        "test if prob is exp(-score)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            self.assertAlmostEqual(gr.get_probability(),
                    exp(-self.m.evaluate(False)),delta=0.001)

    def testSanityEP(self):
        "test if score is -log(prob)"
        gr=LognormalRestraint(*self.all)
        self.m.add_restraint(gr)
        for i in xrange(100):
            map(self.change_value, self.all)
            expected = gr.get_probability()
            if expected == 0:
                continue
            self.assertAlmostEqual(-log(expected),
                    self.m.evaluate(False),delta=0.001)

if __name__ == '__main__':
    IMP.test.main()


