import IMP
import IMP.test
import IMP.core
import IMP.atom
try:
    import sympy
    from sympy.physics.units import *
    use_sympy=True
except:
    use_sympy=False

import math
nreps=10000
if IMP.build=="debug":
    nreps=nreps/100
nsteps=500
timestep=1000
D=.0002
k=.01
f=.1
kt_silly=IMP.atom.get_kb_t(273.)

if use_sympy:
    timestep_u=timestep*femto*second
    t_u= timestep_u*nsteps
    angstrom= sympy.Rational(1,10)*nano*meter
    D_u= .0002*angstrom*angstrom/(femto*second)
    k_u=  IMP.atom.get_spring_constant_in_femto_newtons_per_angstrom(k)*femto*newton/angstrom
    f_u= IMP.atom.get_force_in_femto_newtons(f)*femto*newton
    kT_u= IMP.atom.get_energy_in_femto_joules(kt_silly)*femto*joule
else:
    angstrom=1.0

class Tests(IMP.test.TestCase):
    def _setup(self):
        m= IMP.Model()
        IMP.set_log_level(IMP.SILENT)
        p= IMP.Particle(m)
        xyzr= IMP.core.XYZR.setup_particle(p)
        xyzr.set_coordinates(IMP.algebra.Vector3D(0,0,0))
        xyzr.set_radius(1)
        d=IMP.atom.Diffusion.setup_particle(p)
        d.set_d(D)
        IMP.set_check_level(IMP.NONE)
        bd= IMP.atom.BrownianDynamics(m)
        bd.set_maximum_time_step(float(timestep))
        xyzr.set_coordinates_are_optimized(True)
        return (m, xyzr, d, bd)
    def _measure(self, m, xyzr, bd):
        ub= IMP.algebra.Vector3D(40,40,40)
        # was .1
        h = IMP.statistics.Histogram3D(.3, IMP.algebra.BoundingBox3D(-ub, ub))
        #IMP.benchmark.set_is_profiling(True)
        for i in range(0,nreps):
            xyzr.set_coordinates(IMP.algebra.Vector3D(0,0,0))
            bd.optimize(nsteps)
            h.add(xyzr.get_coordinates())
            if i%1000==0:
                print i, xyzr.get_coordinates()
        print "computing"
        mn= h.get_mean()
        print mn
        std= h.get_standard_deviation(mn)
        print mn, std
        #IMP.benchmark.set_is_profiling(False)
        return (mn, std, nreps)
    def _get_sigma_error(self, sigma, n):
        return 2.0*sigma**4/n
    #def _measure_x(self, m, xyzr, bd):
    #    ub= IMP.algebra.Vector1D(50)
    #    h = IMP.statistics.Histogram1D(.1, IMP.algebra.BoundingBox1D(-ub, ub))
    #   for i in range(0,nreps):
    #        xyzr.set_coordinates(IMP.algebra.Vector3D(0,0,0))
    #        bd.optimize(nsteps)
    #        h.add(IMP.algebra.Vector1D(xyzr.get_coordinates()[0]))
    #        if i%1000==0:
    #            print i, xyzr.get_coordinates()
    #    mn= h.get_mean()
    #    std= h.get_standard_deviation(mn)
    ##    print mn, std
    ##   return mn, std
    def _get_sigma_free(self):
        sigma= (6.0*t_u*D_u)**sympy.Rational(1,2)
        return sigma
    def _get_sigma_1_free(self):
        if use_sympy:
            sigma= (2.0*t_u*D_u)**sympy.Rational(1,2)
            return sigma
        else:
            return 14
    def _get_sigma_harmonic(self):
        if use_sympy:
            delta2= (kT_u/k_u)
            delta= delta2**sympy.Rational(1,2)
            print "delta", delta
            taut= 2*delta2/D_u
            print "taut", taut.evalf()
            tau= t_u/taut
            print "tau", tau.evalf()
            scale= 2*delta2
            print "scale", scale.evalf()
            sigma2= scale*(1-sympy.E**(-4*tau))
            print "sigma2", sigma2.evalf()
            sigma= sigma2**sympy.Rational(1,2)
            print "sigma", sigma.evalf()
            return sigma
        else:
            return 10
    def _get_sigma_limit_harmonic(self):
        if use_sympy:
            sigmass= 2*kT_u/k_u
            print "sigma steady state 2", sigmass.evalf()
            return sigmass**sympy.Rational(1,2)
        else:
            return 0
    def _check(self, (mn, std), (calc_mn, calc_std), n):
        print [(x.evalf()/angstrom).evalf() for x in calc_std]
        print "mean", mn, [float((x/angstrom).evalf()) for x in calc_mn]
        print "std", std, [float((x/angstrom).evalf()) for x in calc_std]
        for i in range(0,3):
            self.assertAlmostEqual(mn[i], float((calc_mn[i]/angstrom).evalf()), delta=std[i]*2/n**.5)
            self.assertAlmostEqual(std[i], float((calc_std[i]/angstrom).evalf()), delta=2.0*self._get_sigma_error(std[i], n))
    def test_free(self):
        """Test brownian free diffusion"""
        #self.skipTest("too expensive")
        (m, xyzr, d, bd)= self._setup()
        sigma= self._get_sigma_1_free()
        print "free sigma is", sigma
        (mn, std,nreps)= self._measure(m, xyzr, bd)
        print mn, std
        self._check((mn, std), ([0*angstrom, 0*angstrom, 0*angstrom], [sigma, sigma, sigma]), nreps)
    def test_linear(self):
        """Test brownian linear diffusion"""
        #self.skipTest("too expensive")
        # from of course notes http://www.ks.uiuc.edu/~kosztin/
        (m, xyzr, d, bd)= self._setup()
        #print D_u, f_u, kT_u
        #print D_u*f_u, f_u/kT_u, 1/kT_u
        if use_sympy:
            mean= -D_u*f_u*t_u/kT_u
        else:
            mean=-18
        print "mean", mean
        h = IMP.core.Linear(0, f)
        dss= IMP.core.AttributeSingletonScore(h, IMP.core.XYZ.get_xyz_keys()[0])
        r= IMP.core.SingletonRestraint(dss, xyzr)
        m.add_restraint(r)
        sigma= self._get_sigma_1_free()
        mn, std, nreps= self._measure(m, xyzr, bd)
        self._check((mn, std), ([mean, 0*angstrom, 0*angstrom], [sigma, sigma, sigma]), nreps)
    def test_harmonic(self):
        """Test a brownian harmonic"""
        #self.skipTest("too expensive")
        # taken from chapter 13 of course notes http://www.ks.uiuc.edu/~kosztin/
        (m, xyzr, d, bd)= self._setup()
        sigma=self._get_sigma_harmonic()
        print "harmonic sigma", sigma
        sigmaf= self._get_sigma_1_free()
        sigmass= self._get_sigma_limit_harmonic()

        h = IMP.core.Harmonic(0, k)
        dss= IMP.core.AttributeSingletonScore(h, IMP.core.XYZ.get_xyz_keys()[0])
        r= IMP.core.SingletonRestraint(dss, xyzr)
        m.add_restraint(r)
        mn, std, nreps= self._measure(m, xyzr, bd)
        self._check((mn, std), ([0*angstrom, 0*angstrom, 0*angstrom], [sigma, sigmaf, sigmaf]), nreps)
if __name__ == '__main__':
    IMP.test.main()
