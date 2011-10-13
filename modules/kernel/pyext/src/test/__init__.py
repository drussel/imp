"""@namespace IMP::test
   @brief Methods and classes for testing the IMP kernel and modules.
   @ingroup python
"""

import re, math
import sys
import os
import random
import IMP
import time
import types
import shutil
import _compat_python
import _compat_python.unittest2
import datetime

# Fall back to the sets.Set class on older Pythons that don't have
# the 'set' builtin type.
try:
    set = set
except NameError:
    import sets
    set = sets.Set

# Load a new enough unittest package (should have the 'skip' decorator)
# - On Python 2.7 or 3.2, the standard 'unittest' package will work.
# - On older Pythons, use the 'unittest2' package if available, otherwise use
#   our bundled version of this package.
def __load_unittest_package():
    errors = []
    for modname, fromlist in (('unittest', []),
                              ('unittest2', []),
                              ):
        try:
            u = __import__(modname, {}, {}, fromlist)
            if hasattr(u, 'skip'):
                return u
            else:
                errors.append("'%s' does not have the 'skip' decorator" \
                              % modname)
        except ImportError, e:
            errors.append(str(e))
        #u = __import__("_compat_python.unittest2
        return _compat_python.unittest2
    raise ImportError("IMP.test requires a newer version of Python's unittest "
                      "package than is available. Either upgrade to a new "
                      "enough Python (at least 2.7 or 3.2) or install the "
                      "unittest2 package. Encountered errors: %s" \
                      % "; ".join(errors))
unittest = __load_unittest_package()

# Expose some unittest decorators for convenience
expectedFailure = unittest.expectedFailure
skip = unittest.skip
skipIf = unittest.skipIf
skipUnless = unittest.skipUnless


def numerical_derivative(func, val, step):
    """Calculate the derivative of the single-value function `func` at
       point `val`. The derivative is calculated using simple finite
       differences starting with the given `step`; Richardson extrapolation
       is then used to extrapolate the derivative at step=0."""
    maxsteps = 50
    con = 1.4
    safe = 2.0
    err = 1.0e30
    f1 = func(val + step)
    f2 = func(val - step)
    # create first element in triangular matrix d of derivatives
    d = [[(f1 - f2) / (2.0 * step)]]
    retval = None
    for i in range(1, maxsteps):
        d.append([0.] * (i + 1))
        step /= con
        f1 = func(val + step)
        f2 = func(val - step)
        d[i][0] = (f1 - f2) / (2.0 * step)
        fac = con * con
        for j in range(1, i + 1):
            d[i][j] = (d[i][j-1] * fac - d[i-1][j-1]) / (fac - 1.)
            fac *= con * con
            errt = max(abs(d[i][j] - d[i][j-1]),
                       abs(d[i][j] - d[i-1][j-1]))
            if errt <= err:
                err = errt
                retval = d[i][j]
        if abs(d[i][i] - d[i-1][i-1]) >= safe * err:
            break
    if retval is None:
        raise ValueError("Cannot calculate numerical derivative")
    return retval


def xyz_numerical_derivatives(model, xyz, step):
    """Calculate the x,y and z derivatives of `model`'s scoring function
       on the `xyz` particle. The derivatives are approximated numerically
       using the numerical_derivatives() function."""
    class _XYZDerivativeFunc(object):
        def __init__(self, xyz, basis_vector):
            self._xyz = xyz
            self._model = xyz.get_particle().get_model()
            self._basis_vector = basis_vector
            self._starting_coordinates = xyz.get_coordinates()

        def __call__(self, val):
            self._xyz.set_coordinates(self._starting_coordinates + \
                                      self._basis_vector * val)
            return self._model.evaluate(False)

    return tuple([IMP.test.numerical_derivative(_XYZDerivativeFunc(xyz,
                                          IMP.algebra.Vector3D(*x)), 0, 0.01) \
                  for x in ((1,0,0), (0,1,0), (0,0,1))])


class TestCase(unittest.TestCase):
    """Super class for IMP test cases"""

    def setUp(self):
        self.__check_level = IMP.get_check_level()
        # Turn on expensive runtime checks while running the test suite:
        IMP.set_check_level(IMP.USAGE_AND_INTERNAL)
        # python ints are bigger than C++ ones, so we need to make sure it fits
        # otherwise python throws fits
        IMP.random_number_generator.seed(hash(time.time())%2**30)
        self.start_time=datetime.datetime.now()

    def tearDown(self):
        # Restore original check level
        IMP.set_check_level(self.__check_level)
        delta= datetime.datetime.now()-self.start_time
        try:
            pv= delta.total_second()
        except:
            pv=(float(delta.microseconds) + (delta.seconds + delta.days * 24 * 3600) * 10**6) / 10**6
        if pv > 1:
            print >> sys.stderr, " in ", pv,"s ... ",


    def get_input_file_name(self, filename):
        """Get the full name of an input file in the top-level
           test directory."""
        # If we ran from run-all-tests.py, it set an env variable for us with
        # the top-level test directory
        if 'TEST_DIRECTORY' in os.environ:
            top = os.environ['TEST_DIRECTORY']
            return os.path.join(top, 'input', filename)
        else:
            # Otherwise, search up from the test's directory until we find
            # the input directory
            testdir = os.path.dirname(os.path.abspath(sys.argv[0]))
            dirs = testdir.split(os.path.sep)
            for i in range(len(dirs), 0, -1):
                input = os.path.sep.join(dirs[:i] + ['input'])
                if os.path.isdir(input):
                    return os.path.join(input, filename)
        # If not found, default to the current working directory:
        ret= os.path.join('input', filename)
        if not open(ret, "r"):
            raise IOError("Test input file "+ret+" does not exist")

    def open_input_file(self, filename, mode='rb'):
        """Open and return an input file in the top-level test directory."""
        return open(self.get_input_file_name(filename), mode)

    def get_tmp_file_name(self, filename):
        """Get the full name of an output file in the build/tmp directory."""
        dirpath=os.environ['IMP_TMP_DIR']
        if not os.path.exists(dirpath):
            os.mkdir(dirpath)
        return os.path.join(dirpath, filename)

    def get_magnitude(self, vector):
        return sum([x*x for x in vector], 0)**.5

    def assertXYZDerivativesInTolerance(self, model, xyz, tolerance=0,
                                        percentage=0):
        """Assert that x,y,z analytical derivatives match numerical within
           a tolerance, or a percentage (of the analytical value), whichever
           is larger."""
        model.evaluate(True)
        derivs = xyz.get_derivatives()
        num_derivs = xyz_numerical_derivatives(model, xyz, 0.01)
        pct = percentage / 100.0
        self.assertAlmostEqual(self.get_magnitude(derivs-num_derivs),0,
                               delta=tolerance+percentage*self.get_magnitude(num_derivs),
                               msg="Don't match "+str(derivs) + str(num_derivs))
        self.assertAlmostEqual(derivs[0], num_derivs[0],
                               delta=max(tolerance, abs(derivs[0]) * pct))
        self.assertAlmostEqual(derivs[1], num_derivs[1],
                               delta=max(tolerance, abs(derivs[1]) * pct))
        self.assertAlmostEqual(derivs[2], num_derivs[2],
                               delta=max(tolerance, abs(derivs[2]) * pct))

    def create_point_particle(self, model, x, y, z):
        """Make a particle with optimizable x, y and z attributes, and
           add it to the model."""
        p = IMP.Particle(model)
        p.add_attribute(IMP.FloatKey("x"), x, True)
        p.add_attribute(IMP.FloatKey("y"), y, True)
        p.add_attribute(IMP.FloatKey("z"), z, True)
        return p

    def probabilistic_test(self, testcall, chance_of_failure):
        """Help handle a test which is expected to fail some fraction of
        the time. The test is run multiple times and an exception
        is thrown only if it fails too many times."""
        prob=chance_of_failure
        tries=1
        while prob > .001:
            tries=tries+1
            prob= prob*chance_of_failure
        for i in range(0, tries):
            try:
                eval(testcall)
            except:
                pass
            else:
                return
        eval(testcall)
        raise AssertError("Too many failures")

    def failure_probability(self, testcall):
        """Estimate how like a given block of code is to raise an
        AssertionError."""
        failures=0
        tries=0.0
        while failures < 10 and tries <1000:
            try:
                eval(testcall)
            except:
                failures=failures+1
            tries=tries+1
        return failures/tries

    def randomize_particles(self, particles, deviation):
        """Randomize the xyz coordinates of a list of particles"""
        # Note: cannot use XYZ here since that pulls in IMP.core
        xkey = IMP.FloatKey("x")
        ykey = IMP.FloatKey("y")
        zkey = IMP.FloatKey("z")
        for p in particles:
            p.set_value(xkey, random.uniform(-deviation, deviation))
            p.set_value(ykey, random.uniform(-deviation, deviation))
            p.set_value(zkey, random.uniform(-deviation, deviation))

    def particle_distance(self, p1, p2):
        """Return distance between two given particles"""
        xkey = IMP.FloatKey("x")
        ykey = IMP.FloatKey("y")
        zkey = IMP.FloatKey("z")
        dx = p1.get_value(xkey) - p2.get_value(xkey)
        dy = p1.get_value(ykey) - p2.get_value(ykey)
        dz = p1.get_value(zkey) - p2.get_value(zkey)
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def check_unary_function_deriv(self, func, lb, ub, step):
        """Check the unary function func's derivatives against numerical
           approximations between lb and ub"""
        for f in [lb + i * step for i in range(1, int((ub-lb)/step))]:
            (v,d)= func.evaluate_with_derivative(f)
            da = numerical_derivative(func.evaluate, f, step / 10.)
            self.assertAlmostEqual(d, da, delta=max(abs(.1 *d), 0.01))

    def check_unary_function_min(self, func, lb, ub, step, expected_fmin):
        """Make sure that the minimum of the unary function func over the
           range between lb and ub is at expected_fmin"""
        fmin, vmin = lb, func.evaluate(lb)
        for f in [lb + i * step for i in range(1, int((ub-lb)/step))]:
            v = func.evaluate(f)
            if v < vmin:
                fmin, vmin = f, v
        self.assertAlmostEqual(fmin, expected_fmin, delta=step)

    def create_particles_in_box(self, model, num=10,
                                lb= [0,0,0],
                                ub= [10,10,10]):
        """Create a bunch of particles in a box"""
        import IMP.algebra
        lbv=IMP.algebra.Vector3D(lb[0],lb[1],lb[2])
        ubv=IMP.algebra.Vector3D(ub[0],ub[1],ub[2])
        ps= []
        for i in range(0,num):
            v = IMP.algebra.get_random_vector_in(IMP.algebra.BoundingBox3D(lbv, ubv))
            p = self.create_point_particle(model, v[0], v[1], v[2])
            ps.append(p)
        return ps
    def _get_type(self, module, name):
        return eval('type('+module+"."+name+')')
    def assertValueObjects(self, module, exceptions):
        "Check that all the C++ classes in the module are values or objects."
        all= dir(module)
        bad=[]
        for name in all:
            if self._get_type(module.__name__, name)==types.TypeType and not name.startswith("_"):
                if name.find("SwigPyIterator") != -1:
                    continue
                # Exclude Python-only classes
                if not eval('hasattr(%s.%s, "__swig_destroy__")' \
                            % (module.__name__, name)):
                    continue
                if name in exceptions:
                    continue
                if name not in eval(module.__name__+"._value_types")\
                       and name not in eval(module.__name__+"._object_types")\
                       and name not in eval(module.__name__+"._raii_types")\
                       and name not in eval(module.__name__+"._plural_types"):
                    bad.append(name)
        message="All IMP classes should be labeled values or as objects to get memory management correct in python. The following are not:\n%s\nPlease add an IMP_SWIG_OBJECT or IMP_SWIG_VALUE call to the python wrapper, or if the class has a good reason to be neither, add the name to the value_object_exceptions list in the IMPModuleTest call." \
                          % (str(bad))
        self.assertEquals(len(bad), 0,
                          message)

    def _check_spelling(self, word, words):
        """Check that the word is spelled correctly"""
        if "words" not in dir(self):
            wordlist= open(IMP.get_data_path("linux.words"), "r").read().split("\n")
            # why is "all" missing on my mac?
            custom_words=["info", "prechange", "int", "ints", "optimizeds", "graphviz",
                          "voxel", "voxels", "endian", 'rna', 'dna',
                          "xyzr", "pdbs", "fft", "ccc"]
            self.words=set(wordlist+custom_words)
        if self.words:
            for i in "0123456789":
                if i in word:
                    return True
            if word in words:
                return True
            if word in self.words:
                return True
            else:
                return False
        else:
            return True
    def assertClassNames(self, module, exceptions, words):
        """Check that all the classes in the module follow the imp naming conventions."""
        all= dir(module)
        bad=[]
        cc=re.compile("([A-Z][a-z]*)")
        for name in all:
            if self._get_type(module.__name__, name)==types.TypeType and not name.startswith("_"):
                if name.find("SwigPyIterator") != -1:
                    continue
                for t in re.findall(cc, name):
                    if not self._check_spelling(t.lower(), words):
                        print "misspelled", t, "in", name
                        bad.append(name)

        self.assertEquals(len(bad), 0,
                          "All IMP classes should be properly spelled. The following are not: %s. Add words to the spelling_exceptions variable of the IMPModuleTest if needed." \
                          % (str(bad)))

        for name in all:
            if self._get_type(module.__name__, name)==types.TypeType and not name.startswith("_"):
                if name.find("SwigPyIterator") != -1:
                    continue
                if name.find('_') != -1:
                    bad.append(name)
                if name.lower== name:
                    bad.append(name)
                for t in re.findall(cc, name):
                    if not self._check_spelling(t.lower(), words):
                        print "misspelled", t, "in", name
                        bad.append(name)
        message="All IMP classes should have CamelCase names. The following do not: %s." \
                          % ("\n".join(bad))
        self.assertEquals(len(bad), 0,
                          message)


    def _check_function_name(self, prefix, name, verbs, all, exceptions, words,
                             misspelled):
        if prefix:
            fullname=prefix+"."+name
        else:
            fullname=name
        old_exceptions=['unprotected_evaluate', "unprotected_evaluate_if_good",
                        "after_evaluate", "before_evaluate", "has_attribute",
                        "decorate_particle","particle_is_instance"]
        if name in old_exceptions:
            return []
        #print "name", fullname
        if fullname in exceptions:
            return []
        if name.endswith("swigregister"):
            return []
        if name.lower() != name:
            if name[0].lower() != name[0] and name.split('_')[0] in all:
                # static methods
                return []
            else:
                return [fullname]
        starts=False
        for v in verbs:
            if name.startswith(v):
                starts=True
                break
        if not starts:
            return [fullname]
        tokens= name.split("_")
        for t in tokens:
            if not self._check_spelling(t, words):
                misspelled.append(t)
                print "misspelled", t, "in", name
                return [fullname]
        return []
    def _check_function_names(self, module, prefix, names, verbs, all, exceptions, words, misspelled):
        bad=[]
        #print "names", module, prefix
        for name in names:
            if name.startswith("_") or name =="weakref_proxy":
                continue
            if self._get_type(module, name)==types.BuiltinMethodType\
                   or self._get_type(module, name)==types.MethodType:
                bad.extend(self._check_function_name(prefix, name, verbs, all, exceptions, words, misspelled))
            if self._get_type(module, name)==types.TypeType and name.find("SwigPyIterator")==-1:
                #print "sub", module+"."+name
                members=eval("dir("+module+"."+name+")")
                #print members
                bad.extend(self._check_function_names(module+"."+name,
                                                      name,
                                                      members,
                                                      verbs, [], exceptions, words, misspelled))
        return bad



    def assertFunctionNames(self, module, exceptions, words):
        """Check that all the functions in the module follow the imp naming conventions."""
        all= dir(module)
        verbs=["add", "remove", "get", "set", "evaluate", "show", "create", "destroy",
               "push", "pop", "write", "read", "do", "show", "load", "save", "reset",
               "clear", "handle", "update", "apply", "optimize", "reserve", "dump",
               "propose", "setup", "teardown", "visit", "find", "run", "swap"]
        misspelled = []
        bad=self._check_function_names(module.__name__, None, all, verbs, all, exceptions, words, misspelled)
        message="All IMP methods should have lower case names separated by underscores and beginning with a verb, preferably one of ['add', 'remove', 'get', 'set', 'create', 'destroy']. Each of the words should be a properly spelled english word. The following do not (given our limited list of verbs that we check for):\n%(bad)s\nIf there is a good reason for them not to (eg it does start with a verb, just one with a meaning that is not covered by the normal list), add them to the function_name_exceptions variable in the IMPModuleTest call. Otherwise, please fix. The current verb list is %(verbs)s" \
                          % {"bad":"\n".join(bad), "verbs":verbs}
        if len(misspelled) > 0:
            message += "\nMisspelled words: " + ", ".join(set(misspelled)) \
                       + ". Add words to the spelling_exceptions variable " \
                       + "of the IMPModuleTest if needed."
        self.assertEquals(len(bad), 0,
                          message)


    def assertShow(self, modulename, exceptions):
        """Check that all the classes in modulename have a show method"""
        all= dir(modulename)
        not_found=[]
        for f in all:
            # Exclude Python-only classes; they are all showable
            if not eval('hasattr(%s.%s, "__swig_destroy__")' \
                        % (modulename.__name__, f)):
                continue
            if self._get_type(modulename.__name__, f) == types.TypeType\
                   and not f.startswith("_") \
                   and not f.endswith("_swigregister")\
                   and f not in exceptions\
                   and not f.endswith("Temp") and not f.endswith("Iterator")\
                   and not f.endswith("Exception") and\
                   f not in eval(modulename.__name__+"._raii_types") and \
                   f not in eval(modulename.__name__+"._plural_types"):
                if not hasattr(getattr(modulename, f), 'show'):
                    not_found.append(f)
        message="All IMP classes should support show and __str__. The following do not:\n%s\n If there is a good reason for them not to, add them to the show_exceptions variable in the IMPModuleTest call. Otherwise, please fix." \
                          % "\n".join(not_found)
        self.assertEquals(len(not_found), 0,
                          message)

    def run_example(self, filename):
        """Run the named example script.
           A dictionary of all the script's global variables is returned.
           This can be queried in a test case to make sure the example
           performed correctly."""
        class _FatalError(Exception): pass

        # Add directory containing the example to sys.path, so it can import
        # other Python modules in the same directory
        path, name = os.path.split(filename)
        oldsyspath = sys.path[:]
        sys.path.append(path)
        vars = {}
        try:
            try:
                exec open(filename) in vars
            # Catch sys.exit() called from within the example; a non-zero exit
            # value should cause the test case to fail
            except SystemExit, e:
                if e.code != 0 and e.code is not None:
                    raise _FatalError("Example exit with code %s" % str(e.code))
        finally:
            # Restore sys.path (note that Python 2.3 does not allow
            # try/except/finally, so we need to use nested trys)
            sys.path = oldsyspath

        return vars


class _TestResult(unittest.TextTestResult):
    def getDescription(self, test):
        doc_first_line = test.shortDescription()
        if self.descriptions and doc_first_line:
            return doc_first_line
        else:
            return str(test)


class _TestRunner(unittest.TextTestRunner):
    def _makeResult(self):
        return _TestResult(self.stream, self.descriptions, self.verbosity)


def main(*args, **keys):
    """Run a set of tests; essentially the same as unittest.main(). Obviates
       the need to separately import the 'unittest' module, and ensures that
       main() is from the same unittest module that the IMP.test testcases
       are."""
    return unittest.main(testRunner=_TestRunner, *args, **keys)

try:
    import subprocess
    class _SubprocessWrapper(subprocess.Popen):
        def __init__(self, app, args):
            # For (non-Python) applications to work on Windows, the
            # PATH must include the directory containing built DLLs
            if sys.platform == 'win32' and app != sys.executable:
                # Hack to find the location of build/lib/
                libdir = os.environ['PYTHONPATH'].split(';')[0]
                env = os.environ.copy()
                env['PATH'] += ';' + libdir
            else:
                env = None
            subprocess.Popen.__init__(self, [app]+list(args),
                                      stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, env=env)
        def wait(self):
            err = self.stderr.read()
            ret = subprocess.Popen.wait(self)
            return ret, err
except ImportError:
    # Provide a subprocess workalike for Python 2.3 systems (e.g. old Macs)
    class _SubprocessWrapper(object):
        def __init__(self, app, args):
            self.stdin, self.stdout, self.stderr = \
                             os.popen3(app + " " + " ".join(args))
        def wait(self):
            return (0, "")

class ApplicationTestCase(TestCase):
    """Super class for simple IMP application test cases"""
    def _get_application_file_name(self, filename):
        # If we ran from run-all-tests.py, it set an env variable for us with
        # the top-level test directory
        if sys.platform == 'win32':
            filename += '.exe'
        #if 'IMP_BUILD_ROOT' in os.environ:
        #    testdir = os.environ['IMP_BUILD_ROOT']
        #    return os.path.join(testdir, "build", "bin", filename)
        return filename

    def run_application(self, app, args):
        """Run an application with the given list of arguments.
           @return a subprocess.Popen-like object containing the child stdin,
                   stdout and stderr.
        """
        filename = self._get_application_file_name(app)
        print "running ", filename
        if sys.platform == 'win32':
            # Cannot rely on PATH on wine builds, so use full pathname
            return _SubprocessWrapper(os.path.join(os.environ['IMP_BIN_DIR'],
                                                   filename), args)
        else:
            return _SubprocessWrapper(filename, args)

    def run_python_application(self, app, args):
        """Run a Python application with the given list of arguments.
           The Python application should be self-runnable (i.e. it should
           be executable and with a #! on the first line).
           @return a subprocess.Popen-like object containing the child stdin,
                   stdout and stderr.
        """
        # Handle platforms where /usr/bin/python doesn't work
        if sys.executable != '/usr/bin/python':
            return _SubprocessWrapper(sys.executable,
                         [os.path.join(os.environ['IMP_BIN_DIR'], app)] + args)
        else:
            return _SubprocessWrapper(app, args)

    def run_script(self, app, args):
        """Run an application with the given list of arguments.
           @return a subprocess.Popen-like object containing the child stdin,
                   stdout and stderr.
        """
        print "running ", app
        return _SubprocessWrapper(sys.executable, [app]+args)

    def assertApplicationExitedCleanly(self, ret, error):
        """Assert that the application exited cleanly, i.e. that the
           return value is zero."""
        if ret < 0:
            raise OSError("Application exited with signal %d\n" % -ret\
                          +error)
        else:
            self.assertEqual(ret, 0,
                       "Application exited uncleanly, with exit code %d\n" % ret\
                             + error)



class TestRefiner(IMP.Refiner):
    """A class which makes sure that the right particles are passed back."""

    def __init__(self, pr):
        IMP.Refiner.__init__(self)
        self.pr= pr
        self.dict={}

    def do_show(self, fh):
        print >> fh, "Testing particle refiner"
        self.pr.show()

    def get_version_info(self):
        return IMP.VersionInfo("Daniel Russel", "0.5")

    def get_can_refine(self, p):
        print "Can refine? "+ str(p.get_name())
        print "Return is " + str(self.pr.get_can_refine(p))
        return self.pr.get_can_refine(p)

    def get_refined(self, p):
        print "Refining "+ str(p.get_name())
        ps= self.pr.get_refined(p)
        self.dict[p.get_name()] = ps
        print self.dict
        return ps


class RefCountChecker(object):
    """Check to make sure the number of C++ object references is as expected"""

    def __init__(self, testcase):
        # Make sure no director objects are hanging around; otherwise these
        # may be unexpectedly garbage collected later, decreasing the
        # live object count
        IMP._director_objects.cleanup()
        self.__testcase = testcase
        if IMP.build != "fast":
            self.__basenum = IMP.base.RefCounted.get_number_of_live_objects()

    def assert_number(self, expected):
        "Make sure that the number of references matches the expected value."
        t = self.__testcase
        IMP._director_objects.cleanup()
        if IMP.build != "fast":
            t.assertEqual(IMP.base.RefCounted.get_number_of_live_objects() \
                          - self.__basenum, expected)


class DirectorObjectChecker(object):
    """Check to make sure the number of director references is as expected"""

    def __init__(self, testcase):
        IMP._director_objects.cleanup()
        self.__testcase = testcase
        self.__basenum = IMP._director_objects.get_object_count()

    def assert_number(self, expected, force_cleanup=True):
        """Make sure that the number of references matches the expected value.
           If force_cleanup is set, clean up any unused references first before
           doing the assertion.
        """
        t = self.__testcase
        if force_cleanup:
            IMP._director_objects.cleanup()
        t.assertEqual(IMP._director_objects.get_object_count() \
                      - self.__basenum, expected)
