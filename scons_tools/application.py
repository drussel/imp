import utility
import pyscanner
import dependency
import doc
import bug_fixes
import install
import test
import data
import environment
import scons_tools
from SCons.Script import Builder, File, Action, Glob, Return, Alias, Dir


def IMPApplication(env, name,
                   authors,
                   brief, overview,
                   version=None,
                   publications=None,
                   license="standard",
                   required_modules=[],
                   optional_dependencies=[],
                   required_dependencies=[],
                   python=True):
    if env.GetOption('help'):
        return
    (nenv, version, found_optional_modules, found_optional_dependencies) =\
         utility.configure(env, name, "application", version,
                           required_modules=required_modules,
                           optional_dependencies=optional_dependencies,
                           required_dependencies= required_dependencies)
    if not nenv:
        data.get(env).add_application(name, ok=False)
        return
    else:
        if python:
            pm=required_modules+found_optional_modules
        else:
            pm=[]
        lkname="application_"+name.replace(" ", "_").replace(":", "_")
        pre="\page "+lkname+" "+name
        doc.add_doc_page(env, pre,
                         authors, version,
                         brief, overview,
                         publications,
                         license)
        data.get(env).add_application(name, link="\\ref "+lkname+' "'+name+'"',
                                      dependencies=required_dependencies\
                                          +found_optional_dependencies,
                                      unfound_dependencies=[x for x in optional_dependencies
                                                            if not x in
                                                            found_optional_dependencies],
                                      modules= required_modules+found_optional_modules,
                                      python_modules=pm,
                                      version=version)


    env= environment.get_bin_environment(nenv)
    scons_tools.data.get(env).add_to_alias("all", env.Alias(name))
    dirs = Glob("*/SConscript")
    for d in dirs:
        env.SConscript(d, exports=['env'])
    return env

def IMPCPPExecutable(envi, target, source):
    if envi.GetOption('help'):
        return
    env= environment.get_bin_environment(envi)

    prog= env.Program(target="#/build/bin/"+target, source=source)


def IMPCPPExecutables(envi, lst):
    if envi.GetOption('help'):
        return
    env= environment.get_bin_environment(envi)
    for l in lst:
        prog= env.Program(target="#/build/bin/"+l[0], source=l[1])


def IMPPythonExecutable(env, file):
    def dummy(target, source, env): pass
    _PythonExeDependency = Builder(action=Action(dummy, dummy),
                                   source_scanner=pyscanner.PythonScanner)

    if env.GetOption('help'):
        return
    inst = install.install(env, "bindir", file)

    # Make sure that when we install a Python executable we first build
    # any Python modules it uses (env.Install() does not appear to accept
    # source_scanner, so we use a dummy do-nothing builder to add these
    # dependencies)
    pydep = _PythonExeDependency(env, target=None, source=file)
    env.Depends(inst, pydep)
    env.Append(IMP_PYTHON_EXECUTABLES=[file])


def IMPApplicationTest(env, python_tests=[]):
    files= [File(x).abspath for x in python_tests]
    tests = test.add_tests(env, source=files, type='application unit test')
    for t in tests:
        env.Requires(t, env.Alias(environment.get_current_name(env)))
