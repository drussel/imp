import colorize_python
from SCons.Script import Glob, Dir, File, Builder, Action
import SCons.Node.FS
import os
import os.path
import hierarchy


def _action_colorize_python(target, source, env):
    colorize_python.Parser(open(str(source[0]), "r").read(),
                           open(str(target[0]), "w")).format(None, None)

def _print_colorize_python(target, source, env):
    print "Generating doxygen pages for examples"

ColorizePython = Builder(action=Action(_action_colorize_python,
                                       _print_colorize_python))


def nice_name(string):
    return string[string.rfind("examples/")+9:]

def write_doxygen(env, name, files, outputname):
    #print "writing "+outputname
    outfile= file(outputname, 'w')
    outfile.write("/**\n")
    outfile.write("\page "+name.replace(".", "_")+"_examples"+ " " + name+" examples\n\n")
    for f in files:
        if str(f).endswith(".py") or str(f).endswith(".cpp"):
            rm= open(os.path.splitext(f.abspath)[0]+".readme", "r").read()
            nm= os.path.splitext(os.path.split(str(f))[1])[0]
            outfile.write("\section " +nm + " " + nice_name(f.abspath)+"\n\n")
            outfile.write(rm+"\n\n")
        if str(f).endswith(".py"):
            outfile.write("\htmlinclude "+ nm+".py.html\n\n")
        if str(f).endswith(".cpp"):
            outfile.write("\include "+nm+".cpp\n\n")

    outfile.write("*/\n")


def _action_make_examples(target, source, env):
    name= env['IMP_MODULE_NICENAME']
    write_doxygen(env, name, source, target[0].path)

def _print_make_examples(target, source, env):
    print "Generating doxygen page for examples"

MakeDox = Builder(action=Action(_action_make_examples,
                                _print_make_examples))



def handle_example_dir(env, inputpath, name, prefix, example_files, data_files):
    build=[]
    dox=[]
    install=[]
    example_files= [File(x) for x in example_files]
    test_files = [x for x in example_files if x.name.startswith("test_")]
    example_files = [x for x in example_files if not x.name.startswith("test_")]
    data_files= [File(x) for x in data_files]
    exampledir = env.GetInstallDirectory('docdir')+"/examples"
    for f in example_files:
        if str(f).endswith(".py"):
            c= env.IMPColorizePython(str(inputpath) + '/' \
                                      + os.path.dirname(str(f)) \
                                      + '/generated/' \
                                      + os.path.basename(str(f))+".html",
                                      f.abspath)
            dox.append(c)
            #install.append(env.Install(exampledir+"/"+prefix, f.abspath))
        #elif str(f).endswith(".readme"):
        #    install.append(env.Install(exampledir+"/"+prefix, f.abspath))
    install = hierarchy.InstallExampleHierarchy(env, exampledir+"/"+prefix, example_files+data_files, False)
    build = hierarchy.InstallExampleHierarchy(env, "#/build/doc/examples/"+prefix, example_files+data_files, True)
    test= env.IMPModuleTest('tests.passed',
                             ["#/tools/imppy.sh",
                              "#/scons_tools/run-all-examples.py"]\
                             +[x for x in example_files
                               if str(x).endswith(".py") \
                               and str(x).find("fragment")==-1] + test_files,
                             TEST_TYPE='example')
    env.AlwaysBuild("tests.passed")
    doxpage= env.IMPModuleExamplesDox(File(str(inputpath)+"/generated/examples.dox"), example_files)
    dox.append(doxpage)
    return (dox, build, install, test)
