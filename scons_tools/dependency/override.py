"""Simple check that compilation works"""
import scons_tools.environment
from SCons.Script import Exit

def _check(context):
    context.Message('Checking if the compiler supports override... ')
    ret = context.TryCompile("""
struct Base {
    virtual void some_func(float);
};

struct Derived : Base {
    virtual void some_func(float) override;
};

        int main()
        {
            auto a=6;
            return 0;
        }
""", '.cpp')
    context.Result(ret)
    return ret

def configure_check(env):
    if env.get("c++11_override", "auto") == "auto":
        custom_tests = {'CheckCompiler':_check}
        tenv= scons_tools.environment.get_test_environment(env)
        conf = tenv.Configure(custom_tests=custom_tests)
        if conf.CheckCompiler():
            env["IMP_COMPILER_OVERRIDE"]=True
        else:
            env["IMP_COMPILER_OVERRIDE"]=False
        conf.Finish()
    elif env.get("c++11_override", "auto")=="no":
        print "Reading if the compiler supports override...no"
        env["IMP_COMPILER_OVERRIDE"]=False
    else:
        print "Reading if the compiler supports override...yes"
        env["IMP_COMPILER_OVERRIDE"]=True