Import("env")
import os

def after_build(source, target, env):
    print("Generating compile_commands.json for the current environment...")
    env.Execute(f"{env.subst('$PYTHONEXE')} -m platformio run -e {env.subst('$PIOENV')} -t compiledb")

env.AddPostAction("checkprogsize", after_build)
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)