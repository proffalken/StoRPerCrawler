Import("env")
import os

def strip_conflicting_atomic(target, source, env):
    # Locate the built micro-ROS library in libdeps
    lib = os.path.join(env['PROJECT_LIBDEPS_DIR'], env['PIOENV'], 'micro_ros_platformio', 'libmicroros', 'libmicroros.a')
    if os.path.isfile(lib):
        print("→ Stripping rcutils 64-bit atomics from libmicroros.a")
        # Remove the conflicting object (rcutils's 64-bit atomic implementation)
        cmd = f"{env['AR']} d {lib} librcutils__atomic_64bits.c.obj"
        env.Execute(cmd)

# Hook this before firmware linking
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", strip_conflicting_atomic)

