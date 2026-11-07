# Pre-build hook: put the shared header-only core on the include path.
#
# platformio.ini cannot express this portably: the ${VAR} substitutions that
# work for other tools either are not defined here or expand to nothing (which
# silently turns -I${X}/../core/include into -I/../core/include, and the build
# then fails with "desk/controller.h: No such file" three directories away from
# the actual problem). Resolving the path in Python against this file's own
# location has exactly one failure mode, which is a missing core/ directory.
import os

Import("env")

# __file__ is not defined when SCons exec()s this script, and PLATFORMIO_* env
# vars are not guaranteed either. Dir("#") is SCons's own handle for the
# project root (firmware/), which is what makes the path reliable.
PROJECT_ROOT = Dir("#").abspath
CORE_INCLUDE = os.path.normpath(os.path.join(PROJECT_ROOT, "..", "core", "include"))

if not os.path.isdir(os.path.join(CORE_INCLUDE, "desk")):
    print("deskbot: FATAL - shared core not found at %s" % CORE_INCLUDE)
    print("deskbot: the firmware is only a HAL; all motion logic lives in core/.")
    Exit(1)

env.Append(CPPPATH=[CORE_INCLUDE])
print("deskbot: core include -> %s" % CORE_INCLUDE)
