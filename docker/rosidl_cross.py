# Takes the place of a rosidl generator script in an arm64 image built on amd64 (os.Dockerfile): runs the original,
# <this file>.native, in the Python of the build machine under /opt/cross-python, with the same arguments and the same
# module path, only the standard library is that interpreter's own.
import os
import sys

home = '/opt/cross-python'
stdlib = os.path.dirname(os.__file__)
own = {stdlib, os.path.join(stdlib, 'lib-dynload'),
       os.path.join(sys.prefix, 'lib', 'python%d%d.zip' % sys.version_info[:2])}
path = [home + p[len(sys.prefix):] if p in own else p for p in sys.path]
boot = ('import runpy, sys; sys.path[:] = %r; del sys.argv[0]; '
        'runpy.run_path(sys.argv[0] + ".native", run_name="__main__")' % (path,))
python = home + '/bin/python3'
os.execv(python, [python, '-S', '-c', boot] + sys.argv)
