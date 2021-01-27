#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/himanshu/nav_sys/src/turtlebot3_autorace/turtlebot3_autorace_camera"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/himanshu/nav_sys/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/himanshu/nav_sys/install/lib/python2.7/dist-packages:/home/himanshu/nav_sys/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/himanshu/nav_sys/build" \
    "/usr/bin/python2" \
    "/home/himanshu/nav_sys/src/turtlebot3_autorace/turtlebot3_autorace_camera/setup.py" \
     \
    build --build-base "/home/himanshu/nav_sys/build/turtlebot3_autorace/turtlebot3_autorace_camera" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/himanshu/nav_sys/install" --install-scripts="/home/himanshu/nav_sys/install/bin"
