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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/src/ucl_drone_gui"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/install/lib/python2.7/dist-packages:/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/build" \
    "/usr/bin/python" \
    "/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/src/ucl_drone_gui/setup.py" \
    build --build-base "/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/build/ucl_drone_gui" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/install" --install-scripts="/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/install/bin"
