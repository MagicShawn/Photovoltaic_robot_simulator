#!/bin/bash
for pkg in $(catkin list -u); do
    echo "=== $pkg ==="
    rosdep keys $pkg
    echo ""
done