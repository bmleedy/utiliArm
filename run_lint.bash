#!/bin/bash

echo "run_lint.bash: LINTING START=========================================================="


cpplint --recursive --exclude=./components/ESP32Servo/src/*.cpp --exclude=./components/ESP32Servo/src/*.h --exclude=./components/ESP32Servo/examples/*.cpp ./main ./components 2>&1

MESSAGE=`cpplint --recursive --exclude=./components/ESP32Servo/src/*.cpp --exclude=./components/ESP32Servo/src/*.h --exclude=./components/ESP32Servo/examples/*.cpp ./main ./components 2>&1 | grep "Total errors found"`

if [ "${MESSAGE}" = "" ]; then
    echo "run_lint.bash: ======================================================================="
    echo "run_lint.bash: linting success!";
    echo "run_lint.bash: ======================================================================="
    exit 0;
else
    echo "run_lint.bash: ======================================================================="
    echo "run_lint.bash: linting failure :      ${MESSAGE}";
    echo "run_lint.bash: ======================================================================="
    exit 1;
fi
