#!/bin/bash
file_path=$(cd "$(dirname "$0")"; pwd)
rm -rf "${file_path}/build"
cmake -S "${file_path}" -B "${file_path}/build"
cmake --build "${file_path}/build" --target all -- -j 4
