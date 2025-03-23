#!/bin/bash

# 检测 build 目录是否存在
if [ ! -d "./build" ]; then
    mkdir build
else
    rm -rf ./build
    mkdir build
fi

# 进入 build 目录
cd build

# 执行 cmake 和 make 操作
cmake ..
make -j4

# 执行完操作后切换回上一级目录（如果需要）
cd ..