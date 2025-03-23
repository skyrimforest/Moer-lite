#!/bin/bash
if [ $# -ne 1 ]; then
    echo "请提供一个文件夹路径作为参数"
    exit 1
fi
folder_path="$1"
if [ ! -d "$folder_path" ]; then
    echo "指定的路径不是一个有效的文件夹"
    exit 1
fi
./target/bin/Moer "$folder_path"