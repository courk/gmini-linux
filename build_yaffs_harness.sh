#!/bin/sh
set -e

export LLVM_CONFIG=/usr/bin/llvm-config-6.0

make -C AFL
make -C AFL/llvm_mode/

make -C ff-gcc/

make -C tools/lkl -j INST_CC="$(pwd)/AFL/afl-clang-fast" CC="$(pwd)/ff-gcc/ff-gcc fs/yaffs2"
