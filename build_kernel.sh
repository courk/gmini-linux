#!/bin/bash
# Build kernel for use with Eureka.

exec_name=$0

set -o errtrace
trap 'echo Fatal error: script ${exec_name} aborting at line $LINENO, command \"$BASH_COMMAND\" returned $?; exit 1' ERR


deterministic_build=false
if [[ "$1" == "-d" ]]; then
  deterministic_build=true
  shift
fi

# Set variables for deterministic kernel builds
# These correspond to linux-3.8/scripts/mkcompile_h

if [[ "$deterministic_build" == true ]]; then
  export KBUILD_BUILD_VERSION=0
  export KBUILD_BUILD_TIMESTAMP="$(git log -1 --format=%cd) ($(git rev-parse HEAD | head -c 8))"
  export KBUILD_BUILD_USER=user
  export KBUILD_BUILD_HOST=host
else
  # Add git hash to date. This corresponds to scripts/mkcompile_h
  # This shows up in /proc/version
  export KBUILD_BUILD_TIMESTAMP="$(date) ($(git rev-parse HEAD | head -c 8))"
fi

arch=arm
cross_compile=armv7a-cros-linux-gnueabihf-

cpu_num=$(grep -c processor /proc/cpuinfo)

function usage(){
    echo "Usage: ${exec_name} [-d] [anchovy] [salami] [pepperoni] [chorizo] [pineapple] [mushroom] [caprica] [coco]"
    echo
    echo "-d : Build a deterministicly. Use git revision and timestamp and"
    echo "     generic user and host for /proc/version"
}

function run_kernel_make(){
    echo "***** building $5 *****"
    CROSS_COMPILE=$1 make -j$2 ARCH=$3 $4 CONFIG_DEBUG_SECTION_MISMATCH=y
    echo "***** completed building $5 *****"
}

function run_perf_make(){
    echo "***** building perf *****"
    LDFLAGS='-static-libgcc -static-libstdc++' \
    CROSS_COMPILE=$1 ARCH=$2 NO_DWARF=1 make -j$3
    echo "***** completed building perf *****"
}

function build_kernel(){
    local kernel_path=$(readlink -f $1)
    local product=$2
    local dts=$3

    pushd $kernel_path

    # Clean kernel
    run_kernel_make $cross_compile $cpu_num $arch clean
    # Build kernel config
    run_kernel_make $cross_compile $cpu_num $arch ${product}_defconfig
    # Verify kernel config
    diff .config arch/arm/configs/${product}_defconfig

    run_kernel_make $cross_compile $cpu_num $arch zImage-dtb.${dts}

    # Give specific names to output files.
    local out_path="arch/arm/boot/"
    mv $out_path/zImage-dtb.${dts} $out_path/zImage-dtb.${dts}.${product}
    popd
}

function build_dtb(){
    local kernel_path=$(readlink -f $1)
    local dtb_file=$2

    pushd $kernel_path
    # Build device tree blob
    run_kernel_make $cross_compile $cpu_num $arch $dtb_file
    popd
}


function build_perf(){
    local kernel_path=$(readlink -f $1)
    pushd ${kernel_path}/tools/perf
    run_perf_make $cross_compile $arch $cpu_num
    popd
}

function create_kernel_pkg(){
    local kernel_path=$(readlink -f $1)
    local pkg_dir=$(mktemp -d)
    local wd=$(pwd)

    for f in ${copy_file_list[@]}
    do
      s=${f%%:*}
      d=${f##*:}
      mkdir -p `dirname $pkg_dir/$d`
      echo "cp $kernel_path/$s $pkg_dir/$d"
      cp $kernel_path/$s $pkg_dir/$d
    done

    (cd $pkg_dir; tar zcvf $wd/kernel.tgz *)
    rm -fr $pkg_dir
}

if (( $# < 1 ))
then
    usage
    exit 2
fi

# List of files to be copied to the output tarball.
declare -a copy_file_list=()
readonly kernel_dir=.

for product in $* ; do
  case $product in
    anchovy)
      dts=berlin2cd-dongle
      build_kernel ${kernel_dir} ${product} ${dts}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)
      copy_file_list+=(tools/perf/perf:${product}/prebuilt/sdk/bin/perf)
      ;;

    salami)
      # Generate A0 device tree blob.
      a0_dts=berlin2cdp-a0-dongle
      a0_dtb=${a0_dts}.dtb

      build_kernel ${kernel_dir} ${product} ${a0_dts}
      build_dtb ${kernel_dir} ${a0_dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${a0_dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)
      copy_file_list+=(tools/perf/perf:${product}/prebuilt/sdk/bin/perf)

      for board in lexx-b3 lexx-b4 ; do
        copy_file_list+=(arch/arm/boot/dts/${a0_dtb}:salami/prebuilt/kernel/${board}.dtb)
      done
      ;;

    chorizo)
      a0_dts=berlin2cdp-a0-dongle
      a0_dtb=${a0_dts}.dtb

      build_kernel ${kernel_dir} ${product} ${a0_dts}
      build_dtb ${kernel_dir} ${a0_dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${a0_dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)

      for board in earth-b1 earth-b2 earth-b3 earth-b4 ; do
        copy_file_list+=(arch/arm/boot/dts/${a0_dtb}:${product}/prebuilt/kernel/${board}.dtb)
      done
      ;;

    pepperoni)
      dts=berlin2cdp-a0-hendrix
      dtb=${dts}.dtb

      build_kernel ${kernel_dir} ${product} ${dts}
      build_dtb ${kernel_dir} ${dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)
      copy_file_list+=(tools/perf/perf:${product}/prebuilt/sdk/bin/perf)

      for board in hendrix-b1 hendrix-b3 hendrix-b4 ; do
        copy_file_list+=(arch/arm/boot/dts/${dtb}:${product}/prebuilt/kernel/${board}.dtb)
      done
      ;;

    pineapple)
      dts=berlin2cdp-a0-chirp
      dtb=${dts}.dtb

      build_kernel ${kernel_dir} ${product} ${dts}
      build_dtb ${kernel_dir} ${dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)

      for board in chirp-b1 chirp-b3 chirp-b4 ; do
        copy_file_list+=(arch/arm/boot/dts/${dtb}:${product}/prebuilt/kernel/${board}.dtb)
      done
      ;;

    mushroom)
      dts=berlin2cdp-a0-joplin
      dtb=${dts}.dtb

      build_kernel ${kernel_dir} ${product} ${dts}
      build_dtb ${kernel_dir} ${dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)

      for board in joplin-b1 joplin-b3 joplin-b4 ; do
        copy_file_list+=(arch/arm/boot/dts/${dtb}:${product}/prebuilt/kernel/${board}.dtb)
      done
      ;;

    coco)
      dts=berlin2cdp-a0-coco
      dtb=${dts}.dtb

      build_kernel ${kernel_dir} ${product} ${dts}
      build_dtb ${kernel_dir} ${dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)

      for board in coco-p1 coco-p2; do
        copy_file_list+=(arch/arm/boot/dts/${dtb}:${product}/prebuilt/kernel/${board}.dtb)
      done
      ;;

    caprica)
      a0_dts=berlin2cdp-a0-caprica
      a0_dtb=${a0_dts}.dtb

      build_kernel ${kernel_dir} ${product} ${a0_dts}
      build_dtb ${kernel_dir} ${a0_dtb}

      copy_file_list+=(arch/arm/boot/zImage-dtb.${a0_dts}.${product}:${product}/prebuilt/kernel/zImage)
      copy_file_list+=(COPYING:${product}/prebuilt/kernel/COPYING)

      for board in caprica-b1 caprica-b2 caprica-b3 caprica-b4 ; do
        copy_file_list+=(arch/arm/boot/dts/${a0_dtb}:${product}/prebuilt/kernel/${board}.dtb)
      done
      ;;

    *)
      echo "unknown product: $product"
      exit 1
  esac
done

# Build the perf tool
build_perf ${kernel_dir}

# Create a kernel package for all products
create_kernel_pkg $kernel_dir
