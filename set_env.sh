#!/bin/bash

#set -x

function setup_mips_env()
{
	echo "====>setup env for MIPS..."
#	CC_PREFIX=/opt/mips64el-linux-gcc-8.x/host
#	export PATH=$CC_PREFIX/bin:$PATH
#	export LD_LIBRARY_PATH=$CC_PREFIX/lib:$LD_LIBRARY_PATH
#	export LD_LIBRARY_PATH=$CC_PREFIX/mips64el-buildroot-linux-gnu/lib64:$LD_LIBRARY_PATH

	CC_PREFIX=/opt/mips-loongson-gcc8-linux-gnu-2021-06-04
	export PATH=$CC_PREFIX/bin:$PATH
	export LD_LIBRARY_PATH=$CC_PREFIX/lib:$LD_LIBRARY_PATH

	export ARCH=mips
	export CROSS_COMPILE=mips-linux-gnu-
#	export CROSS_COMPILE=mips64el-linux-
}



function setup_loongarch_env()
{
	echo "====>setup env for LoongArch..."
	CC_PREFIX=/opt/loongarch64-linux-gnu-gcc13.3

	export PATH=$PATH:$CC_PREFIX/bin
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CC_PREFIX/lib
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CC_PREFIX/loongarch64-linux-gnu/lib64

	export ARCH=loongarch
	export CROSS_COMPILE=loongarch64-linux-gnu-
}


if [ $# -eq 1 ] ; then
	if [ "$1" == "mips" ]; then
		setup_mips_env
	else
		setup_loongarch_env
	fi
else
	setup_loongarch_env
fi

#set +x
