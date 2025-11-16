#!/bin/bash

set -e

PS3="Please enter your choice:"

conf=""

function get_conf()
{
	plat=loongson${1}
	opts=(`cd arch/loongarch/configs && ls ${plat}*`)
	select opt in "${opts[@]}"
	do
		conf=$opt
		break
	done

	echo
	echo "Your select is:${conf}"
	echo
}

function usage()
{
	echo "Usage:"
	echo "./buildenv.sh <chip> [options]"
	echo "chip: <2k300/2k500/2k1000/2p500/all>"
	echo "options: it's same as u-boot make defconfig"
}

function main()
{
	if [ $# -ge 1 ]; then
	       param=${1}	
	       if [ ${param} = "all" ]; then
		       get_conf
		else
			get_conf "_"$param
		fi

		shift
		make $conf  $@
	else
		usage
	fi
}

main $@

