#!/bin/sh

#
# The following script will generate a set of empty header files for
# all header files used in the project. 
#

pre_scan()
{
	while read F
	do
		[ ! -z "$1" ] && (find $1/dev/usb -name "$F")
		find ../src.port -name "$F"
	done
}

scan()
{
    while read F
    do
	grep -E "^[#	 ]*include[	 ]*[\"<]" "$F" | \
	    sed -e "s/[>\"].*[/][*].*//g" | \
	    sed -e "s/[<>	 #\"]//g" |\
	    sed -e "s/^include//g" |\
	    grep -v -E "sound/|ndis/|usb/|usb2/|i4b/" >> temp0

	grep -E "^[	 ]*DEVMETHOD[(]" "$F" | sed -e "s/^[	 ]*DEVMETHOD[(]//g" | sed -e "s/[	 ]//g" | sed -e "s/[,].*//g" >> temp1

	grep -E "^DRIVER_MODULE[(]" "$F" | sed -e "s/[)][,;]/)/g" >> bsd_module_driver.h
	grep -E "^MODULE_DEPEND[(]" "$F" | sed -e "s/[)][,;]/)/g" >> bsd_module_depend.h
	grep -E "^MODULE_VERSION[(]" "$F" | sed -e "s/[)][,;]/)/g" >> bsd_module_version.h
	grep -E "^SYSINIT[(]" "$F" | sed -e "s/[)][,;]/)/g" >> bsd_module_sysinit.h
	grep -E "^SYSUNINIT[(]" "$F" | sed -e "s/[)][,;]/)/g" >> bsd_module_sysuninit.h
	grep -E "^SYSCTL_[:alpha:_]*[(]" "$F" | sed -e "s/[)][,;]/)/g" >> bsd_module_sysctl.h

	echo -n "."
    done
}

mkautogen()
{
cat << EOF
/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: `date`
 */
EOF
}

mkempty()
{
    while read F
    do
	[ -d $F ] && (rmdir $F)
	[ -f $F ] || (mkdir -p $F; rmdir $F; touch $F; echo $F)
    done
}

mkdevmethod()
{
    while read F
    do
      echo "#define `echo $F | tr "[:lower:]" "[:upper:]"`(dev, ...) \\"
      echo "  (((${F}_t *)(device_get_method(dev, \"$F\")))(dev,## __VA_ARGS__))"
    done
}

mkrename()
{
    while read F
    do
      echo "#define ${F} fbsd_${F}"
    done
}

rm -f temp0 temp1

mkautogen > bsd_module_driver.h
mkautogen > bsd_module_depend.h
mkautogen > bsd_module_version.h
mkautogen > bsd_module_devmethod.h
mkautogen > bsd_module_rename.h
mkautogen > bsd_module_sysctl.h
mkautogen > bsd_module_sysinit.h
mkautogen > bsd_module_sysuninit.h

echo -n "Scanning "

cat Makefile.inc | sed -e "s/^SRCS..//g" | grep -v "^#" | pre_scan "`cat .sysdir`" | scan

echo " done"

echo "Generating:"

sort temp0 | uniq \
    | mkempty

sort temp1 | uniq \
    | mkdevmethod >> bsd_module_devmethod.h

sort module_rename_list | uniq \
    | mkrename >> bsd_module_rename.h

rm -f temp0 temp1

echo "Done."

