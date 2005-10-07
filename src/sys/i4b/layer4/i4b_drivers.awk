#!/usr/bin/awk
#
# this script produce an assembly-file that re-allocates
# missing i4b-functions to a memory-address of zero.
#
function m(object,prefix) {
 if(index(OBJ,object) == 0)
 {
   printf \
   ".global %s_response_to_user\n"\
   ".set %s_response_to_user,0\n"\
   "\n"\
   ".global %s_setup_ft\n"\
   ".set %s_setup_ft,0\n"\
   "\n",prefix,prefix,prefix,prefix;
 }
}

index($0,"i4b") { OBJ=OBJ $0 }

END {
#
#object files to check for
#
  m("i4b_ibc.o","ibc") ;
  m("i4b_ing.o","ing") ;
  m("i4b_ipr.o","ipr") ;
  m("i4b_isppp.o","i4bisppp") ;
  m("i4b_rbch.o","rbch") ;
  m("i4b_tel.o","tel") ;
  m("i4b_ihfc2_dev.o","ihfc_dev") ;
  m("i4b_capidrv.o","capi") ;
  m("dss1_l2fsm.o","dss1") ;

  m("i4b_unknown.o","diehl") ;
  m("i4b_unknown.o","tina_dd") ;
  m("i4b_unknown.o","amv_b1") ;
}
