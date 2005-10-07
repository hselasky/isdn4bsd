#include <dss1_aoc.h>

enum states {
  ST_EXP_COMP_TYP,
  ST_EXP_INV_ID,
  ST_EXP_OP_VAL,
  ST_EXP_INFO,
  ST_EXP_RUL,
  ST_EXP_RU,
  ST_EXP_RNOU,
  ST_EXP_TOCI,
  ST_EXP_DBID,    

  ST_EXP_RR_INV_ID,
  ST_EXP_RR_OP_VAL,
  ST_EXP_RR_RESULT,       

  ST_EXP_REJ_INV_ID,
  ST_EXP_REJ_OP_VAL,
  ST_EXP_REJ_RESULT,      
        
  ST_EXP_NIX      
};
