/*
 * PicoRuby AMY - VM selection wrapper
 */

#include "../include/amy_gem.h"

#if defined(PICORB_VM_MRUBY)
  #include "mruby/amy.c"
#elif defined(PICORB_VM_MRUBYC)
  #include "mrubyc/amy.c"
#endif
