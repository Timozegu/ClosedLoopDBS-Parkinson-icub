#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _GP_reg(void);
extern void _Izhi2003b_reg(void);
extern void _Str_reg(void);
extern void _SubTN_reg(void);
extern void _thalamus_reg(void);

void modl_reg() {
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");
    fprintf(stderr, " \"GP.mod\"");
    fprintf(stderr, " \"Izhi2003b.mod\"");
    fprintf(stderr, " \"Str.mod\"");
    fprintf(stderr, " \"SubTN.mod\"");
    fprintf(stderr, " \"thalamus.mod\"");
    fprintf(stderr, "\n");
  }
  _GP_reg();
  _Izhi2003b_reg();
  _Str_reg();
  _SubTN_reg();
  _thalamus_reg();
}

#if defined(__cplusplus)
}
#endif
