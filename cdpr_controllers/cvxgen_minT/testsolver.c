/* Produced by CVXGEN, 2017-06-27 04:20:17 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
	int i;
    for ( i = 0; i < 8; i++)
  printf("  %9.4f\n", vars.x[i]);
  printf("  %d\n", num_iters);
  return 0;
}
void load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.5507979025745755;
  params.Q[8] = 0;
  params.Q[16] = 0;
  params.Q[24] = 0;
  params.Q[32] = 0;
  params.Q[40] = 0;
  params.Q[48] = 0;
  params.Q[56] = 0;
  params.Q[1] = 0;
  params.Q[9] = 1.7081478226181048;
  params.Q[17] = 0;
  params.Q[25] = 0;
  params.Q[33] = 0;
  params.Q[41] = 0;
  params.Q[49] = 0;
  params.Q[57] = 0;
  params.Q[2] = 0;
  params.Q[10] = 0;
  params.Q[18] = 1.2909047389129444;
  params.Q[26] = 0;
  params.Q[34] = 0;
  params.Q[42] = 0;
  params.Q[50] = 0;
  params.Q[58] = 0;
  params.Q[3] = 0;
  params.Q[11] = 0;
  params.Q[19] = 0;
  params.Q[27] = 1.510827605197663;
  params.Q[35] = 0;
  params.Q[43] = 0;
  params.Q[51] = 0;
  params.Q[59] = 0;
  params.Q[4] = 0;
  params.Q[12] = 0;
  params.Q[20] = 0;
  params.Q[28] = 0;
  params.Q[36] = 1.8929469543476547;
  params.Q[44] = 0;
  params.Q[52] = 0;
  params.Q[60] = 0;
  params.Q[5] = 0;
  params.Q[13] = 0;
  params.Q[21] = 0;
  params.Q[29] = 0;
  params.Q[37] = 0;
  params.Q[45] = 1.896293088933438;
  params.Q[53] = 0;
  params.Q[61] = 0;
  params.Q[6] = 0;
  params.Q[14] = 0;
  params.Q[22] = 0;
  params.Q[30] = 0;
  params.Q[38] = 0;
  params.Q[46] = 0;
  params.Q[54] = 1.1255853104638363;
  params.Q[62] = 0;
  params.Q[7] = 0;
  params.Q[15] = 0;
  params.Q[23] = 0;
  params.Q[31] = 0;
  params.Q[39] = 0;
  params.Q[47] = 0;
  params.Q[55] = 0;
  params.Q[63] = 1.2072428781381868;
  params.c[0] = 0;
  params.c[1] = 0;
  params.c[2] = 0;
  params.c[3] = -0;
  params.c[4] = 0;
  params.c[5] = 0;
  params.c[6] = 0;
  params.c[7] = 0;
  params.A[0] = -1.9040724704913385;
  params.A[1] = 0.23541635196352795;
  params.A[2] = -0.9629902123701384;
  params.A[3] = -0.3395952119597214;
  params.A[4] = -0.865899672914725;
  params.A[5] = 0.7725516732519853;
  params.A[6] = -0.23818512931704205;
  params.A[7] = -1.372529046100147;
  params.A[8] = 0.17859607212737894;
  params.A[9] = 1.1212590580454682;
  params.A[10] = -0.774545870495281;
  params.A[11] = -1.1121684642712744;
  params.A[12] = -0.44811496977740495;
  params.A[13] = 1.7455345994417217;
  params.A[14] = 1.9039816898917352;
  params.A[15] = 0.6895347036512547;
  params.A[16] = 1.6113364341535923;
  params.A[17] = 1.383003485172717;
  params.A[18] = -0.48802383468444344;
  params.A[19] = -1.631131964513103;
  params.A[20] = 0.6136436100941447;
  params.A[21] = 0.2313630495538037;
  params.A[22] = -0.5537409477496875;
  params.A[23] = -1.0997819806406723;
  params.A[24] = -0.3739203344950055;
  params.A[25] = -0.12423900520332376;
  params.A[26] = -0.923057686995755;
  params.A[27] = -0.8328289030982696;
  params.A[28] = -0.16925440270808823;
  params.A[29] = 1.442135651787706;
  params.A[30] = 0.34501161787128565;
  params.A[31] = -0.8660485502711608;
  params.A[32] = -0.8880899735055947;
  params.A[33] = -0.1815116979122129;
  params.A[34] = -1.17835862158005;
  params.A[35] = -1.1944851558277074;
  params.A[36] = 0.05614023926976763;
  params.A[37] = -1.6510825248767813;
  params.A[38] = -0.06565787059365391;
  params.A[39] = -0.5512951504486665;
  params.A[40] = 0.8307464872626844;
  params.A[41] = 0.9869848924080182;
  params.A[42] = 0.7643716874230573;
  params.A[43] = 0.7567216550196565;
  params.A[44] = -0.5055995034042868;
  params.A[45] = 0.6725392189410702;
  params.A[46] = -0.6406053441727284;
  params.A[47] = 0.29117547947550015;
  params.b[0] = -0.6967713677405021;
  params.b[1] = -0.21941980294587182;
  params.b[2] = -1.753884276680243;
  params.b[3] = -1.0292983112626475;
  params.b[4] = 1.8864104246942706;
  params.b[5] = -1.077663182579704;
}
