/* Produced by CVXGEN, 2017-07-03 04:28:34 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[6])-rhs[2]*(params.A[12])-rhs[3]*(params.A[18])-rhs[4]*(params.A[24])-rhs[5]*(params.A[30])-rhs[6]*(params.A[36])-rhs[7]*(params.A[42])-rhs[8]*(params.A[48])-rhs[9]*(params.A[54])-rhs[10]*(params.A[60])-rhs[11]*(params.A[66])-rhs[12]*(params.A[72])-rhs[13]*(params.A[78]);
  lhs[1] = -rhs[0]*(params.A[1])-rhs[1]*(params.A[7])-rhs[2]*(params.A[13])-rhs[3]*(params.A[19])-rhs[4]*(params.A[25])-rhs[5]*(params.A[31])-rhs[6]*(params.A[37])-rhs[7]*(params.A[43])-rhs[8]*(params.A[49])-rhs[9]*(params.A[55])-rhs[10]*(params.A[61])-rhs[11]*(params.A[67])-rhs[12]*(params.A[73])-rhs[13]*(params.A[79]);
  lhs[2] = -rhs[0]*(params.A[2])-rhs[1]*(params.A[8])-rhs[2]*(params.A[14])-rhs[3]*(params.A[20])-rhs[4]*(params.A[26])-rhs[5]*(params.A[32])-rhs[6]*(params.A[38])-rhs[7]*(params.A[44])-rhs[8]*(params.A[50])-rhs[9]*(params.A[56])-rhs[10]*(params.A[62])-rhs[11]*(params.A[68])-rhs[12]*(params.A[74])-rhs[13]*(params.A[80]);
  lhs[3] = -rhs[0]*(params.A[3])-rhs[1]*(params.A[9])-rhs[2]*(params.A[15])-rhs[3]*(params.A[21])-rhs[4]*(params.A[27])-rhs[5]*(params.A[33])-rhs[6]*(params.A[39])-rhs[7]*(params.A[45])-rhs[8]*(params.A[51])-rhs[9]*(params.A[57])-rhs[10]*(params.A[63])-rhs[11]*(params.A[69])-rhs[12]*(params.A[75])-rhs[13]*(params.A[81]);
  lhs[4] = -rhs[0]*(params.A[4])-rhs[1]*(params.A[10])-rhs[2]*(params.A[16])-rhs[3]*(params.A[22])-rhs[4]*(params.A[28])-rhs[5]*(params.A[34])-rhs[6]*(params.A[40])-rhs[7]*(params.A[46])-rhs[8]*(params.A[52])-rhs[9]*(params.A[58])-rhs[10]*(params.A[64])-rhs[11]*(params.A[70])-rhs[12]*(params.A[76])-rhs[13]*(params.A[82]);
  lhs[5] = -rhs[0]*(params.A[5])-rhs[1]*(params.A[11])-rhs[2]*(params.A[17])-rhs[3]*(params.A[23])-rhs[4]*(params.A[29])-rhs[5]*(params.A[35])-rhs[6]*(params.A[41])-rhs[7]*(params.A[47])-rhs[8]*(params.A[53])-rhs[9]*(params.A[59])-rhs[10]*(params.A[65])-rhs[11]*(params.A[71])-rhs[12]*(params.A[77])-rhs[13]*(params.A[83]);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2])-rhs[3]*(params.A[3])-rhs[4]*(params.A[4])-rhs[5]*(params.A[5]);
  lhs[1] = -rhs[0]*(params.A[6])-rhs[1]*(params.A[7])-rhs[2]*(params.A[8])-rhs[3]*(params.A[9])-rhs[4]*(params.A[10])-rhs[5]*(params.A[11]);
  lhs[2] = -rhs[0]*(params.A[12])-rhs[1]*(params.A[13])-rhs[2]*(params.A[14])-rhs[3]*(params.A[15])-rhs[4]*(params.A[16])-rhs[5]*(params.A[17]);
  lhs[3] = -rhs[0]*(params.A[18])-rhs[1]*(params.A[19])-rhs[2]*(params.A[20])-rhs[3]*(params.A[21])-rhs[4]*(params.A[22])-rhs[5]*(params.A[23]);
  lhs[4] = -rhs[0]*(params.A[24])-rhs[1]*(params.A[25])-rhs[2]*(params.A[26])-rhs[3]*(params.A[27])-rhs[4]*(params.A[28])-rhs[5]*(params.A[29]);
  lhs[5] = -rhs[0]*(params.A[30])-rhs[1]*(params.A[31])-rhs[2]*(params.A[32])-rhs[3]*(params.A[33])-rhs[4]*(params.A[34])-rhs[5]*(params.A[35]);
  lhs[6] = -rhs[0]*(params.A[36])-rhs[1]*(params.A[37])-rhs[2]*(params.A[38])-rhs[3]*(params.A[39])-rhs[4]*(params.A[40])-rhs[5]*(params.A[41]);
  lhs[7] = -rhs[0]*(params.A[42])-rhs[1]*(params.A[43])-rhs[2]*(params.A[44])-rhs[3]*(params.A[45])-rhs[4]*(params.A[46])-rhs[5]*(params.A[47]);
  lhs[8] = -rhs[0]*(params.A[48])-rhs[1]*(params.A[49])-rhs[2]*(params.A[50])-rhs[3]*(params.A[51])-rhs[4]*(params.A[52])-rhs[5]*(params.A[53]);
  lhs[9] = -rhs[0]*(params.A[54])-rhs[1]*(params.A[55])-rhs[2]*(params.A[56])-rhs[3]*(params.A[57])-rhs[4]*(params.A[58])-rhs[5]*(params.A[59]);
  lhs[10] = -rhs[0]*(params.A[60])-rhs[1]*(params.A[61])-rhs[2]*(params.A[62])-rhs[3]*(params.A[63])-rhs[4]*(params.A[64])-rhs[5]*(params.A[65]);
  lhs[11] = -rhs[0]*(params.A[66])-rhs[1]*(params.A[67])-rhs[2]*(params.A[68])-rhs[3]*(params.A[69])-rhs[4]*(params.A[70])-rhs[5]*(params.A[71]);
  lhs[12] = -rhs[0]*(params.A[72])-rhs[1]*(params.A[73])-rhs[2]*(params.A[74])-rhs[3]*(params.A[75])-rhs[4]*(params.A[76])-rhs[5]*(params.A[77]);
  lhs[13] = -rhs[0]*(params.A[78])-rhs[1]*(params.A[79])-rhs[2]*(params.A[80])-rhs[3]*(params.A[81])-rhs[4]*(params.A[82])-rhs[5]*(params.A[83]);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[6]*(-1);
  lhs[7] = -rhs[7]*(-1);
  lhs[8] = -rhs[0]*(1);
  lhs[9] = -rhs[1]*(1);
  lhs[10] = -rhs[2]*(1);
  lhs[11] = -rhs[3]*(1);
  lhs[12] = -rhs[4]*(1);
  lhs[13] = -rhs[5]*(1);
  lhs[14] = -rhs[6]*(1);
  lhs[15] = -rhs[7]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[8]*(1);
  lhs[1] = -rhs[1]*(-1)-rhs[9]*(1);
  lhs[2] = -rhs[2]*(-1)-rhs[10]*(1);
  lhs[3] = -rhs[3]*(-1)-rhs[11]*(1);
  lhs[4] = -rhs[4]*(-1)-rhs[12]*(1);
  lhs[5] = -rhs[5]*(-1)-rhs[13]*(1);
  lhs[6] = -rhs[6]*(-1)-rhs[14]*(1);
  lhs[7] = -rhs[7]*(-1)-rhs[15]*(1);
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.Q[0])+rhs[1]*(2*params.Q[14])+rhs[2]*(2*params.Q[28])+rhs[3]*(2*params.Q[42])+rhs[4]*(2*params.Q[56])+rhs[5]*(2*params.Q[70])+rhs[6]*(2*params.Q[84])+rhs[7]*(2*params.Q[98])+rhs[8]*(2*params.Q[112])+rhs[9]*(2*params.Q[126])+rhs[10]*(2*params.Q[140])+rhs[11]*(2*params.Q[154])+rhs[12]*(2*params.Q[168])+rhs[13]*(2*params.Q[182]);
  lhs[1] = rhs[0]*(2*params.Q[1])+rhs[1]*(2*params.Q[15])+rhs[2]*(2*params.Q[29])+rhs[3]*(2*params.Q[43])+rhs[4]*(2*params.Q[57])+rhs[5]*(2*params.Q[71])+rhs[6]*(2*params.Q[85])+rhs[7]*(2*params.Q[99])+rhs[8]*(2*params.Q[113])+rhs[9]*(2*params.Q[127])+rhs[10]*(2*params.Q[141])+rhs[11]*(2*params.Q[155])+rhs[12]*(2*params.Q[169])+rhs[13]*(2*params.Q[183]);
  lhs[2] = rhs[0]*(2*params.Q[2])+rhs[1]*(2*params.Q[16])+rhs[2]*(2*params.Q[30])+rhs[3]*(2*params.Q[44])+rhs[4]*(2*params.Q[58])+rhs[5]*(2*params.Q[72])+rhs[6]*(2*params.Q[86])+rhs[7]*(2*params.Q[100])+rhs[8]*(2*params.Q[114])+rhs[9]*(2*params.Q[128])+rhs[10]*(2*params.Q[142])+rhs[11]*(2*params.Q[156])+rhs[12]*(2*params.Q[170])+rhs[13]*(2*params.Q[184]);
  lhs[3] = rhs[0]*(2*params.Q[3])+rhs[1]*(2*params.Q[17])+rhs[2]*(2*params.Q[31])+rhs[3]*(2*params.Q[45])+rhs[4]*(2*params.Q[59])+rhs[5]*(2*params.Q[73])+rhs[6]*(2*params.Q[87])+rhs[7]*(2*params.Q[101])+rhs[8]*(2*params.Q[115])+rhs[9]*(2*params.Q[129])+rhs[10]*(2*params.Q[143])+rhs[11]*(2*params.Q[157])+rhs[12]*(2*params.Q[171])+rhs[13]*(2*params.Q[185]);
  lhs[4] = rhs[0]*(2*params.Q[4])+rhs[1]*(2*params.Q[18])+rhs[2]*(2*params.Q[32])+rhs[3]*(2*params.Q[46])+rhs[4]*(2*params.Q[60])+rhs[5]*(2*params.Q[74])+rhs[6]*(2*params.Q[88])+rhs[7]*(2*params.Q[102])+rhs[8]*(2*params.Q[116])+rhs[9]*(2*params.Q[130])+rhs[10]*(2*params.Q[144])+rhs[11]*(2*params.Q[158])+rhs[12]*(2*params.Q[172])+rhs[13]*(2*params.Q[186]);
  lhs[5] = rhs[0]*(2*params.Q[5])+rhs[1]*(2*params.Q[19])+rhs[2]*(2*params.Q[33])+rhs[3]*(2*params.Q[47])+rhs[4]*(2*params.Q[61])+rhs[5]*(2*params.Q[75])+rhs[6]*(2*params.Q[89])+rhs[7]*(2*params.Q[103])+rhs[8]*(2*params.Q[117])+rhs[9]*(2*params.Q[131])+rhs[10]*(2*params.Q[145])+rhs[11]*(2*params.Q[159])+rhs[12]*(2*params.Q[173])+rhs[13]*(2*params.Q[187]);
  lhs[6] = rhs[0]*(2*params.Q[6])+rhs[1]*(2*params.Q[20])+rhs[2]*(2*params.Q[34])+rhs[3]*(2*params.Q[48])+rhs[4]*(2*params.Q[62])+rhs[5]*(2*params.Q[76])+rhs[6]*(2*params.Q[90])+rhs[7]*(2*params.Q[104])+rhs[8]*(2*params.Q[118])+rhs[9]*(2*params.Q[132])+rhs[10]*(2*params.Q[146])+rhs[11]*(2*params.Q[160])+rhs[12]*(2*params.Q[174])+rhs[13]*(2*params.Q[188]);
  lhs[7] = rhs[0]*(2*params.Q[7])+rhs[1]*(2*params.Q[21])+rhs[2]*(2*params.Q[35])+rhs[3]*(2*params.Q[49])+rhs[4]*(2*params.Q[63])+rhs[5]*(2*params.Q[77])+rhs[6]*(2*params.Q[91])+rhs[7]*(2*params.Q[105])+rhs[8]*(2*params.Q[119])+rhs[9]*(2*params.Q[133])+rhs[10]*(2*params.Q[147])+rhs[11]*(2*params.Q[161])+rhs[12]*(2*params.Q[175])+rhs[13]*(2*params.Q[189]);
  lhs[8] = rhs[0]*(2*params.Q[8])+rhs[1]*(2*params.Q[22])+rhs[2]*(2*params.Q[36])+rhs[3]*(2*params.Q[50])+rhs[4]*(2*params.Q[64])+rhs[5]*(2*params.Q[78])+rhs[6]*(2*params.Q[92])+rhs[7]*(2*params.Q[106])+rhs[8]*(2*params.Q[120])+rhs[9]*(2*params.Q[134])+rhs[10]*(2*params.Q[148])+rhs[11]*(2*params.Q[162])+rhs[12]*(2*params.Q[176])+rhs[13]*(2*params.Q[190]);
  lhs[9] = rhs[0]*(2*params.Q[9])+rhs[1]*(2*params.Q[23])+rhs[2]*(2*params.Q[37])+rhs[3]*(2*params.Q[51])+rhs[4]*(2*params.Q[65])+rhs[5]*(2*params.Q[79])+rhs[6]*(2*params.Q[93])+rhs[7]*(2*params.Q[107])+rhs[8]*(2*params.Q[121])+rhs[9]*(2*params.Q[135])+rhs[10]*(2*params.Q[149])+rhs[11]*(2*params.Q[163])+rhs[12]*(2*params.Q[177])+rhs[13]*(2*params.Q[191]);
  lhs[10] = rhs[0]*(2*params.Q[10])+rhs[1]*(2*params.Q[24])+rhs[2]*(2*params.Q[38])+rhs[3]*(2*params.Q[52])+rhs[4]*(2*params.Q[66])+rhs[5]*(2*params.Q[80])+rhs[6]*(2*params.Q[94])+rhs[7]*(2*params.Q[108])+rhs[8]*(2*params.Q[122])+rhs[9]*(2*params.Q[136])+rhs[10]*(2*params.Q[150])+rhs[11]*(2*params.Q[164])+rhs[12]*(2*params.Q[178])+rhs[13]*(2*params.Q[192]);
  lhs[11] = rhs[0]*(2*params.Q[11])+rhs[1]*(2*params.Q[25])+rhs[2]*(2*params.Q[39])+rhs[3]*(2*params.Q[53])+rhs[4]*(2*params.Q[67])+rhs[5]*(2*params.Q[81])+rhs[6]*(2*params.Q[95])+rhs[7]*(2*params.Q[109])+rhs[8]*(2*params.Q[123])+rhs[9]*(2*params.Q[137])+rhs[10]*(2*params.Q[151])+rhs[11]*(2*params.Q[165])+rhs[12]*(2*params.Q[179])+rhs[13]*(2*params.Q[193]);
  lhs[12] = rhs[0]*(2*params.Q[12])+rhs[1]*(2*params.Q[26])+rhs[2]*(2*params.Q[40])+rhs[3]*(2*params.Q[54])+rhs[4]*(2*params.Q[68])+rhs[5]*(2*params.Q[82])+rhs[6]*(2*params.Q[96])+rhs[7]*(2*params.Q[110])+rhs[8]*(2*params.Q[124])+rhs[9]*(2*params.Q[138])+rhs[10]*(2*params.Q[152])+rhs[11]*(2*params.Q[166])+rhs[12]*(2*params.Q[180])+rhs[13]*(2*params.Q[194]);
  lhs[13] = rhs[0]*(2*params.Q[13])+rhs[1]*(2*params.Q[27])+rhs[2]*(2*params.Q[41])+rhs[3]*(2*params.Q[55])+rhs[4]*(2*params.Q[69])+rhs[5]*(2*params.Q[83])+rhs[6]*(2*params.Q[97])+rhs[7]*(2*params.Q[111])+rhs[8]*(2*params.Q[125])+rhs[9]*(2*params.Q[139])+rhs[10]*(2*params.Q[153])+rhs[11]*(2*params.Q[167])+rhs[12]*(2*params.Q[181])+rhs[13]*(2*params.Q[195]);
}
void fillq(void) {
  work.q[0] = params.c[0];
  work.q[1] = params.c[1];
  work.q[2] = params.c[2];
  work.q[3] = params.c[3];
  work.q[4] = params.c[4];
  work.q[5] = params.c[5];
  work.q[6] = params.c[6];
  work.q[7] = params.c[7];
  work.q[8] = params.c[8];
  work.q[9] = params.c[9];
  work.q[10] = params.c[10];
  work.q[11] = params.c[11];
  work.q[12] = params.c[12];
  work.q[13] = params.c[13];
}
void fillh(void) {
  work.h[0] = 4975;
  work.h[1] = 4975;
  work.h[2] = 4975;
  work.h[3] = 4975;
  work.h[4] = 4975;
  work.h[5] = 4975;
  work.h[6] = 4975;
  work.h[7] = 4975;
  work.h[8] = 4975;
  work.h[9] = 4975;
  work.h[10] = 4975;
  work.h[11] = 4975;
  work.h[12] = 4975;
  work.h[13] = 4975;
  work.h[14] = 4975;
  work.h[15] = 4975;
}
void fillb(void) {
  work.b[0] = params.b[0];
  work.b[1] = params.b[1];
  work.b[2] = params.b[2];
  work.b[3] = params.b[3];
  work.b[4] = params.b[4];
  work.b[5] = params.b[5];
}
void pre_ops(void) {
}
