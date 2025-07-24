/*
 * File: inv.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "inv.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"
#include <math.h>

/* Function Definitions */

/*
 * Arguments    : const double x[36]
 *                double y[36]
 * Return Type  : void
 */
void inv(const double x[36], double y[36])
{
  int i;
  double b_x[36];
  int j;
  signed char ipiv[6];
  int mmj_tmp;
  int b;
  int k;
  signed char p[6];
  int jj;
  int jp1j;
  int jy;
  int iy;
  int ix;
  double smax;
  double s;
  int i1;
  for (i = 0; i < 36; i++) {
    y[i] = 0.0;
    b_x[i] = x[i];
  }

  for (i = 0; i < 6; i++) {
    ipiv[i] = (signed char)(i + 1);
  }

  for (j = 0; j < 5; j++) {
    mmj_tmp = 4 - j;
    b = j * 7;
    jj = j * 7;
    jp1j = b + 2;
    jy = 6 - j;
    iy = 0;
    ix = b;
    smax = fabs(b_x[jj]);
    for (k = 2; k <= jy; k++) {
      ix++;
      s = fabs(b_x[ix]);
      if (s > smax) {
        iy = k - 1;
        smax = s;
      }
    }

    if (b_x[jj + iy] != 0.0) {
      if (iy != 0) {
        iy += j;
        ipiv[j] = (signed char)(iy + 1);
        ix = j;
        for (k = 0; k < 6; k++) {
          smax = b_x[ix];
          b_x[ix] = b_x[iy];
          b_x[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i = (jj - j) + 6;
      for (ix = jp1j; ix <= i; ix++) {
        b_x[ix - 1] /= b_x[jj];
      }
    }

    jy = b + 6;
    iy = jj;
    for (b = 0; b <= mmj_tmp; b++) {
      smax = b_x[jy];
      if (b_x[jy] != 0.0) {
        ix = jj + 1;
        i = iy + 8;
        i1 = (iy - j) + 12;
        for (jp1j = i; jp1j <= i1; jp1j++) {
          b_x[jp1j - 1] += b_x[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (i = 0; i < 6; i++) {
    p[i] = (signed char)(i + 1);
  }

  for (k = 0; k < 5; k++) {
    if (ipiv[k] > k + 1) {
      jy = ipiv[k] - 1;
      iy = p[jy];
      p[jy] = p[k];
      p[k] = (signed char)iy;
    }
  }

  for (k = 0; k < 6; k++) {
    b = 6 * (p[k] - 1);
    y[k + b] = 1.0;
    for (j = k + 1; j < 7; j++) {
      i = (j + b) - 1;
      if (y[i] != 0.0) {
        i1 = j + 1;
        for (ix = i1; ix < 7; ix++) {
          jy = (ix + b) - 1;
          y[jy] -= y[i] * b_x[(ix + 6 * (j - 1)) - 1];
        }
      }
    }
  }

  for (j = 0; j < 6; j++) {
    jy = 6 * j;
    for (k = 5; k >= 0; k--) {
      iy = 6 * k;
      i = k + jy;
      if (y[i] != 0.0) {
        y[i] /= b_x[k + iy];
        for (ix = 0; ix < k; ix++) {
          b = ix + jy;
          y[b] -= y[i] * b_x[ix + iy];
        }
      }
    }
  }
}

/*
 * File trailer for inv.c
 *
 * [EOF]
 */
