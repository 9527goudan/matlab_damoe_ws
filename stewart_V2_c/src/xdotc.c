/*
 * File: xdotc.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "xdotc.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                const double x[36]
 *                int ix0
 *                const double y[36]
 *                int iy0
 * Return Type  : double
 */
double xdotc(int n, const double x[36], int ix0, const double y[36], int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  ix = ix0;
  iy = iy0;
  d = 0.0;
  for (k = 0; k < n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

/*
 * File trailer for xdotc.c
 *
 * [EOF]
 */
