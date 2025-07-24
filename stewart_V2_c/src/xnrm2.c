/*
 * File: xnrm2.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"
#include <math.h>

/* Function Definitions */

/*
 * Arguments    : int n
 *                const double x[6]
 *                int ix0
 * Return Type  : double
 */
double b_xnrm2(int n, const double x[6], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double b_t;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      b_t = scale / absxk;
      y = y * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      y += b_t * b_t;
    }
  }

  return scale * sqrt(y);
}

/*
 * Arguments    : int n
 *                const double x[36]
 *                int ix0
 * Return Type  : double
 */
double xnrm2(int n, const double x[36], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double b_t;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      b_t = scale / absxk;
      y = y * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      y += b_t * b_t;
    }
  }

  return scale * sqrt(y);
}

/*
 * File trailer for xnrm2.c
 *
 * [EOF]
 */
