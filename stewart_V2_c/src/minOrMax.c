/*
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"

/* Function Definitions */

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
double maximum(const double x[3])
{
  double ex;
  int idx;
  int k;
  bool exitg1;
  double d;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      d = x[k - 1];
      if (ex < d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
