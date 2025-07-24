/*
 * File: cumtrapz.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "cumtrapz.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"
#include "stewart_control_function_V4_part_test_1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                const emxArray_real_T *y
 *                emxArray_real_T *z
 * Return Type  : void
 */
void cumtrapz(const emxArray_real_T *x, const emxArray_real_T *y,
              emxArray_real_T *z)
{
  emxArray_real_T *b_x;
  int i;
  int ix;
  double s;
  int iyz;
  double ylast;
  int k;
  emxInit_real_T(&b_x, 2);
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = x->size[1];
  emxEnsureCapacity_real_T(b_x, i);
  ix = x->size[0] * x->size[1];
  for (i = 0; i < ix; i++) {
    b_x->data[i] = x->data[i];
  }

  i = y->size[1];
  for (ix = 0; ix <= i - 2; ix++) {
    b_x->data[ix] = b_x->data[ix + 1] - b_x->data[ix];
  }

  i = z->size[0] * z->size[1];
  z->size[0] = 1;
  z->size[1] = y->size[1];
  emxEnsureCapacity_real_T(z, i);
  if (y->size[1] != 0) {
    s = 0.0;
    ix = -1;
    iyz = 0;
    ylast = y->data[0];
    z->data[0] = 0.0;
    i = y->size[1];
    for (k = 0; k <= i - 2; k++) {
      iyz++;
      if (b_x->size[1] == 0) {
        s += (ylast + y->data[iyz]) / 2.0;
      } else {
        ix++;
        s += b_x->data[ix] * ((ylast + y->data[iyz]) / 2.0);
      }

      ylast = y->data[iyz];
      z->data[iyz] = s;
    }
  }

  emxFree_real_T(&b_x);
}

/*
 * File trailer for cumtrapz.c
 *
 * [EOF]
 */
