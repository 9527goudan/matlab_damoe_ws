/*
 * File: stewart_control_function_V4_part_test_1_initialize.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "stewart_control_function_V4_part_test_1_initialize.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"
#include "stewart_control_function_V4_part_test_1_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void stewart_control_function_V4_part_test_1_initialize(void)
{
  int i;
  static const double b_dv[6] = { 170.986, 186.426, 194.285, 188.708, 179.721,
    169.118 };

  static const short iv[6] = { 0, 0, 700, 10, 10, 0 };

  rt_InitInfAndNaN();
  flag_v = 2.0;
  for (i = 0; i < 6; i++) {
    pre_length_last[i] = b_dv[i];
    pre_Pf[i] = iv[i];
  }

  c_stewart_control_function_V4_p();
  isInitialized_stewart_control_function_V4_part_test_1 = true;
}

/*
 * File trailer for stewart_control_function_V4_part_test_1_initialize.c
 *
 * [EOF]
 */
