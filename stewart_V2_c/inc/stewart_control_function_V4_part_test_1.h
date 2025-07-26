/*
 * File: stewart_control_function_V4_part_test_1.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

#ifndef STEWART_CONTROL_FUNCTION_V4_PART_TEST_1_H
#define STEWART_CONTROL_FUNCTION_V4_PART_TEST_1_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "stewart_control_function_V4_part_test_1_types.h"

/* Function Declarations */
extern void c_stewart_control_function_V4_p(void);
extern void d_stewart_control_function_V4_p(void);

#ifdef __cplusplus
extern "C" {
   void stewart_control_function_V4_part_test_1(double *inputData, int* outputDataLenger, int *inputSize, int inputSizeLenger,
                                                const double sensor_length[6],double Pf[6], double vec[6]);
}
#endif

#endif

/*
 * File trailer for stewart_control_function_V4_part_test_1.h
 *
 * [EOF]
 */
