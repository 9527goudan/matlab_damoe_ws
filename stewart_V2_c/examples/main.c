/*
 * File: main.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "rt_nonfinite.h"
#include "stewart_control_function_V4_part_test_1.h"
#include "stewart_control_function_V4_part_test_1_terminate.h"

/* Function Declarations */
static void argInit_1x6_real_T(double result[6]);
static double argInit_real_T(void);
static void main_stewart_control_function_V4_part_test_1(void);

/* Function Definitions */

/*
 * Arguments    : double result[6]
 * Return Type  : void
 */
static void argInit_1x6_real_T(double result[6])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 6; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_stewart_control_function_V4_part_test_1(void)
{
  double sensor_length_tmp[6];
  int i;
  double b_sensor_length_tmp[6];
  double vec[6];

  /* Initialize function 'stewart_control_function_V4_part_test_1' input arguments. */
  /* Initialize function input argument 'sensor_length'. */
  argInit_1x6_real_T(sensor_length_tmp);

  /* Initialize function input argument 'Pf'. */
  /* Call the entry-point 'stewart_control_function_V4_part_test_1'. */
  for (i = 0; i < 6; i++) {
    b_sensor_length_tmp[i] = sensor_length_tmp[i];
  }

  stewart_control_function_V4_part_test_1(sensor_length_tmp, b_sensor_length_tmp,
    vec);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_stewart_control_function_V4_part_test_1();

  /* Terminate the application.
     You do not need to do this more than one time. */
  stewart_control_function_V4_part_test_1_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
