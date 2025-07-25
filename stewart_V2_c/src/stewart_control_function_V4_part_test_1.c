/*
 * File: stewart_control_function_V4_part_test_1.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Jul-2025 17:39:51
 */

/* Include Files */
#include "stewart_control_function_V4_part_test_1.h"
#include "cosd.h"
#include "cumtrapz.h"
#include "inv.h"
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "stewart_control_function_V4_part_test_1_data.h"
#include "stewart_control_function_V4_part_test_1_emxutil.h"
#include "stewart_control_function_V4_part_test_1_initialize.h"
#include "svd.h"
#include <math.h>
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T_1x6
#define struct_emxArray_real_T_1x6

struct emxArray_real_T_1x6
{
  double data[6];
  int size[2];
};

#endif                                 /*struct_emxArray_real_T_1x6*/

#ifndef typedef_emxArray_real_T_1x6
#define typedef_emxArray_real_T_1x6

typedef struct emxArray_real_T_1x6 emxArray_real_T_1x6;

#endif                                 /*typedef_emxArray_real_T_1x6*/

/* Variable Definitions */
static emxArray_real_T *traj;
static bool traj_not_empty;
static emxArray_real_T *vel;
static double time_idx;
static double T;
static double t;
static bool converged;
static double v0[6];
static double MovingPlat[18];
static double StaticPlat[18];
static emxArray_real_T_1x6 filtered_P0;
static bool filtered_P0_not_empty;
static double t1;
static double Pf_next[6];
static double pre_length[6];
static double flag;
static const double dv[18] = { -54.9990470842857, 205.259238086427, 0.0,
  -205.259238086427, -54.9990470842857, 0.0, -150.260191002141,
  -150.260191002141, 0.0, 150.260191002141, -150.260191002141, 0.0,
  205.259238086427, -54.9990470842857, 0.0, 54.9990470842857, 205.259238086427,
  0.0 };

static const double dv1[18] = { -160.708576946995, 244.329110714855, 0.0,
  -287.497989750303, 53.5618048293256, 0.0, -58.6956909496092, -286.493938698514,
  0.0, 58.6956909496092, -286.493938698514, 0.0, 287.497989750303,
  53.5618048293256, 0.0, 160.708576946995, 244.329110714855, 0.0 };

/* Function Declarations */
static void Calculateposevelocity(const double b_MovingPlat[18], const double
  b_StaticPlat[18], const double P0[6], const double pre_vec_actual[6], double
  v0_1[6]);
static void F(const double b_MovingPlat[18], const double b_StaticPlat[18],
              const double X[6], const double L_target[6], double F_val[6]);
static void Jocobian(const double b_MovingPlat[18], const double b_StaticPlat[18],
                     const double X[6], double J[36]);
static void PositiveSolution(const double b_MovingPlat[18], const double
  SataticPlat[18], const double L_target[6], double Xn[6], double Record[10]);
static double adjust_planning_time(const double P0[6], const double Pf[6], const
  double b_v0[6]);
static void c_velocity_shaped_trajectory_fi(const double P0[6], const double Pf
  [6], const double b_v0[6], double traj_data[], int traj_size[2], double
  vel_data[], int vel_size[2], double acc_data[], int acc_size[2]);
static void d_velocity_shaped_trajectory_fi(double P0_data[], const int P0_size
  [2], const double Pf[6], double b_T, const double b_v0[6], emxArray_real_T
  *b_traj, emxArray_real_T *b_vel, emxArray_real_T *acc);
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */

/*
 * 计算斯图尔特平台各支链的杆速
 *  输入:
 *    MovingPlat - 动平台铰链点坐标（3×n矩阵）
 *    StaticPlat - 静平台铰链点坐标（3×n矩阵）
 *    X - 当前位姿 [t; angles] (6×1向量)
 *    Vn - 平台位姿速度 [v; omega] (6×1向量)
 *  输出:
 *    l_dot - 各支链的杆速 (n×1向量)
 * Arguments    : const double b_MovingPlat[18]
 *                const double b_StaticPlat[18]
 *                const double P0[6]
 *                const double pre_vec_actual[6]
 *                double v0_1[6]
 * Return Type  : void
 */
static void Calculateposevelocity(const double b_MovingPlat[18], const double
  b_StaticPlat[18], const double P0[6], const double pre_vec_actual[6], double
  v0_1[6])
{
  double J[36];
  bool p;
  int br;
  double C[36];
  double U[36];
  double s[6];
  double V[36];
  int i;
  double absx;
  int vcol;
  int r;
  int j;
  int ar;
  int ib;
  int ia;
  int i1;
  int ic;

  /*  计算位姿速度的函数 */
  /* 计算的应该是列向量 */
  /* 计算的应该是列向量 */
  /*  计算雅可比矩阵 */
  Jocobian(b_MovingPlat, b_StaticPlat, P0, J);

  /*  计算杆速：l_dot = J' * Vn */
  /*  使用伪逆，因为J可能不是方阵 */
  p = true;
  for (br = 0; br < 36; br++) {
    C[br] = 0.0;
    if ((!p) || (rtIsInf(J[br]) || rtIsNaN(J[br]))) {
      p = false;
    }
  }

  if (!p) {
    for (i = 0; i < 36; i++) {
      C[i] = rtNaN;
    }
  } else {
    svd(J, U, s, V);
    absx = fabs(s[0]);
    if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &vcol);
        absx = ldexp(1.0, vcol - 53);
      }
    } else {
      absx = rtNaN;
    }

    absx *= 6.0;
    r = -1;
    br = 0;
    while ((br < 6) && (s[br] > absx)) {
      r++;
      br++;
    }

    if (r + 1 > 0) {
      vcol = 1;
      for (j = 0; j <= r; j++) {
        absx = 1.0 / s[j];
        i = vcol + 5;
        for (br = vcol; br <= i; br++) {
          V[br - 1] *= absx;
        }

        vcol += 6;
      }

      for (vcol = 0; vcol <= 30; vcol += 6) {
        i = vcol + 1;
        j = vcol + 6;
        if (i <= j) {
          memset(&C[i + -1], 0, ((j - i) + 1) * sizeof(double));
        }
      }

      br = 0;
      for (vcol = 0; vcol <= 30; vcol += 6) {
        ar = -1;
        br++;
        i = br + 6 * r;
        for (ib = br; ib <= i; ib += 6) {
          ia = ar;
          j = vcol + 1;
          i1 = vcol + 6;
          for (ic = j; ic <= i1; ic++) {
            ia++;
            C[ic - 1] += U[ib - 1] * V[ia];
          }

          ar += 6;
        }
      }
    }
  }

  for (i = 0; i < 6; i++) {
    absx = 0.0;
    for (j = 0; j < 6; j++) {
      absx += C[i + 6 * j] * pre_vec_actual[j];
    }

    v0_1[i] = absx;
  }
}

/*
 * 计算当前支链长度与目标长度差值向量
 * Arguments    : const double b_MovingPlat[18]
 *                const double b_StaticPlat[18]
 *                const double X[6]
 *                const double L_target[6]
 *                double F_val[6]
 * Return Type  : void
 */
static void F(const double b_MovingPlat[18], const double b_StaticPlat[18],
              const double X[6], const double L_target[6], double F_val[6])
{
  double Pt[16];
  double scale;
  double absxk;
  double b_t;
  double rotmat_tmp;
  double b_rotmat_tmp;
  double c_rotmat_tmp;
  double Rt[16];
  double d_rotmat_tmp[9];
  double e_rotmat_tmp[9];
  int k;
  int MovingPlat_Hom_tmp;
  int i;
  double f_rotmat_tmp[9];
  double MovingPlat_Hom[24];
  double b_Pt[24];
  double c_Pt[16];

  /*  InverseSolution(MovingPlat,SataticPlat,X) 并联机构运动学反解 */
  /*  MovingPlat 动平台铰链点结构参数 */
  /*  SataticPlat 静平台铰链点结构参数 */
  /*  X 为动平台相对静平台的位姿坐标 */
  /*  Result 为各支链长度 */
  /*  平移齐次变换矩阵 */
  memset(&Pt[0], 0, 16U * sizeof(double));
  Pt[0] = 1.0;
  Pt[5] = 1.0;
  Pt[10] = 1.0;
  Pt[15] = 1.0;
  Pt[12] = X[0];
  Pt[13] = X[1];
  Pt[14] = X[2];

  /* 平移齐次变换矩阵 */
  scale = X[5];
  b_sind(&scale);
  absxk = X[5];
  b_cosd(&absxk);
  b_t = X[4];
  b_sind(&b_t);
  rotmat_tmp = X[4];
  b_cosd(&rotmat_tmp);
  b_rotmat_tmp = X[3];
  b_sind(&b_rotmat_tmp);
  c_rotmat_tmp = X[3];
  b_cosd(&c_rotmat_tmp);

  /*  RPY角转齐次变换矩阵（仅旋转部分） */
  memset(&Rt[0], 0, 16U * sizeof(double));
  Rt[0] = 1.0;
  Rt[5] = 1.0;
  Rt[10] = 1.0;
  Rt[15] = 1.0;
  d_rotmat_tmp[0] = absxk;
  d_rotmat_tmp[3] = -scale;
  d_rotmat_tmp[6] = 0.0;
  d_rotmat_tmp[1] = scale;
  d_rotmat_tmp[4] = absxk;
  d_rotmat_tmp[7] = 0.0;
  e_rotmat_tmp[0] = rotmat_tmp;
  e_rotmat_tmp[3] = 0.0;
  e_rotmat_tmp[6] = b_t;
  d_rotmat_tmp[2] = 0.0;
  e_rotmat_tmp[1] = 0.0;
  d_rotmat_tmp[5] = 0.0;
  e_rotmat_tmp[4] = 1.0;
  d_rotmat_tmp[8] = 1.0;
  e_rotmat_tmp[7] = 0.0;
  e_rotmat_tmp[2] = -b_t;
  e_rotmat_tmp[5] = 0.0;
  e_rotmat_tmp[8] = rotmat_tmp;
  for (k = 0; k < 3; k++) {
    b_t = d_rotmat_tmp[k + 3];
    MovingPlat_Hom_tmp = (int)d_rotmat_tmp[k + 6];
    for (i = 0; i < 3; i++) {
      f_rotmat_tmp[k + 3 * i] = (d_rotmat_tmp[k] * e_rotmat_tmp[3 * i] + b_t *
        e_rotmat_tmp[3 * i + 1]) + (double)MovingPlat_Hom_tmp * e_rotmat_tmp[3 *
        i + 2];
    }
  }

  d_rotmat_tmp[0] = 1.0;
  d_rotmat_tmp[3] = 0.0;
  d_rotmat_tmp[6] = 0.0;
  d_rotmat_tmp[1] = 0.0;
  d_rotmat_tmp[4] = c_rotmat_tmp;
  d_rotmat_tmp[7] = -b_rotmat_tmp;
  d_rotmat_tmp[2] = 0.0;
  d_rotmat_tmp[5] = b_rotmat_tmp;
  d_rotmat_tmp[8] = c_rotmat_tmp;
  for (k = 0; k < 3; k++) {
    b_t = f_rotmat_tmp[k + 3];
    scale = f_rotmat_tmp[k + 6];
    for (MovingPlat_Hom_tmp = 0; MovingPlat_Hom_tmp < 3; MovingPlat_Hom_tmp++) {
      Rt[k + (MovingPlat_Hom_tmp << 2)] = (f_rotmat_tmp[k] * d_rotmat_tmp[3 *
        MovingPlat_Hom_tmp] + b_t * d_rotmat_tmp[3 * MovingPlat_Hom_tmp + 1]) +
        scale * d_rotmat_tmp[3 * MovingPlat_Hom_tmp + 2];
    }
  }

  /* RPY旋转齐次变换 */
  /* 初始位置 动平台相对静平台的位姿齐次变换矩阵 */
  /* 动平台铰链点变为齐次形式 */
  for (k = 0; k < 6; k++) {
    MovingPlat_Hom_tmp = k << 2;
    MovingPlat_Hom[MovingPlat_Hom_tmp] = b_MovingPlat[3 * k];
    MovingPlat_Hom[MovingPlat_Hom_tmp + 1] = b_MovingPlat[3 * k + 1];
    MovingPlat_Hom_tmp += 3;
    MovingPlat_Hom[MovingPlat_Hom_tmp - 1] = b_MovingPlat[3 * k + 2];
    MovingPlat_Hom[MovingPlat_Hom_tmp] = 1.0;
  }

  /* 实际动平台坐标为 */
  for (k = 0; k < 4; k++) {
    b_t = Pt[k + 4];
    scale = Pt[k + 8];
    absxk = Pt[k + 12];
    for (MovingPlat_Hom_tmp = 0; MovingPlat_Hom_tmp < 4; MovingPlat_Hom_tmp++) {
      i = MovingPlat_Hom_tmp << 2;
      c_Pt[k + i] = ((Pt[k] * Rt[i] + b_t * Rt[i + 1]) + scale * Rt[i + 2]) +
        absxk * Rt[i + 3];
    }

    b_t = c_Pt[k + 4];
    scale = c_Pt[k + 8];
    absxk = c_Pt[k + 12];
    for (MovingPlat_Hom_tmp = 0; MovingPlat_Hom_tmp < 6; MovingPlat_Hom_tmp++) {
      i = MovingPlat_Hom_tmp << 2;
      b_Pt[k + i] = ((c_Pt[k] * MovingPlat_Hom[i] + b_t * MovingPlat_Hom[i + 1])
                     + scale * MovingPlat_Hom[i + 2]) + absxk * MovingPlat_Hom[i
        + 3];
    }
  }

  memcpy(&MovingPlat_Hom[0], &b_Pt[0], 24U * sizeof(double));
  for (k = 0; k < 6; k++) {
    scale = 3.3121686421112381E-170;
    MovingPlat_Hom_tmp = k << 2;
    absxk = fabs(MovingPlat_Hom[MovingPlat_Hom_tmp] - b_StaticPlat[3 * k]);
    if (absxk > 3.3121686421112381E-170) {
      rotmat_tmp = 1.0;
      scale = absxk;
    } else {
      b_t = absxk / 3.3121686421112381E-170;
      rotmat_tmp = b_t * b_t;
    }

    absxk = fabs(MovingPlat_Hom[MovingPlat_Hom_tmp + 1] - b_StaticPlat[3 * k + 1]);
    if (absxk > scale) {
      b_t = scale / absxk;
      rotmat_tmp = rotmat_tmp * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      rotmat_tmp += b_t * b_t;
    }

    absxk = fabs(MovingPlat_Hom[MovingPlat_Hom_tmp + 2] - b_StaticPlat[3 * k + 2]);
    if (absxk > scale) {
      b_t = scale / absxk;
      rotmat_tmp = rotmat_tmp * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      rotmat_tmp += b_t * b_t;
    }

    F_val[k] = scale * sqrt(rotmat_tmp) - L_target[k];
  }

  /*  确保列向量 */
}

/*
 * 计算雅可比矩阵（速度雅可比）
 * Arguments    : const double b_MovingPlat[18]
 *                const double b_StaticPlat[18]
 *                const double X[6]
 *                double J[36]
 * Return Type  : void
 */
static void Jocobian(const double b_MovingPlat[18], const double b_StaticPlat[18],
                     const double X[6], double J[36])
{
  double absxk;
  double b_t;
  double rotmat_tmp;
  double b_rotmat_tmp;
  double scale;
  double c_rotmat_tmp;
  double d_rotmat_tmp[9];
  double R[9];
  int i;
  double d;
  int k;
  int b_i;
  double e_rotmat_tmp[9];
  double p_i_tmp[3];
  double l_i[3];

  /*  构造旋转矩阵 (ZYX欧拉角) */
  absxk = X[5];
  b_sind(&absxk);
  b_t = X[5];
  b_cosd(&b_t);
  rotmat_tmp = X[4];
  b_sind(&rotmat_tmp);
  b_rotmat_tmp = X[4];
  b_cosd(&b_rotmat_tmp);
  scale = X[3];
  b_sind(&scale);
  c_rotmat_tmp = X[3];
  b_cosd(&c_rotmat_tmp);
  d_rotmat_tmp[0] = b_t;
  d_rotmat_tmp[3] = -absxk;
  d_rotmat_tmp[6] = 0.0;
  d_rotmat_tmp[1] = absxk;
  d_rotmat_tmp[4] = b_t;
  d_rotmat_tmp[7] = 0.0;
  R[0] = b_rotmat_tmp;
  R[3] = 0.0;
  R[6] = rotmat_tmp;
  d_rotmat_tmp[2] = 0.0;
  R[1] = 0.0;
  d_rotmat_tmp[5] = 0.0;
  R[4] = 1.0;
  d_rotmat_tmp[8] = 1.0;
  R[7] = 0.0;
  R[2] = -rotmat_tmp;
  R[5] = 0.0;
  R[8] = b_rotmat_tmp;
  for (i = 0; i < 3; i++) {
    d = d_rotmat_tmp[i + 3];
    k = (int)d_rotmat_tmp[i + 6];
    for (b_i = 0; b_i < 3; b_i++) {
      e_rotmat_tmp[i + 3 * b_i] = (d_rotmat_tmp[i] * R[3 * b_i] + d * R[3 * b_i
        + 1]) + (double)k * R[3 * b_i + 2];
    }
  }

  d_rotmat_tmp[0] = 1.0;
  d_rotmat_tmp[3] = 0.0;
  d_rotmat_tmp[6] = 0.0;
  d_rotmat_tmp[1] = 0.0;
  d_rotmat_tmp[4] = c_rotmat_tmp;
  d_rotmat_tmp[7] = -scale;
  d_rotmat_tmp[2] = 0.0;
  d_rotmat_tmp[5] = scale;
  d_rotmat_tmp[8] = c_rotmat_tmp;
  for (i = 0; i < 3; i++) {
    d = e_rotmat_tmp[i + 3];
    rotmat_tmp = e_rotmat_tmp[i + 6];
    for (k = 0; k < 3; k++) {
      R[i + 3 * k] = (e_rotmat_tmp[i] * d_rotmat_tmp[3 * k] + d * d_rotmat_tmp[3
                      * k + 1]) + rotmat_tmp * d_rotmat_tmp[3 * k + 2];
    }
  }

  for (i = 0; i < 6; i++) {
    /*  动平台铰链点局部坐标 */
    /*  静平台铰链点全局坐标 */
    /*  动平台铰链点全局坐标 */
    /*  支链向量 */
    /*  单位方向向量 */
    b_rotmat_tmp = 0.0;
    scale = 3.3121686421112381E-170;
    d = b_MovingPlat[3 * i + 2];
    rotmat_tmp = b_MovingPlat[3 * i];
    c_rotmat_tmp = b_MovingPlat[3 * i + 1];
    for (k = 0; k < 3; k++) {
      absxk = (R[k] * rotmat_tmp + R[k + 3] * c_rotmat_tmp) + R[k + 6] * d;
      p_i_tmp[k] = absxk;
      absxk = (absxk + X[k]) - b_StaticPlat[k + 3 * i];
      l_i[k] = absxk;
      absxk = fabs(absxk);
      if (absxk > scale) {
        b_t = scale / absxk;
        b_rotmat_tmp = b_rotmat_tmp * b_t * b_t + 1.0;
        scale = absxk;
      } else {
        b_t = absxk / scale;
        b_rotmat_tmp += b_t * b_t;
      }
    }

    b_rotmat_tmp = scale * sqrt(b_rotmat_tmp);

    /*  平移部分雅可比（方向向量） */
    d = l_i[0] / b_rotmat_tmp;
    l_i[0] = d;
    J[i] = d;
    d = l_i[1] / b_rotmat_tmp;
    l_i[1] = d;
    J[i + 6] = d;
    d = l_i[2] / b_rotmat_tmp;
    J[i + 12] = d;

    /*  旋转部分雅可比（方向向量叉乘动点位置） */
    J[i + 18] = p_i_tmp[1] * d - p_i_tmp[2] * l_i[1];
    J[i + 24] = p_i_tmp[2] * l_i[0] - p_i_tmp[0] * d;
    J[i + 30] = p_i_tmp[0] * l_i[1] - p_i_tmp[1] * l_i[0];
  }
}

/*
 * UNTITLED7 此处显示有关此函数的摘要
 *  MovingPlat 动平台铰链点结构参数
 *  SataticPlat 静平台铰链点结构参数
 *  X0 初始位姿
 *  L_target 目标杆长
 *  Xn 求解得到的位姿
 *  Record 记录迭代过程相关信息
 * Arguments    : const double b_MovingPlat[18]
 *                const double SataticPlat[18]
 *                const double L_target[6]
 *                double Xn[6]
 *                double Record[10]
 * Return Type  : void
 */
static void PositiveSolution(const double b_MovingPlat[18], const double
  SataticPlat[18], const double L_target[6], double Xn[6], double Record[10])
{
  int k;
  int MovingPlat_Hom_tmp;
  int i;
  double MovingPlat_Hom[24];
  double a[24];
  short i1;
  static const short b_a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 710,
    1 };

  short i2;
  short i3;
  double scale;
  double absxk;
  double b_t;
  double Ferr;
  int n;
  static const short X0[6] = { 0, 0, 710, 0, 0, 0 };

  double Jn[36];
  double Ln[6];
  double length[6];
  double Deta_L[6];
  double b_dv[36];
  double Fi_delta[6];
  int NewtonNum;
  bool exitg1;
  double Fi[6];
  static const signed char e[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  /*  下面函数是求解正解 */
  /*  函数 */
  /*  InverseSolution(MovingPlat,SataticPlat,X) 并联机构运动学反解 */
  /*  MovingPlat 动平台铰链点结构参数 */
  /*  SataticPlat 静平台铰链点结构参数 */
  /*  X 为动平台相对静平台的位姿坐标 */
  /*  Result 为各支链长度 */
  /* 平移齐次变换矩阵 */
  /* RPY旋转齐次变换 */
  /* 初始位置 动平台相对静平台的位姿齐次变换矩阵 */
  /* 动平台铰链点变为齐次形式 */
  for (k = 0; k < 6; k++) {
    MovingPlat_Hom_tmp = k << 2;
    MovingPlat_Hom[MovingPlat_Hom_tmp] = b_MovingPlat[3 * k];
    MovingPlat_Hom[MovingPlat_Hom_tmp + 1] = b_MovingPlat[3 * k + 1];
    MovingPlat_Hom_tmp += 3;
    MovingPlat_Hom[MovingPlat_Hom_tmp - 1] = b_MovingPlat[3 * k + 2];
    MovingPlat_Hom[MovingPlat_Hom_tmp] = 1.0;
  }

  /* 实际动平台坐标为 */
  for (i = 0; i < 4; i++) {
    i1 = b_a[i + 4];
    i2 = b_a[i + 8];
    i3 = b_a[i + 12];
    for (k = 0; k < 6; k++) {
      MovingPlat_Hom_tmp = k << 2;
      a[i + MovingPlat_Hom_tmp] = (((double)b_a[i] *
        MovingPlat_Hom[MovingPlat_Hom_tmp] + (double)i1 *
        MovingPlat_Hom[MovingPlat_Hom_tmp + 1]) + (double)i2 *
        MovingPlat_Hom[MovingPlat_Hom_tmp + 2]) + (double)i3 *
        MovingPlat_Hom[MovingPlat_Hom_tmp + 3];
    }
  }

  memcpy(&MovingPlat_Hom[0], &a[0], 24U * sizeof(double));

  /*  用到的参数 */
  /* t是0~1的数 */
  for (k = 0; k < 6; k++) {
    scale = 3.3121686421112381E-170;
    MovingPlat_Hom_tmp = k << 2;
    absxk = fabs(MovingPlat_Hom[MovingPlat_Hom_tmp] - SataticPlat[3 * k]);
    if (absxk > 3.3121686421112381E-170) {
      Ferr = 1.0;
      scale = absxk;
    } else {
      b_t = absxk / 3.3121686421112381E-170;
      Ferr = b_t * b_t;
    }

    absxk = fabs(MovingPlat_Hom[MovingPlat_Hom_tmp + 1] - SataticPlat[3 * k + 1]);
    if (absxk > scale) {
      b_t = scale / absxk;
      Ferr = Ferr * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      Ferr += b_t * b_t;
    }

    absxk = fabs(MovingPlat_Hom[MovingPlat_Hom_tmp + 2] - SataticPlat[3 * k + 2]);
    if (absxk > scale) {
      b_t = scale / absxk;
      Ferr = Ferr * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      Ferr += b_t * b_t;
    }

    Ferr = scale * sqrt(Ferr);
    length[k] = Ferr;
    Deta_L[k] = (L_target[k] - Ferr) * 0.2;
  }

  /* 函数F的阈值 */
  /* 求偏导时的微小变动量 */
  memset(&Record[0], 0, 10U * sizeof(double));

  /* 第一列为逼近次序，第二列为牛顿迭代次数 */
  /* 赋初值 */
  for (k = 0; k < 6; k++) {
    Xn[k] = X0[k];
  }

  for (n = 0; n < 5; n++) {
    for (k = 0; k < 6; k++) {
      Ln[k] = length[k] + ((double)n + 1.0) * Deta_L[k];
    }

    Jocobian(b_MovingPlat, SataticPlat, Xn, Jn);
    inv(Jn, b_dv);
    for (i = 0; i < 6; i++) {
      Ferr = 0.0;
      for (k = 0; k < 6; k++) {
        Ferr += b_dv[i + 6 * k] * Deta_L[k];
      }

      Xn[i] += Ferr;
    }

    /* 欧拉法 */
    /* Xn=Xn+inv(Jn,1e-5)*Deta_L; */
    /* 误差 */
    F(b_MovingPlat, SataticPlat, Xn, Ln, Fi_delta);
    Ferr = 0.0;
    scale = 3.3121686421112381E-170;
    for (k = 0; k < 6; k++) {
      absxk = fabs(Fi_delta[k]);
      if (absxk > scale) {
        b_t = scale / absxk;
        Ferr = Ferr * b_t * b_t + 1.0;
        scale = absxk;
      } else {
        b_t = absxk / scale;
        Ferr += b_t * b_t;
      }
    }

    Ferr = scale * sqrt(Ferr);

    /* 牛顿迭代进行逼近 校正 */
    NewtonNum = 0;

    /* 牛顿迭代次数 */
    exitg1 = false;
    while ((!exitg1) && (Ferr > 1.0E-6)) {
      /* dFdX(MPlat,SPlat,X,L,delta) F对X的偏导 */
      for (MovingPlat_Hom_tmp = 0; MovingPlat_Hom_tmp < 6; MovingPlat_Hom_tmp++)
      {
        for (i = 0; i < 6; i++) {
          Fi[i] = Xn[i] + 1.0E-10 * (double)e[i + 6 * MovingPlat_Hom_tmp];
        }

        F(b_MovingPlat, SataticPlat, Fi, Ln, Fi_delta);
        F(b_MovingPlat, SataticPlat, Xn, Ln, Fi);
        for (i = 0; i < 6; i++) {
          Jn[i + 6 * MovingPlat_Hom_tmp] = (Fi_delta[i] - Fi[i]) / 1.0E-10;
        }
      }

      /* 偏导 */
      F(b_MovingPlat, SataticPlat, Xn, Ln, Fi_delta);

      /*          FXL=F(MovingPlat,SataticPlat,Xn,Ln);%可以用pinv() */
      inv(Jn, b_dv);
      for (i = 0; i < 6; i++) {
        Ferr = 0.0;
        for (k = 0; k < 6; k++) {
          Ferr += b_dv[i + 6 * k] * Fi_delta[k];
        }

        Xn[i] -= Ferr;
      }

      /* 可以用pinv() */
      /*          Xn=Xn-dF_dX/FXL;%可以用pinv() */
      F(b_MovingPlat, SataticPlat, Xn, Ln, Fi_delta);
      Ferr = 0.0;
      scale = 3.3121686421112381E-170;
      for (k = 0; k < 6; k++) {
        absxk = fabs(Fi_delta[k]);
        if (absxk > scale) {
          b_t = scale / absxk;
          Ferr = Ferr * b_t * b_t + 1.0;
          scale = absxk;
        } else {
          b_t = absxk / scale;
          Ferr += b_t * b_t;
        }
      }

      Ferr = scale * sqrt(Ferr);
      NewtonNum++;
      if (NewtonNum > 30) {
        /* fprintf('第%d次近似时，牛顿迭代不收敛\n',n) */
        exitg1 = true;
      } else {
        Record[n] = (double)n + 1.0;
        Record[n + 5] = NewtonNum;
      }
    }
  }
}

/*
 * 调整轨迹规划时间以满足支链速度限制
 *  输入参数：
 *    P0 - 初始位姿 [x,y,z,roll,pitch,yaw]
 *    Pf - 目标位姿 [x,y,z,roll,pitch,yaw]
 *    T_original - 原始规划时间(s)
 *    dt - 时间步长(s)
 *    velocity_limit - 支链速度限制(mm/s)
 *  输出：
 *    T_new - 调整后的规划时间(s)
 * Arguments    : const double P0[6]
 *                const double Pf[6]
 *                const double b_v0[6]
 * Return Type  : double
 */
static double adjust_planning_time(const double P0[6], const double Pf[6], const
  double b_v0[6])
{
  double T_new;
  emxArray_real_T *y;
  double traj1_data[366];
  int traj1_size[2];
  double vel1_data[366];
  int vel1_size[2];
  double unusedU4_data[366];
  int unusedU4_size[2];
  int i;
  int k;
  double kd;
  int vec1_size_idx_1;
  int nx;
  double vec1_data[366];
  int varargin_1_size_idx_0;
  bool exitg1;
  double max_leg_vel;
  double traj1[6];
  double J[36];
  emxInit_real_T(&y, 2);

  /*  下面函数用于调整规划的总时间 */
  /*     %% 第一次轨迹规划 */
  /*      [traj1, vel1, ~] = plan_trajectory(P0, Pf, T_original); */
  c_velocity_shaped_trajectory_fi(P0, Pf, b_v0, traj1_data, traj1_size,
    vel1_data, vel1_size, unusedU4_data, unusedU4_size);
  i = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = 61;
  emxEnsureCapacity_real_T(y, i);
  for (k = 0; k < 29; k++) {
    kd = ((double)k + 1.0) * 0.05;
    y->data[k + 1] = kd;
    y->data[59 - k] = 3.0 - kd;
  }

  /*     %% 计算支链速度 */
  vec1_size_idx_1 = y->size[1];
  nx = 6 * y->size[1];
  if (0 <= nx - 1) {
    memset(&vec1_data[0], 0, nx * sizeof(double));
  }

  i = y->size[1];
  emxFree_real_T(&y);
  for (nx = 0; nx < i; nx++) {
    /*  需要确保以下函数和变量在作用域内可用： */
    /*  MovingPlat, StaticPlat, calculateLegVelocities */
    /*  计算斯图尔特平台各支链的杆速 */
    /*  输入: */
    /*    MovingPlat - 动平台铰链点坐标（3×n矩阵） */
    /*    StaticPlat - 静平台铰链点坐标（3×n矩阵） */
    /*    X - 当前位姿 [t; angles] (6×1向量) */
    /*    Vn - 平台位姿速度 [v; omega] (6×1向量) */
    /*  输出: */
    /*    l_dot - 各支链的杆速 (n×1向量) */
    /*  计算杆速的函数 */
    /*  计算雅可比矩阵 */
    for (varargin_1_size_idx_0 = 0; varargin_1_size_idx_0 < 6;
         varargin_1_size_idx_0++) {
      traj1[varargin_1_size_idx_0] = traj1_data[nx + traj1_size[0] *
        varargin_1_size_idx_0];
    }

    Jocobian(dv, dv1, traj1, J);

    /*  计算杆速：l_dot = J' * Vn */
    for (varargin_1_size_idx_0 = 0; varargin_1_size_idx_0 < 6;
         varargin_1_size_idx_0++) {
      kd = 0.0;
      for (k = 0; k < 6; k++) {
        kd += J[varargin_1_size_idx_0 + 6 * k] * vel1_data[nx + vel1_size[0] * k];
      }

      vec1_data[varargin_1_size_idx_0 + 6 * nx] = kd;
    }
  }

  /*     %% 速度限制分析 */
  nx = 6 * vec1_size_idx_1;
  varargin_1_size_idx_0 = 6 * vec1_size_idx_1;
  for (k = 0; k < nx; k++) {
    unusedU4_data[k] = fabs(vec1_data[k]);
  }

  if (!rtIsNaN(unusedU4_data[0])) {
    nx = 1;
  } else {
    nx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= varargin_1_size_idx_0)) {
      if (!rtIsNaN(unusedU4_data[k - 1])) {
        nx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (nx == 0) {
    max_leg_vel = unusedU4_data[0];
  } else {
    max_leg_vel = unusedU4_data[nx - 1];
    i = nx + 1;
    for (k = i; k <= varargin_1_size_idx_0; k++) {
      kd = unusedU4_data[k - 1];
      if (max_leg_vel < kd) {
        max_leg_vel = kd;
      }
    }
  }

  /*     %% 时间调整逻辑 */
  if (max_leg_vel > 14.0) {
    /*  超速情况：增大时间以降低速度 */
    T_new = 3.0 * (max_leg_vel / 14.0);

    /* disp(['超速调整: 速度超过限制，调整时间至 ', num2str(T_new), ' 秒']); */
  } else {
    /*  低速情况：减小时间以提高效率 */
    T_new = 3.0 / (14.0 / max_leg_vel);

    /* disp(['欠速调整: 速度低于限制，调整时间至 ', num2str(T_new), ' 秒']); */
  }

  /*     %% 验证输出 */
  /* if T_new < 0 */
  /*    error('计算得到负时间，请检查输入参数'); */
  /* end */
  return T_new;
}

/*
 * 固定终点的S型速度插值轨迹生成器
 *  P0, Pf: 6x1 初末姿态
 *  v0, vf: 6x1 初末速度
 *  T, dt: 时间
 * Arguments    : const double P0[6]
 *                const double Pf[6]
 *                const double b_v0[6]
 *                double traj_data[]
 *                int traj_size[2]
 *                double vel_data[]
 *                int vel_size[2]
 *                double acc_data[]
 *                int acc_size[2]
 * Return Type  : void
 */
static void c_velocity_shaped_trajectory_fi(const double P0[6], const double Pf
  [6], const double b_v0[6], double traj_data[], int traj_size[2], double
  vel_data[], int vel_size[2], double acc_data[], int acc_size[2])
{
  emxArray_real_T *y;
  int i;
  int k;
  double kd;
  int nx;
  emxArray_real_T *a;
  emxArray_real_T *b_tmp;
  emxArray_real_T *b_b_tmp;
  emxArray_real_T *b_y;
  int s_size_idx_1;
  double s_data[61];
  double ds_data[61];
  int loop_ub;
  int b_loop_ub;
  int b_i;
  int vi_size[2];
  double vi_data[61];
  emxArray_real_T b_vi_data;
  int delta_raw_size_idx_1;
  int c_loop_ub;
  double delta_raw_data[61];
  emxInit_real_T(&y, 2);

  /*  辅助函数保持不变 (这部分轨迹规划的速度开始大，整体小，这样的话缩放时间，会使时间变得非常长，从而单片机内存不够用) */
  /*  function [traj, vel, acc] = plan_trajectory(P0, Pf, T) */
  /*  % 改进型指数轨迹规划（带可调时间常数） */
  /*  % 输入: */
  /*  %   P0: 初始位姿 [x0,y0,z0,roll0,pitch0,yaw0] */
  /*  %   Pf: 目标位姿 [xf,yf,zf,rollf,pitchf,yawf] */
  /*  %   T: 总时间(s) */
  /*  %   tau: 时间常数调节因子（默认=5，值越大初始越快，衰减越慢） */
  /*  % 输出: */
  /*  %   traj: 轨迹矩阵 (n×6) */
  /*  %   vel: 速度矩阵 (n×6) */
  /*  %   acc: 加速度矩阵 (n×6) */
  /*   */
  /*  %     if nargin < 4  % 默认参数处理 */
  /*  %         tau = 5;    % 保持原有特性 */
  /*  %     end */
  /*   */
  /*  %     tau = 5;    % 保持原有特性 */
  /*      tau = 3;    % 保持原有特性 */
  /*       */
  /*      dt = 0.01; */
  /*      t = 0:dt:T; */
  /*      n = length(t); */
  /*      traj = zeros(n,6); */
  /*      vel = zeros(n,6); */
  /*      acc = zeros(n,6); */
  /*   */
  /*      for i = 1:6 */
  /*          delta = Pf(i) - P0(i); */
  /*          if abs(delta) < 1e-6  % 避免除零错误 */
  /*              continue; */
  /*          end */
  /*           */
  /*          % 改进型指数轨迹（带时间常数调节） */
  /*          scale = t / (T/tau);  % 时间尺度变换 */
  /*          traj(:,i) = P0(i) + delta * (1 - exp(-scale)); */
  /*           */
  /*          % 速度计算（一阶导数） */
  /*          vel(:,i) = delta * (tau/T) * exp(-scale); */
  /*           */
  /*          % 加速度计算（二阶导数） */
  /*          acc(:,i) = -delta * (tau/T)^2 * exp(-scale); */
  /*      end */
  /*  end */
  /*  辅助函数保持不变 (这部分改进了轨迹规划的速度开始大，整体小，这样的话缩放时间，会使时间变得非常长，从而单片机内存不够用) */
  /* 末速度 */
  i = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = 61;
  emxEnsureCapacity_real_T(y, i);
  y->data[0] = 0.0;
  y->data[60] = 3.0;
  for (k = 0; k < 29; k++) {
    kd = ((double)k + 1.0) * 0.05;
    y->data[k + 1] = kd;
    y->data[59 - k] = 3.0 - kd;
  }

  y->data[30] = 1.5;
  traj_size[0] = y->size[1];
  traj_size[1] = 6;
  nx = y->size[1] * 6;
  if (0 <= nx - 1) {
    memset(&traj_data[0], 0, nx * sizeof(double));
  }

  vel_size[0] = y->size[1];
  vel_size[1] = 6;
  nx = y->size[1] * 6;
  if (0 <= nx - 1) {
    memset(&vel_data[0], 0, nx * sizeof(double));
  }

  acc_size[0] = y->size[1];
  acc_size[1] = 6;
  nx = y->size[1] * 6;
  if (0 <= nx - 1) {
    memset(&acc_data[0], 0, nx * sizeof(double));
  }

  emxInit_real_T(&a, 2);

  /*  S型插值函数 */
  i = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = y->size[1];
  emxEnsureCapacity_real_T(a, i);
  nx = y->size[0] * y->size[1];
  for (i = 0; i < nx; i++) {
    a->data[i] = y->data[i] / 3.0;
  }

  emxInit_real_T(&b_tmp, 2);
  i = b_tmp->size[0] * b_tmp->size[1];
  b_tmp->size[0] = 1;
  b_tmp->size[1] = (signed char)a->size[1];
  emxEnsureCapacity_real_T(b_tmp, i);
  nx = (signed char)a->size[1];
  for (k = 0; k < nx; k++) {
    b_tmp->data[k] = rt_powd_snf(a->data[k], 3.0);
  }

  i = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = y->size[1];
  emxEnsureCapacity_real_T(a, i);
  nx = y->size[0] * y->size[1];
  for (i = 0; i < nx; i++) {
    a->data[i] = y->data[i] / 3.0;
  }

  emxInit_real_T(&b_b_tmp, 2);
  i = b_b_tmp->size[0] * b_b_tmp->size[1];
  b_b_tmp->size[0] = 1;
  b_b_tmp->size[1] = (signed char)a->size[1];
  emxEnsureCapacity_real_T(b_b_tmp, i);
  nx = (signed char)a->size[1];
  for (k = 0; k < nx; k++) {
    b_b_tmp->data[k] = rt_powd_snf(a->data[k], 4.0);
  }

  emxInit_real_T(&b_y, 2);
  i = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = y->size[1];
  emxEnsureCapacity_real_T(a, i);
  nx = y->size[0] * y->size[1];
  for (i = 0; i < nx; i++) {
    a->data[i] = y->data[i] / 3.0;
  }

  i = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = (signed char)a->size[1];
  emxEnsureCapacity_real_T(b_y, i);
  nx = (signed char)a->size[1];
  for (k = 0; k < nx; k++) {
    b_y->data[k] = rt_powd_snf(a->data[k], 5.0);
  }

  s_size_idx_1 = b_tmp->size[1];
  nx = b_tmp->size[0] * b_tmp->size[1];
  for (i = 0; i < nx; i++) {
    s_data[i] = (10.0 * b_tmp->data[i] - 15.0 * b_b_tmp->data[i]) + 6.0 *
      b_y->data[i];
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  i = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = y->size[1];
  emxEnsureCapacity_real_T(a, i);
  nx = y->size[0] * y->size[1];
  for (i = 0; i < nx; i++) {
    a->data[i] = y->data[i] / 3.0;
  }

  i = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = (signed char)a->size[1];
  emxEnsureCapacity_real_T(b_y, i);
  nx = (signed char)a->size[1];
  for (k = 0; k < nx; k++) {
    kd = a->data[k];
    b_y->data[k] = kd * kd;
  }

  k = b_y->size[1];
  nx = b_y->size[0] * b_y->size[1];
  for (i = 0; i < nx; i++) {
    ds_data[i] = ((30.0 * b_y->data[i] - 60.0 * b_tmp->data[i]) + 30.0 *
                  b_b_tmp->data[i]) / 3.0;
  }

  emxFree_real_T(&b_y);
  emxFree_real_T(&b_b_tmp);
  emxFree_real_T(&b_tmp);
  nx = traj_size[0];
  loop_ub = vel_size[0];
  b_loop_ub = acc_size[0];
  for (b_i = 0; b_i < 6; b_i++) {
    /*  原始速度曲线 */
    vi_size[0] = 1;
    vi_size[1] = s_size_idx_1;
    kd = b_v0[b_i];
    for (i = 0; i < s_size_idx_1; i++) {
      vi_data[i] = kd + (0.0 - kd) * s_data[i];
    }

    /*  原始积分位移 */
    b_vi_data.data = &vi_data[0];
    b_vi_data.size = &vi_size[0];
    b_vi_data.allocatedSize = 61;
    b_vi_data.numDimensions = 2;
    b_vi_data.canFreeData = false;
    cumtrapz(y, &b_vi_data, a);
    delta_raw_size_idx_1 = a->size[1];
    c_loop_ub = a->size[0] * a->size[1];
    for (i = 0; i < c_loop_ub; i++) {
      delta_raw_data[i] = a->data[i];
    }

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 */
    kd = (Pf[b_i] - P0[b_i]) / delta_raw_data[delta_raw_size_idx_1 - 1];

    /*  缩放后的轨迹与速度、加速度 */
    for (i = 0; i < nx; i++) {
      traj_data[i + traj_size[0] * b_i] = P0[b_i] + kd * delta_raw_data[i];
    }

    i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = vi_size[1];
    emxEnsureCapacity_real_T(a, i);
    c_loop_ub = vi_size[0] * vi_size[1];
    for (i = 0; i < c_loop_ub; i++) {
      a->data[i] = kd * vi_data[i];
    }

    for (i = 0; i < loop_ub; i++) {
      vel_data[i + vel_size[0] * b_i] = a->data[i];
    }

    kd *= 0.0 - b_v0[b_i];
    i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = k;
    emxEnsureCapacity_real_T(a, i);
    for (i = 0; i < k; i++) {
      a->data[i] = kd * ds_data[i];
    }

    for (i = 0; i < b_loop_ub; i++) {
      acc_data[i + acc_size[0] * b_i] = a->data[i];
    }
  }

  emxFree_real_T(&a);
  emxFree_real_T(&y);
}

/*
 * 固定终点的S型速度插值轨迹生成器
 *  P0, Pf: 6x1 初末姿态
 *  v0, vf: 6x1 初末速度
 *  T, dt: 时间
 * Arguments    : double P0_data[]
 *                const int P0_size[2]
 *                const double Pf[6]
 *                double b_T
 *                const double b_v0[6]
 *                emxArray_real_T *b_traj
 *                emxArray_real_T *b_vel
 *                emxArray_real_T *acc
 * Return Type  : void
 */
static void d_velocity_shaped_trajectory_fi(double P0_data[], const int P0_size
  [2], const double Pf[6], double b_T, const double b_v0[6], emxArray_real_T
  *b_traj, emxArray_real_T *b_vel, emxArray_real_T *acc)
{
  int nm1d2;
  int i;
  int loop_ub;
  int k;
  double b_P0_data[36];
  emxArray_real_T *b_t;
  double ndbl;
  double apnd;
  double cdiff;
  emxArray_real_T *vi;
  emxArray_real_T *delta_raw;
  emxArray_real_T *b_tmp;
  emxArray_real_T *s;
  emxArray_real_T *ds;

  /*  辅助函数保持不变 (这部分轨迹规划的速度开始大，整体小，这样的话缩放时间，会使时间变得非常长，从而单片机内存不够用) */
  /*  function [traj, vel, acc] = plan_trajectory(P0, Pf, T) */
  /*  % 改进型指数轨迹规划（带可调时间常数） */
  /*  % 输入: */
  /*  %   P0: 初始位姿 [x0,y0,z0,roll0,pitch0,yaw0] */
  /*  %   Pf: 目标位姿 [xf,yf,zf,rollf,pitchf,yawf] */
  /*  %   T: 总时间(s) */
  /*  %   tau: 时间常数调节因子（默认=5，值越大初始越快，衰减越慢） */
  /*  % 输出: */
  /*  %   traj: 轨迹矩阵 (n×6) */
  /*  %   vel: 速度矩阵 (n×6) */
  /*  %   acc: 加速度矩阵 (n×6) */
  /*   */
  /*  %     if nargin < 4  % 默认参数处理 */
  /*  %         tau = 5;    % 保持原有特性 */
  /*  %     end */
  /*   */
  /*  %     tau = 5;    % 保持原有特性 */
  /*      tau = 3;    % 保持原有特性 */
  /*       */
  /*      dt = 0.01; */
  /*      t = 0:dt:T; */
  /*      n = length(t); */
  /*      traj = zeros(n,6); */
  /*      vel = zeros(n,6); */
  /*      acc = zeros(n,6); */
  /*   */
  /*      for i = 1:6 */
  /*          delta = Pf(i) - P0(i); */
  /*          if abs(delta) < 1e-6  % 避免除零错误 */
  /*              continue; */
  /*          end */
  /*           */
  /*          % 改进型指数轨迹（带时间常数调节） */
  /*          scale = t / (T/tau);  % 时间尺度变换 */
  /*          traj(:,i) = P0(i) + delta * (1 - exp(-scale)); */
  /*           */
  /*          % 速度计算（一阶导数） */
  /*          vel(:,i) = delta * (tau/T) * exp(-scale); */
  /*           */
  /*          % 加速度计算（二阶导数） */
  /*          acc(:,i) = -delta * (tau/T)^2 * exp(-scale); */
  /*      end */
  /*  end */
  /*  辅助函数保持不变 (这部分改进了轨迹规划的速度开始大，整体小，这样的话缩放时间，会使时间变得非常长，从而单片机内存不够用) */
  /* 末速度 */
  nm1d2 = P0_size[0];
  for (i = 0; i < nm1d2; i++) {
    loop_ub = P0_size[1];
    for (k = 0; k < loop_ub; k++) {
      b_P0_data[k + P0_size[1] * i] = P0_data[i + P0_size[0] * k];
    }
  }

  nm1d2 = P0_size[1] * P0_size[0];
  if (0 <= nm1d2 - 1) {
    memcpy(&P0_data[0], &b_P0_data[0], nm1d2 * sizeof(double));
  }

  emxInit_real_T(&b_t, 2);
  if (rtIsNaN(b_T)) {
    i = b_t->size[0] * b_t->size[1];
    b_t->size[0] = 1;
    b_t->size[1] = 1;
    emxEnsureCapacity_real_T(b_t, i);
    b_t->data[0] = rtNaN;
  } else if (b_T < 0.0) {
    b_t->size[0] = 1;
    b_t->size[1] = 0;
  } else if (rtIsInf(b_T) && (0.0 == b_T)) {
    i = b_t->size[0] * b_t->size[1];
    b_t->size[0] = 1;
    b_t->size[1] = 1;
    emxEnsureCapacity_real_T(b_t, i);
    b_t->data[0] = rtNaN;
  } else {
    ndbl = floor(b_T / 0.05 + 0.5);
    apnd = ndbl * 0.05;
    cdiff = apnd - b_T;
    if (fabs(cdiff) < 4.4408920985006262E-16 * b_T) {
      ndbl++;
      apnd = b_T;
    } else if (cdiff > 0.0) {
      apnd = (ndbl - 1.0) * 0.05;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      loop_ub = (int)ndbl;
    } else {
      loop_ub = 0;
    }

    i = b_t->size[0] * b_t->size[1];
    b_t->size[0] = 1;
    b_t->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b_t, i);
    if (loop_ub > 0) {
      b_t->data[0] = 0.0;
      if (loop_ub > 1) {
        b_t->data[loop_ub - 1] = apnd;
        nm1d2 = (loop_ub - 1) / 2;
        for (k = 0; k <= nm1d2 - 2; k++) {
          ndbl = ((double)k + 1.0) * 0.05;
          b_t->data[k + 1] = ndbl;
          b_t->data[(loop_ub - k) - 2] = apnd - ndbl;
        }

        if (nm1d2 << 1 == loop_ub - 1) {
          b_t->data[nm1d2] = apnd / 2.0;
        } else {
          ndbl = (double)nm1d2 * 0.05;
          b_t->data[nm1d2] = ndbl;
          b_t->data[nm1d2 + 1] = apnd - ndbl;
        }
      }
    }
  }

  i = b_traj->size[0] * b_traj->size[1];
  b_traj->size[0] = b_t->size[1];
  b_traj->size[1] = 6;
  emxEnsureCapacity_real_T(b_traj, i);
  nm1d2 = b_t->size[1] * 6;
  for (i = 0; i < nm1d2; i++) {
    b_traj->data[i] = 0.0;
  }

  i = b_vel->size[0] * b_vel->size[1];
  b_vel->size[0] = b_t->size[1];
  b_vel->size[1] = 6;
  emxEnsureCapacity_real_T(b_vel, i);
  nm1d2 = b_t->size[1] * 6;
  for (i = 0; i < nm1d2; i++) {
    b_vel->data[i] = 0.0;
  }

  i = acc->size[0] * acc->size[1];
  acc->size[0] = b_t->size[1];
  acc->size[1] = 6;
  emxEnsureCapacity_real_T(acc, i);
  nm1d2 = b_t->size[1] * 6;
  for (i = 0; i < nm1d2; i++) {
    acc->data[i] = 0.0;
  }

  emxInit_real_T(&vi, 2);

  /*  S型插值函数 */
  i = vi->size[0] * vi->size[1];
  vi->size[0] = 1;
  vi->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(vi, i);
  nm1d2 = b_t->size[0] * b_t->size[1];
  for (i = 0; i < nm1d2; i++) {
    vi->data[i] = b_t->data[i] / b_T;
  }

  emxInit_real_T(&delta_raw, 2);
  i = delta_raw->size[0] * delta_raw->size[1];
  delta_raw->size[0] = 1;
  delta_raw->size[1] = vi->size[1];
  emxEnsureCapacity_real_T(delta_raw, i);
  nm1d2 = vi->size[1];
  for (k = 0; k < nm1d2; k++) {
    delta_raw->data[k] = rt_powd_snf(vi->data[k], 3.0);
  }

  emxInit_real_T(&b_tmp, 2);
  i = b_tmp->size[0] * b_tmp->size[1];
  b_tmp->size[0] = 1;
  b_tmp->size[1] = vi->size[1];
  emxEnsureCapacity_real_T(b_tmp, i);
  nm1d2 = vi->size[1];
  for (k = 0; k < nm1d2; k++) {
    b_tmp->data[k] = rt_powd_snf(vi->data[k], 4.0);
  }

  emxInit_real_T(&s, 2);
  i = s->size[0] * s->size[1];
  s->size[0] = 1;
  s->size[1] = vi->size[1];
  emxEnsureCapacity_real_T(s, i);
  nm1d2 = vi->size[1];
  for (k = 0; k < nm1d2; k++) {
    s->data[k] = rt_powd_snf(vi->data[k], 5.0);
  }

  i = s->size[0] * s->size[1];
  s->size[0] = 1;
  s->size[1] = delta_raw->size[1];
  emxEnsureCapacity_real_T(s, i);
  nm1d2 = delta_raw->size[0] * delta_raw->size[1] - 1;
  for (i = 0; i <= nm1d2; i++) {
    s->data[i] = (10.0 * delta_raw->data[i] - 15.0 * b_tmp->data[i]) + 6.0 *
      s->data[i];
  }

  emxInit_real_T(&ds, 2);

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  i = ds->size[0] * ds->size[1];
  ds->size[0] = 1;
  ds->size[1] = vi->size[1];
  emxEnsureCapacity_real_T(ds, i);
  nm1d2 = vi->size[1];
  for (k = 0; k < nm1d2; k++) {
    ndbl = vi->data[k];
    ds->data[k] = ndbl * ndbl;
  }

  i = ds->size[0] * ds->size[1];
  k = ds->size[0] * ds->size[1];
  ds->size[0] = 1;
  emxEnsureCapacity_real_T(ds, k);
  nm1d2 = i - 1;
  for (i = 0; i <= nm1d2; i++) {
    ds->data[i] = ((30.0 * ds->data[i] - 60.0 * delta_raw->data[i]) + 30.0 *
                   b_tmp->data[i]) / b_T;
  }

  emxFree_real_T(&b_tmp);
  nm1d2 = s->size[0] * s->size[1];
  for (k = 0; k < 6; k++) {
    /*  原始速度曲线 */
    i = vi->size[0] * vi->size[1];
    vi->size[0] = 1;
    vi->size[1] = s->size[1];
    emxEnsureCapacity_real_T(vi, i);
    ndbl = b_v0[k];
    for (i = 0; i < nm1d2; i++) {
      vi->data[i] = ndbl + (0.0 - ndbl) * s->data[i];
    }

    /*  原始积分位移 */
    cumtrapz(b_t, vi, delta_raw);

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 */
    ndbl = (Pf[k] - P0_data[k]) / delta_raw->data[delta_raw->size[1] - 1];

    /*  缩放后的轨迹与速度、加速度 */
    loop_ub = b_traj->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_traj->data[i + b_traj->size[0] * k] = P0_data[k] + ndbl *
        delta_raw->data[i];
    }

    loop_ub = b_vel->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_vel->data[i + b_vel->size[0] * k] = ndbl * vi->data[i];
    }

    ndbl *= 0.0 - b_v0[k];
    loop_ub = acc->size[0];
    for (i = 0; i < loop_ub; i++) {
      acc->data[i + acc->size[0] * k] = ndbl * ds->data[i];
    }
  }

  emxFree_real_T(&delta_raw);
  emxFree_real_T(&vi);
  emxFree_real_T(&ds);
  emxFree_real_T(&s);
  emxFree_real_T(&b_t);
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * STEWART_CONTROL 斯图尔特平台闭环控制主函数
 *  输入:
 *    sensor_length - 当前传感器杆长(1x6)
 *    Pf - 目标位姿 [x,y,z,roll,pitch,yaw] (单位: mm和弧度)
 *  输出:
 *    vec - 当前杆速指令(6x1)
 *    converged - 是否收敛标志
 * Arguments    : void
 * Return Type  : void
 */
void c_stewart_control_function_V4_p(void)
{
  filtered_P0.size[1] = 0;
  emxInit_real_T(&vel, 2);
  emxInit_real_T(&traj, 2);
  filtered_P0_not_empty = false;
  traj_not_empty = false;
}

/*
 * STEWART_CONTROL 斯图尔特平台闭环控制主函数
 *  输入:
 *    sensor_length - 当前传感器杆长(1x6)
 *    Pf - 目标位姿 [x,y,z,roll,pitch,yaw] (单位: mm和弧度)
 *  输出:
 *    vec - 当前杆速指令(6x1)
 *    converged - 是否收敛标志
 * Arguments    : void
 * Return Type  : void
 */
void d_stewart_control_function_V4_p(void)
{
  emxFree_real_T(&traj);
  emxFree_real_T(&vel);
}

/*
 * STEWART_CONTROL 斯图尔特平台闭环控制主函数
 *  输入:
 *    sensor_length - 当前传感器杆长(1x6)
 *    Pf - 目标位姿 [x,y,z,roll,pitch,yaw] (单位: mm和弧度)
 *  输出:
 *    vec - 当前杆速指令(6x1)
 *    converged - 是否收敛标志
 * Arguments    : const double sensor_length[6]
 *                double Pf[6]
 *                double vec[6]
 * Return Type  : void
 */
void stewart_control_function_V4_part_test_1(const double sensor_length[6],
  double Pf[6], double vec[6])
{
  bool guard1 = false;
  double l_i[3];
  int i;
  int k;
  double P0[6];
  int idx;
  bool exitg1;
  double absxk;
  int b_i;
  emxArray_real_T *unusedU2;
  emxArray_real_T *b_traj;
  double d;
  emxArray_real_T *b_vel;
  static const double b_dv[6] = { -40.0, -50.0, 70.0, -0.20943951023931956,
    0.29670597283903605, 0.31415926535897931 };

  double Xn_data[6];
  double unusedU0[10];
  double b_Pf[6];
  double Pt[16];
  double b_t[3];
  double c_t;
  double rotmat_tmp;
  double b_rotmat_tmp;
  bool guard2 = false;
  double scale;
  double c_rotmat_tmp;
  double Rt[16];
  double d_rotmat_tmp[9];
  bool y;
  int tmp_size[2];
  double R[9];
  double J[36];
  double e_rotmat_tmp[9];
  double MovingPlat_Hom[24];
  double b_Pt[24];
  double c_Pt[16];
  double p_i_tmp[3];
  if (!isInitialized_stewart_control_function_V4_part_test_1) {
    stewart_control_function_V4_part_test_1_initialize();
  }

  /*  % 此版本是可以处理变化的目标位姿（即Pf可以变化），不只是只处理一次    %%%% */
  /*      但是此版本速度较低，不能较快的响应 */
  /*  */
  /*  声明全局变量  (在matlab中全局变量在函数里面应用，要先在函数外面进行定义，在要用到的函数里面要进行声明之后才能用) */
  /*  声明持久变量保存控制状态 */
  /*  参数初始化 */
  /*     %% 当规划轨迹为空，或者前一次杆长为空作为一个条件初始化的条件。 另外当目标位姿的位置最大的变化超过2mm的时候，需要重新初始化。 */
  /* 再或者当目标姿态的最大变化超过0.5度（角度）的时候，需要重新初始化 */
  guard1 = false;
  if (!traj_not_empty) {
    guard1 = true;
  } else {
    l_i[0] = fabs(pre_Pf[0] - Pf[0]);
    l_i[1] = fabs(pre_Pf[1] - Pf[1]);
    l_i[2] = fabs(pre_Pf[2] - Pf[2]);
    if (maximum(l_i) > 2.0) {
      guard1 = true;
    } else {
      l_i[0] = fabs(pre_Pf[3] - Pf[3]);
      l_i[1] = fabs(pre_Pf[4] - Pf[4]);
      l_i[2] = fabs(pre_Pf[5] - Pf[5]);
      if (maximum(l_i) > 0.5) {
        guard1 = true;
      }
    }
  }

  if (guard1) {
    /* 只有初始满足这个条件的时候才进行初始化 */
    /*  --- 控制参数 --- */
    /*          max_speed = 200;    % 杆速限幅 */
    traj->size[0] = 0;
    traj->size[1] = 0;
    traj_not_empty = false;

    /* 初始化为空矩阵 */
    /*          vel = []; %初始化为空矩阵 */
    vel->size[0] = 0;
    vel->size[1] = 0;

    /*  初始化前次杆长 */
    /*          pre_length_last = zeros(1,6); % 前一次更新前的杆长  这个值应该在这个函数外面设置成一个为0的全局变量，否则每次进入初始化都会重新使其为0，上次更新的数据没法用了%%%%%%%%%%%%%%%%%% */
    time_idx = 1.0;

    /*  */
    converged = false;

    /* 这个可能要设置为全局变量 */
    t = 0.0;
    t1 = 0.0;
    flag = 1.0;

    /*          v0 = [40; 19; 38; deg2rad(12); deg2rad(-6); deg2rad(17)]; % 起始速度（角度转为弧度） */
    for (i = 0; i < 6; i++) {
      pre_length[i] = 0.0;
      v0[i] = b_dv[i];
    }

    /*  起始速度（角度转为弧度） */
    /*  MovingPlat 数据 */
    /*  StaticPlat 数据 */
    memcpy(&MovingPlat[0], &dv[0], 18U * sizeof(double));
    memcpy(&StaticPlat[0], &dv1[0], 18U * sizeof(double));

    /*  杆速限幅 */
    /* 先把输入的角度存起来（单位是角度） */
    /*          %% 将输入的目标位姿的单位是角度转化成弧度 */
    /*  前三个元素（保持不变） */
    /*  后三个角度值（单位：度） */
    /*  将角度转换为弧度 */
    /*  使用 deg2rad 函数转 */
    /*  合并结果 */
    b_Pf[0] = Pf[0];
    b_Pf[3] = 0.017453292519943295 * Pf[3];
    b_Pf[1] = Pf[1];
    b_Pf[4] = 0.017453292519943295 * Pf[4];
    b_Pf[2] = Pf[2];
    b_Pf[5] = 0.017453292519943295 * Pf[5];

    /* 弧度   */
    /*  计算初始位姿 */
    for (b_i = 0; b_i < 6; b_i++) {
      Pf_next[b_i] = Pf[b_i];
      Pf[b_i] = b_Pf[b_i];
      Xn_data[b_i] = sensor_length[b_i] + 550.8875;
    }

    PositiveSolution(MovingPlat, StaticPlat, Xn_data, P0, unusedU0);

    /*          % 调用时间调整函数（仅在初始化时执行） */
    /*  速度限制值 */
    /*  时间步长 */
    /*          T_original = 5;         % 初始规划时间 */
    b_Pf[0] = P0[0];
    b_Pf[3] = 0.017453292519943295 * P0[3];
    b_Pf[1] = P0[1];
    b_Pf[4] = 0.017453292519943295 * P0[4];
    b_Pf[2] = P0[2];
    b_Pf[5] = 0.017453292519943295 * P0[5];
    T = adjust_planning_time(b_Pf, Pf, v0);
    filtered_P0.size[0] = 0;
    filtered_P0.size[1] = 0;
    filtered_P0_not_empty = false;

    /*  初始化滤波后的P0 = [];   % 初始化滤波后的P0 */
  }

  for (i = 0; i < 6; i++) {
    vec[i] = 0.0;
  }

  if (!converged) {
    /*     %% 闭环控制核心逻辑 */
    /*  检测杆长变化(阈值1mm)   第一种情况：当杆长变化超过3mm的时候，则重新进行规划，主要考虑到Pf（目标位姿）不变的情况。 */
    /*                          第二种情况：当目标位姿中的位置变化超过2mm，或者姿态变化超过0.5度，则也进行重新规划（目标位姿变化的时候需要重新规划） */
    for (k = 0; k < 6; k++) {
      P0[k] = fabs(sensor_length[k] - pre_length[k]);
    }

    if (!rtIsNaN(P0[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 6)) {
        if (!rtIsNaN(P0[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      absxk = P0[0];
    } else {
      absxk = P0[idx - 1];
      b_i = idx + 1;
      for (k = b_i; k < 7; k++) {
        d = P0[k - 1];
        if (absxk < d) {
          absxk = d;
        }
      }
    }

    emxInit_real_T(&unusedU2, 2);
    emxInit_real_T(&b_traj, 2);
    emxInit_real_T(&b_vel, 2);
    guard1 = false;
    if (absxk > 2.0) {
      guard1 = true;
    } else {
      l_i[0] = fabs(pre_Pf[0] - Pf_next[0]);
      l_i[1] = fabs(pre_Pf[1] - Pf_next[1]);
      l_i[2] = fabs(pre_Pf[2] - Pf_next[2]);
      if (maximum(l_i) > 2.0) {
        guard1 = true;
      } else {
        l_i[0] = fabs(pre_Pf[3] - Pf_next[3]);
        l_i[1] = fabs(pre_Pf[4] - Pf_next[4]);
        l_i[2] = fabs(pre_Pf[5] - Pf_next[5]);
        if (maximum(l_i) > 0.5) {
          guard1 = true;
        }
      }
    }

    if (guard1) {
      /*  杆长变化时重新规划轨迹 */
      /*          v0 = [40; 19; 40; deg2rad(12); deg2rad(-6); deg2rad(17)]; % 起始速度（角度转为弧度） */
      /* disp('1收敛完成，退出控制循环速度：  '); */
      /*              disp(speed); */
      /* 解算初始杆长对应的初始位姿 */
      /*  计算初始位姿   （这里的初始位姿指的是为了求解6自由度斯图尔特平台的正解，即由杆长求解位姿时候需要用到的迭代的解） */
      for (b_i = 0; b_i < 6; b_i++) {
        Xn_data[b_i] = sensor_length[b_i] + 550.8875;
      }

      PositiveSolution(MovingPlat, StaticPlat, Xn_data, P0, unusedU0);

      /* 这里角度的单位是弧度 */
      /*          disp('P0_1，退出控制循环速度：  '); */
      /*          disp(P0); */
      b_Pf[0] = P0[0];
      b_Pf[3] = 0.017453292519943295 * P0[3];
      b_Pf[1] = P0[1];
      b_Pf[4] = 0.017453292519943295 * P0[4];
      b_Pf[2] = P0[2];
      b_Pf[5] = 0.017453292519943295 * P0[5];
      T -= t;

      /*          disp('Pf，退出控制循环速度：  '); */
      /*          disp(Pf); */
      /*  对P0进行滤波，减少噪声影响 */
      /*  滤波系数（0~1，越小越平滑） */
      if (!filtered_P0_not_empty) {
        filtered_P0.size[0] = 1;
        filtered_P0.size[1] = 6;
        for (b_i = 0; b_i < 6; b_i++) {
          filtered_P0.data[b_i] = b_Pf[b_i];
        }

        filtered_P0_not_empty = true;
      } else {
        idx = filtered_P0.size[0] * filtered_P0.size[1];
        for (b_i = 0; b_i < idx; b_i++) {
          Xn_data[b_i] = 0.30000000000000004 * filtered_P0.data[b_i];
        }

        filtered_P0.size[0] = 1;
        filtered_P0.size[1] = 6;
        for (b_i = 0; b_i < 6; b_i++) {
          filtered_P0.data[b_i] = 0.7 * b_Pf[b_i] + Xn_data[b_i];
        }

        filtered_P0_not_empty = true;
      }

      /*  主要分成两部分：第一部分：是将规划好的轨迹执行了至少一次，说明已经将轨迹运行了一段时间，规划轨迹和实际轨迹不符了，所以需要用真实轨迹进行拼接 */
      /*                  第二部分：是目标位姿变化了，和第一种目标位姿不变，所以也是已经运行一段时间了，所以也需要用真实轨迹进行拼接 */
      guard2 = false;
      if (t1 != 0.0) {
        guard2 = true;
      } else {
        l_i[0] = fabs(pre_Pf[0] - Pf_next[0]);
        l_i[1] = fabs(pre_Pf[1] - Pf_next[1]);
        l_i[2] = fabs(pre_Pf[2] - Pf_next[2]);
        if (maximum(l_i) > 2.0) {
          guard2 = true;
        } else {
          l_i[0] = fabs(pre_Pf[3] - Pf_next[3]);
          l_i[1] = fabs(pre_Pf[4] - Pf_next[4]);
          l_i[2] = fabs(pre_Pf[5] - Pf_next[5]);
          if (maximum(l_i) > 0.5) {
            guard2 = true;
          }
        }
      }

      if (guard2) {
        for (b_i = 0; b_i < 6; b_i++) {
          pre_length_last[b_i] = (sensor_length[b_i] - pre_length_last[b_i]) /
            0.05;
        }

        /* 这里面除的值应该是实际的两个函数之间的运行时间（应该和这个任务的时间间隔有关系） */
        /* 判断是要进行速度拼接还是重新开始以初始速度运行 */
        y = false;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k < 6)) {
          if (pre_length_last[k] != 0.0) {
            y = true;
            exitg1 = true;
          } else {
            k++;
          }
        }

        if (y && (flag_v != 1.0)) {
          Calculateposevelocity(MovingPlat, StaticPlat, b_Pf, pre_length_last,
                                v0);

          /* 计算实际位姿的速度是什么    这里的位姿速度单位是弧度   */
        }

        /*             v0 = Calculateposevelocity(MovingPlat, StaticPlat, P0, pre_vec_actual); %计算实际位姿的速度是什么    这里的位姿速度单位是弧度    */
      }

      if (flag != 1.0) {
        /* 判断这次运行该函数时有没有初始化，若不是，则进行角度转弧度  */
        /*  前三个元素（保持不变） */
        /* 将角度转换为弧度 */
        /*  使用 deg2rad 函数转 */
        /* 合并结果 */
        b_Pf[0] = Pf[0];
        b_Pf[3] = 0.017453292519943295 * Pf[3];
        b_Pf[1] = Pf[1];
        b_Pf[4] = 0.017453292519943295 * Pf[4];
        b_Pf[2] = Pf[2];
        b_Pf[5] = 0.017453292519943295 * Pf[5];
        for (b_i = 0; b_i < 6; b_i++) {
          Pf[b_i] = b_Pf[b_i];
        }
      }

      /*          disp('Pf，退出控制循环速度：  '); */
      /*          disp(Pf); */
      /*          disp('v0，退出控制循环速度：  '); */
      /*          disp(v0); */
      /*          disp('filtered_P0，退出控制循环速度：  '); */
      /*          disp(filtered_P0); */
      tmp_size[0] = filtered_P0.size[0];
      tmp_size[1] = filtered_P0.size[1];
      idx = filtered_P0.size[0] * filtered_P0.size[1] - 1;
      if (0 <= idx) {
        memcpy(&J[0], &filtered_P0.data[0], (idx + 1) * sizeof(double));
      }

      d_velocity_shaped_trajectory_fi(J, tmp_size, Pf, T, v0, b_traj, b_vel,
        unusedU2);
      b_i = traj->size[0] * traj->size[1];
      traj->size[0] = b_traj->size[0];
      traj->size[1] = 6;
      emxEnsureCapacity_real_T(traj, b_i);
      idx = b_traj->size[0] * b_traj->size[1];
      for (b_i = 0; b_i < idx; b_i++) {
        traj->data[b_i] = b_traj->data[b_i];
      }

      traj_not_empty = (traj->size[0] != 0);
      b_i = vel->size[0] * vel->size[1];
      vel->size[0] = b_vel->size[0];
      vel->size[1] = 6;
      emxEnsureCapacity_real_T(vel, b_i);
      idx = b_vel->size[0] * b_vel->size[1];
      for (b_i = 0; b_i < idx; b_i++) {
        vel->data[b_i] = b_vel->data[b_i];
      }

      /* %%%%%%%%%%%%%%%%% */
      time_idx = 1.0;

      /*  重置时间索引 */
      for (b_i = 0; b_i < 6; b_i++) {
        pre_length[b_i] = sensor_length[b_i];
      }

      /*  更新记录值 */
      t = 0.0;

      /*  重置运行的时间索引 */
      t1 = 0.0;

      /*  重置运行的时间索引 */
    }

    emxFree_real_T(&b_vel);
    emxFree_real_T(&b_traj);
    emxFree_real_T(&unusedU2);

    /*  初始化收敛标志 */
    /*      converged = false; %这个可能要设置为全局变量 */
    /*  执行当前轨迹 */
    if (traj_not_empty && (time_idx <= traj->size[0])) {
      /* size(traj,1)表示traj的行数 */
      /*  获取当前轨迹点 */
      idx = traj->size[1];
      for (b_i = 0; b_i < idx; b_i++) {
        Xn_data[b_i] = traj->data[((int)time_idx + traj->size[0] * b_i) - 1];
      }

      /*  计算杆速 */
      /*  计算斯图尔特平台各支链的杆速 */
      /*  输入: */
      /*    MovingPlat - 动平台铰链点坐标（3×n矩阵） */
      /*    StaticPlat - 静平台铰链点坐标（3×n矩阵） */
      /*    X - 当前位姿 [t; angles] (6×1向量) */
      /*    Vn - 平台位姿速度 [v; omega] (6×1向量) */
      /*  输出: */
      /*    l_dot - 各支链的杆速 (n×1向量) */
      /*  计算杆速的函数 */
      /*  计算雅可比矩阵 */
      /*  计算雅可比矩阵（速度雅可比） */
      b_t[0] = Xn_data[0];
      b_t[1] = Xn_data[1];
      b_t[2] = Xn_data[2];

      /*  构造旋转矩阵 (ZYX欧拉角) */
      absxk = Xn_data[5];
      b_sind(&absxk);
      c_t = Xn_data[5];
      b_cosd(&c_t);
      rotmat_tmp = Xn_data[4];
      b_sind(&rotmat_tmp);
      b_rotmat_tmp = Xn_data[4];
      b_cosd(&b_rotmat_tmp);
      scale = Xn_data[3];
      b_sind(&scale);
      c_rotmat_tmp = Xn_data[3];
      b_cosd(&c_rotmat_tmp);
      d_rotmat_tmp[0] = c_t;
      d_rotmat_tmp[3] = -absxk;
      d_rotmat_tmp[6] = 0.0;
      d_rotmat_tmp[1] = absxk;
      d_rotmat_tmp[4] = c_t;
      d_rotmat_tmp[7] = 0.0;
      R[0] = b_rotmat_tmp;
      R[3] = 0.0;
      R[6] = rotmat_tmp;
      d_rotmat_tmp[2] = 0.0;
      R[1] = 0.0;
      d_rotmat_tmp[5] = 0.0;
      R[4] = 1.0;
      d_rotmat_tmp[8] = 1.0;
      R[7] = 0.0;
      R[2] = -rotmat_tmp;
      R[5] = 0.0;
      R[8] = b_rotmat_tmp;
      for (b_i = 0; b_i < 3; b_i++) {
        d = d_rotmat_tmp[b_i + 3];
        idx = (int)d_rotmat_tmp[b_i + 6];
        for (i = 0; i < 3; i++) {
          e_rotmat_tmp[b_i + 3 * i] = (d_rotmat_tmp[b_i] * R[3 * i] + d * R[3 *
            i + 1]) + (double)idx * R[3 * i + 2];
        }
      }

      d_rotmat_tmp[0] = 1.0;
      d_rotmat_tmp[3] = 0.0;
      d_rotmat_tmp[6] = 0.0;
      d_rotmat_tmp[1] = 0.0;
      d_rotmat_tmp[4] = c_rotmat_tmp;
      d_rotmat_tmp[7] = -scale;
      d_rotmat_tmp[2] = 0.0;
      d_rotmat_tmp[5] = scale;
      d_rotmat_tmp[8] = c_rotmat_tmp;
      for (b_i = 0; b_i < 3; b_i++) {
        d = e_rotmat_tmp[b_i + 3];
        rotmat_tmp = e_rotmat_tmp[b_i + 6];
        for (idx = 0; idx < 3; idx++) {
          R[b_i + 3 * idx] = (e_rotmat_tmp[b_i] * d_rotmat_tmp[3 * idx] + d *
                              d_rotmat_tmp[3 * idx + 1]) + rotmat_tmp *
            d_rotmat_tmp[3 * idx + 2];
        }
      }

      /*  计算杆速：l_dot = J' * Vn */
      for (i = 0; i < 6; i++) {
        /*  动平台铰链点局部坐标 */
        /*  静平台铰链点全局坐标 */
        /*  动平台铰链点全局坐标 */
        /*  支链向量 */
        /*  单位方向向量 */
        b_rotmat_tmp = 0.0;
        scale = 3.3121686421112381E-170;
        d = MovingPlat[3 * i + 2];
        rotmat_tmp = MovingPlat[3 * i];
        c_rotmat_tmp = MovingPlat[3 * i + 1];
        for (k = 0; k < 3; k++) {
          absxk = (R[k] * rotmat_tmp + R[k + 3] * c_rotmat_tmp) + R[k + 6] * d;
          p_i_tmp[k] = absxk;
          absxk = (absxk + b_t[k]) - StaticPlat[k + 3 * i];
          l_i[k] = absxk;
          absxk = fabs(absxk);
          if (absxk > scale) {
            c_t = scale / absxk;
            b_rotmat_tmp = b_rotmat_tmp * c_t * c_t + 1.0;
            scale = absxk;
          } else {
            c_t = absxk / scale;
            b_rotmat_tmp += c_t * c_t;
          }
        }

        b_rotmat_tmp = scale * sqrt(b_rotmat_tmp);

        /*  平移部分雅可比（方向向量） */
        d = l_i[0] / b_rotmat_tmp;
        l_i[0] = d;
        J[i] = d;
        d = l_i[1] / b_rotmat_tmp;
        l_i[1] = d;
        J[i + 6] = d;
        d = l_i[2] / b_rotmat_tmp;
        J[i + 12] = d;

        /*  旋转部分雅可比（方向向量叉乘动点位置） */
        J[i + 18] = p_i_tmp[1] * d - p_i_tmp[2] * l_i[1];
        J[i + 24] = p_i_tmp[2] * l_i[0] - p_i_tmp[0] * d;
        J[i + 30] = p_i_tmp[0] * l_i[1] - p_i_tmp[1] * l_i[0];
        d = 0.0;
        for (b_i = 0; b_i < 6; b_i++) {
          d += J[i + 6 * b_i] * vel->data[((int)time_idx + vel->size[0] * b_i) -
            1];
        }

        vec[i] = fmin(fmax(d, -20.0), 20.0);
      }

      /*  限幅 */
      time_idx++;

      /*  推进时间索引 */
      t += 0.05;
      t1 += 0.05;
    }

    /*  收敛检测（误差<2mm） */
    /*  InverseSolution(MovingPlat,SataticPlat,X) 并联机构运动学反解 */
    /*  MovingPlat 动平台铰链点结构参数 */
    /*  SataticPlat 静平台铰链点结构参数 */
    /*  X 为动平台相对静平台的位姿坐标 */
    /*  Result 为各支链长度 */
    /*  平移齐次变换矩阵 */
    memset(&Pt[0], 0, 16U * sizeof(double));
    Pt[0] = 1.0;
    Pt[5] = 1.0;
    Pt[10] = 1.0;
    Pt[15] = 1.0;
    Pt[12] = Pf_next[0];
    Pt[13] = Pf_next[1];
    Pt[14] = Pf_next[2];

    /* 平移齐次变换矩阵 */
    absxk = Pf_next[5];
    b_sind(&absxk);
    c_t = Pf_next[5];
    b_cosd(&c_t);
    rotmat_tmp = Pf_next[4];
    b_sind(&rotmat_tmp);
    b_rotmat_tmp = Pf_next[4];
    b_cosd(&b_rotmat_tmp);
    scale = Pf_next[3];
    b_sind(&scale);
    c_rotmat_tmp = Pf_next[3];
    b_cosd(&c_rotmat_tmp);

    /*  RPY角转齐次变换矩阵（仅旋转部分） */
    memset(&Rt[0], 0, 16U * sizeof(double));
    Rt[0] = 1.0;
    Rt[5] = 1.0;
    Rt[10] = 1.0;
    Rt[15] = 1.0;
    d_rotmat_tmp[0] = c_t;
    d_rotmat_tmp[3] = -absxk;
    d_rotmat_tmp[6] = 0.0;
    d_rotmat_tmp[1] = absxk;
    d_rotmat_tmp[4] = c_t;
    d_rotmat_tmp[7] = 0.0;
    R[0] = b_rotmat_tmp;
    R[3] = 0.0;
    R[6] = rotmat_tmp;
    d_rotmat_tmp[2] = 0.0;
    R[1] = 0.0;
    d_rotmat_tmp[5] = 0.0;
    R[4] = 1.0;
    d_rotmat_tmp[8] = 1.0;
    R[7] = 0.0;
    R[2] = -rotmat_tmp;
    R[5] = 0.0;
    R[8] = b_rotmat_tmp;
    for (b_i = 0; b_i < 3; b_i++) {
      d = d_rotmat_tmp[b_i + 3];
      idx = (int)d_rotmat_tmp[b_i + 6];
      for (i = 0; i < 3; i++) {
        e_rotmat_tmp[b_i + 3 * i] = (d_rotmat_tmp[b_i] * R[3 * i] + d * R[3 * i
          + 1]) + (double)idx * R[3 * i + 2];
      }
    }

    d_rotmat_tmp[0] = 1.0;
    d_rotmat_tmp[3] = 0.0;
    d_rotmat_tmp[6] = 0.0;
    d_rotmat_tmp[1] = 0.0;
    d_rotmat_tmp[4] = c_rotmat_tmp;
    d_rotmat_tmp[7] = -scale;
    d_rotmat_tmp[2] = 0.0;
    d_rotmat_tmp[5] = scale;
    d_rotmat_tmp[8] = c_rotmat_tmp;
    for (b_i = 0; b_i < 3; b_i++) {
      d = e_rotmat_tmp[b_i + 3];
      rotmat_tmp = e_rotmat_tmp[b_i + 6];
      for (idx = 0; idx < 3; idx++) {
        Rt[b_i + (idx << 2)] = (e_rotmat_tmp[b_i] * d_rotmat_tmp[3 * idx] + d *
          d_rotmat_tmp[3 * idx + 1]) + rotmat_tmp * d_rotmat_tmp[3 * idx + 2];
      }
    }

    /* RPY旋转齐次变换 */
    /* 初始位置 动平台相对静平台的位姿齐次变换矩阵 */
    /* 动平台铰链点变为齐次形式 */
    for (k = 0; k < 6; k++) {
      idx = k << 2;
      MovingPlat_Hom[idx] = MovingPlat[3 * k];
      MovingPlat_Hom[idx + 1] = MovingPlat[3 * k + 1];
      idx += 3;
      MovingPlat_Hom[idx - 1] = MovingPlat[3 * k + 2];
      MovingPlat_Hom[idx] = 1.0;
    }

    /* 实际动平台坐标为 */
    for (b_i = 0; b_i < 4; b_i++) {
      d = Pt[b_i + 4];
      rotmat_tmp = Pt[b_i + 8];
      c_rotmat_tmp = Pt[b_i + 12];
      for (idx = 0; idx < 4; idx++) {
        i = idx << 2;
        c_Pt[b_i + i] = ((Pt[b_i] * Rt[i] + d * Rt[i + 1]) + rotmat_tmp * Rt[i +
                         2]) + c_rotmat_tmp * Rt[i + 3];
      }

      d = c_Pt[b_i + 4];
      rotmat_tmp = c_Pt[b_i + 8];
      c_rotmat_tmp = c_Pt[b_i + 12];
      for (idx = 0; idx < 6; idx++) {
        i = idx << 2;
        b_Pt[b_i + i] = ((c_Pt[b_i] * MovingPlat_Hom[i] + d * MovingPlat_Hom[i +
                          1]) + rotmat_tmp * MovingPlat_Hom[i + 2]) +
          c_rotmat_tmp * MovingPlat_Hom[i + 3];
      }
    }

    memcpy(&MovingPlat_Hom[0], &b_Pt[0], 24U * sizeof(double));
    for (i = 0; i < 6; i++) {
      scale = 3.3121686421112381E-170;
      idx = i << 2;
      absxk = fabs(MovingPlat_Hom[idx] - StaticPlat[3 * i]);
      if (absxk > 3.3121686421112381E-170) {
        b_rotmat_tmp = 1.0;
        scale = absxk;
      } else {
        c_t = absxk / 3.3121686421112381E-170;
        b_rotmat_tmp = c_t * c_t;
      }

      absxk = fabs(MovingPlat_Hom[idx + 1] - StaticPlat[3 * i + 1]);
      if (absxk > scale) {
        c_t = scale / absxk;
        b_rotmat_tmp = b_rotmat_tmp * c_t * c_t + 1.0;
        scale = absxk;
      } else {
        c_t = absxk / scale;
        b_rotmat_tmp += c_t * c_t;
      }

      absxk = fabs(MovingPlat_Hom[idx + 2] - StaticPlat[3 * i + 2]);
      if (absxk > scale) {
        c_t = scale / absxk;
        b_rotmat_tmp = b_rotmat_tmp * c_t * c_t + 1.0;
        scale = absxk;
      } else {
        c_t = absxk / scale;
        b_rotmat_tmp += c_t * c_t;
      }

      P0[i] = fabs((scale * sqrt(b_rotmat_tmp) - 550.8875) - sensor_length[i]);
    }

    if (!rtIsNaN(P0[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 6)) {
        if (!rtIsNaN(P0[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      absxk = P0[0];
    } else {
      absxk = P0[idx - 1];
      b_i = idx + 1;
      for (k = b_i; k < 7; k++) {
        d = P0[k - 1];
        if (absxk < d) {
          absxk = d;
        }
      }
    }

    converged = (absxk < 2.0);

    /*          if converged || all(vec == 0) */
    if (converged) {
      for (i = 0; i < 6; i++) {
        vec[i] = 0.0;
      }

      /*  发送零速 */
      traj->size[0] = 0;
      traj->size[1] = 0;
      traj_not_empty = false;

      /* 初始化为空矩阵 */
    }

    /*  前一次更新前的杆长 */
    for (b_i = 0; b_i < 6; b_i++) {
      pre_length_last[b_i] = sensor_length[b_i];
      pre_Pf[b_i] = Pf_next[b_i];
    }

    flag++;
    flag_v++;
    if (flag_v > 10000.0) {
      flag_v = 2.0;
    }
  }
}

/*
 * File trailer for stewart_control_function_V4_part_test_1.c
 *
 * [EOF]
 */
