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
 * ����˹ͼ����ƽ̨��֧���ĸ���
 *  ����:
 *    MovingPlat - ��ƽ̨���������꣨3��n����
 *    StaticPlat - ��ƽ̨���������꣨3��n����
 *    X - ��ǰλ�� [t; angles] (6��1����)
 *    Vn - ƽ̨λ���ٶ� [v; omega] (6��1����)
 *  ���:
 *    l_dot - ��֧���ĸ��� (n��1����)
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

  /*  ����λ���ٶȵĺ��� */
  /* �����Ӧ���������� */
  /* �����Ӧ���������� */
  /*  �����ſɱȾ��� */
  Jocobian(b_MovingPlat, b_StaticPlat, P0, J);

  /*  ������٣�l_dot = J' * Vn */
  /*  ʹ��α�棬��ΪJ���ܲ��Ƿ��� */
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
 * ���㵱ǰ֧��������Ŀ�곤�Ȳ�ֵ����
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

  /*  InverseSolution(MovingPlat,SataticPlat,X) ���������˶�ѧ���� */
  /*  MovingPlat ��ƽ̨������ṹ���� */
  /*  SataticPlat ��ƽ̨������ṹ���� */
  /*  X Ϊ��ƽ̨��Ծ�ƽ̨��λ������ */
  /*  Result Ϊ��֧������ */
  /*  ƽ����α任���� */
  memset(&Pt[0], 0, 16U * sizeof(double));
  Pt[0] = 1.0;
  Pt[5] = 1.0;
  Pt[10] = 1.0;
  Pt[15] = 1.0;
  Pt[12] = X[0];
  Pt[13] = X[1];
  Pt[14] = X[2];

  /* ƽ����α任���� */
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

  /*  RPY��ת��α任���󣨽���ת���֣� */
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

  /* RPY��ת��α任 */
  /* ��ʼλ�� ��ƽ̨��Ծ�ƽ̨��λ����α任���� */
  /* ��ƽ̨�������Ϊ�����ʽ */
  for (k = 0; k < 6; k++) {
    MovingPlat_Hom_tmp = k << 2;
    MovingPlat_Hom[MovingPlat_Hom_tmp] = b_MovingPlat[3 * k];
    MovingPlat_Hom[MovingPlat_Hom_tmp + 1] = b_MovingPlat[3 * k + 1];
    MovingPlat_Hom_tmp += 3;
    MovingPlat_Hom[MovingPlat_Hom_tmp - 1] = b_MovingPlat[3 * k + 2];
    MovingPlat_Hom[MovingPlat_Hom_tmp] = 1.0;
  }

  /* ʵ�ʶ�ƽ̨����Ϊ */
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

  /*  ȷ�������� */
}

/*
 * �����ſɱȾ����ٶ��ſɱȣ�
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

  /*  ������ת���� (ZYXŷ����) */
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
    /*  ��ƽ̨������ֲ����� */
    /*  ��ƽ̨������ȫ������ */
    /*  ��ƽ̨������ȫ������ */
    /*  ֧������ */
    /*  ��λ�������� */
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

    /*  ƽ�Ʋ����ſɱȣ����������� */
    d = l_i[0] / b_rotmat_tmp;
    l_i[0] = d;
    J[i] = d;
    d = l_i[1] / b_rotmat_tmp;
    l_i[1] = d;
    J[i + 6] = d;
    d = l_i[2] / b_rotmat_tmp;
    J[i + 12] = d;

    /*  ��ת�����ſɱȣ�����������˶���λ�ã� */
    J[i + 18] = p_i_tmp[1] * d - p_i_tmp[2] * l_i[1];
    J[i + 24] = p_i_tmp[2] * l_i[0] - p_i_tmp[0] * d;
    J[i + 30] = p_i_tmp[0] * l_i[1] - p_i_tmp[1] * l_i[0];
  }
}

/*
 * UNTITLED7 �˴���ʾ�йش˺�����ժҪ
 *  MovingPlat ��ƽ̨������ṹ����
 *  SataticPlat ��ƽ̨������ṹ����
 *  X0 ��ʼλ��
 *  L_target Ŀ��˳�
 *  Xn ���õ���λ��
 *  Record ��¼�������������Ϣ
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

  /*  ���溯����������� */
  /*  ���� */
  /*  InverseSolution(MovingPlat,SataticPlat,X) ���������˶�ѧ���� */
  /*  MovingPlat ��ƽ̨������ṹ���� */
  /*  SataticPlat ��ƽ̨������ṹ���� */
  /*  X Ϊ��ƽ̨��Ծ�ƽ̨��λ������ */
  /*  Result Ϊ��֧������ */
  /* ƽ����α任���� */
  /* RPY��ת��α任 */
  /* ��ʼλ�� ��ƽ̨��Ծ�ƽ̨��λ����α任���� */
  /* ��ƽ̨�������Ϊ�����ʽ */
  for (k = 0; k < 6; k++) {
    MovingPlat_Hom_tmp = k << 2;
    MovingPlat_Hom[MovingPlat_Hom_tmp] = b_MovingPlat[3 * k];
    MovingPlat_Hom[MovingPlat_Hom_tmp + 1] = b_MovingPlat[3 * k + 1];
    MovingPlat_Hom_tmp += 3;
    MovingPlat_Hom[MovingPlat_Hom_tmp - 1] = b_MovingPlat[3 * k + 2];
    MovingPlat_Hom[MovingPlat_Hom_tmp] = 1.0;
  }

  /* ʵ�ʶ�ƽ̨����Ϊ */
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

  /*  �õ��Ĳ��� */
  /* t��0~1���� */
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

  /* ����F����ֵ */
  /* ��ƫ��ʱ��΢С�䶯�� */
  memset(&Record[0], 0, 10U * sizeof(double));

  /* ��һ��Ϊ�ƽ����򣬵ڶ���Ϊţ�ٵ������� */
  /* ����ֵ */
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

    /* ŷ���� */
    /* Xn=Xn+inv(Jn,1e-5)*Deta_L; */
    /* ��� */
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

    /* ţ�ٵ������бƽ� У�� */
    NewtonNum = 0;

    /* ţ�ٵ������� */
    exitg1 = false;
    while ((!exitg1) && (Ferr > 1.0E-6)) {
      /* dFdX(MPlat,SPlat,X,L,delta) F��X��ƫ�� */
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

      /* ƫ�� */
      F(b_MovingPlat, SataticPlat, Xn, Ln, Fi_delta);

      /*          FXL=F(MovingPlat,SataticPlat,Xn,Ln);%������pinv() */
      inv(Jn, b_dv);
      for (i = 0; i < 6; i++) {
        Ferr = 0.0;
        for (k = 0; k < 6; k++) {
          Ferr += b_dv[i + 6 * k] * Fi_delta[k];
        }

        Xn[i] -= Ferr;
      }

      /* ������pinv() */
      /*          Xn=Xn-dF_dX/FXL;%������pinv() */
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
        /* fprintf('��%d�ν���ʱ��ţ�ٵ���������\n',n) */
        exitg1 = true;
      } else {
        Record[n] = (double)n + 1.0;
        Record[n + 5] = NewtonNum;
      }
    }
  }
}

/*
 * �����켣�滮ʱ��������֧���ٶ�����
 *  ���������
 *    P0 - ��ʼλ�� [x,y,z,roll,pitch,yaw]
 *    Pf - Ŀ��λ�� [x,y,z,roll,pitch,yaw]
 *    T_original - ԭʼ�滮ʱ��(s)
 *    dt - ʱ�䲽��(s)
 *    velocity_limit - ֧���ٶ�����(mm/s)
 *  �����
 *    T_new - ������Ĺ滮ʱ��(s)
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

  /*  ���溯�����ڵ����滮����ʱ�� */
  /*     %% ��һ�ι켣�滮 */
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

  /*     %% ����֧���ٶ� */
  vec1_size_idx_1 = y->size[1];
  nx = 6 * y->size[1];
  if (0 <= nx - 1) {
    memset(&vec1_data[0], 0, nx * sizeof(double));
  }

  i = y->size[1];
  emxFree_real_T(&y);
  for (nx = 0; nx < i; nx++) {
    /*  ��Ҫȷ�����º����ͱ������������ڿ��ã� */
    /*  MovingPlat, StaticPlat, calculateLegVelocities */
    /*  ����˹ͼ����ƽ̨��֧���ĸ��� */
    /*  ����: */
    /*    MovingPlat - ��ƽ̨���������꣨3��n���� */
    /*    StaticPlat - ��ƽ̨���������꣨3��n���� */
    /*    X - ��ǰλ�� [t; angles] (6��1����) */
    /*    Vn - ƽ̨λ���ٶ� [v; omega] (6��1����) */
    /*  ���: */
    /*    l_dot - ��֧���ĸ��� (n��1����) */
    /*  ������ٵĺ��� */
    /*  �����ſɱȾ��� */
    for (varargin_1_size_idx_0 = 0; varargin_1_size_idx_0 < 6;
         varargin_1_size_idx_0++) {
      traj1[varargin_1_size_idx_0] = traj1_data[nx + traj1_size[0] *
        varargin_1_size_idx_0];
    }

    Jocobian(dv, dv1, traj1, J);

    /*  ������٣�l_dot = J' * Vn */
    for (varargin_1_size_idx_0 = 0; varargin_1_size_idx_0 < 6;
         varargin_1_size_idx_0++) {
      kd = 0.0;
      for (k = 0; k < 6; k++) {
        kd += J[varargin_1_size_idx_0 + 6 * k] * vel1_data[nx + vel1_size[0] * k];
      }

      vec1_data[varargin_1_size_idx_0 + 6 * nx] = kd;
    }
  }

  /*     %% �ٶ����Ʒ��� */
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

  /*     %% ʱ������߼� */
  if (max_leg_vel > 14.0) {
    /*  �������������ʱ���Խ����ٶ� */
    T_new = 3.0 * (max_leg_vel / 14.0);

    /* disp(['���ٵ���: �ٶȳ������ƣ�����ʱ���� ', num2str(T_new), ' ��']); */
  } else {
    /*  �����������Сʱ�������Ч�� */
    T_new = 3.0 / (14.0 / max_leg_vel);

    /* disp(['Ƿ�ٵ���: �ٶȵ������ƣ�����ʱ���� ', num2str(T_new), ' ��']); */
  }

  /*     %% ��֤��� */
  /* if T_new < 0 */
  /*    error('����õ���ʱ�䣬�����������'); */
  /* end */
  return T_new;
}

/*
 * �̶��յ��S���ٶȲ�ֵ�켣������
 *  P0, Pf: 6x1 ��ĩ��̬
 *  v0, vf: 6x1 ��ĩ�ٶ�
 *  T, dt: ʱ��
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

  /*  �����������ֲ��� (�ⲿ�ֹ켣�滮���ٶȿ�ʼ������С�������Ļ�����ʱ�䣬��ʹʱ���÷ǳ������Ӷ���Ƭ���ڴ治����) */
  /*  function [traj, vel, acc] = plan_trajectory(P0, Pf, T) */
  /*  % �Ľ���ָ���켣�滮�����ɵ�ʱ�䳣���� */
  /*  % ����: */
  /*  %   P0: ��ʼλ�� [x0,y0,z0,roll0,pitch0,yaw0] */
  /*  %   Pf: Ŀ��λ�� [xf,yf,zf,rollf,pitchf,yawf] */
  /*  %   T: ��ʱ��(s) */
  /*  %   tau: ʱ�䳣���������ӣ�Ĭ��=5��ֵԽ���ʼԽ�죬˥��Խ���� */
  /*  % ���: */
  /*  %   traj: �켣���� (n��6) */
  /*  %   vel: �ٶȾ��� (n��6) */
  /*  %   acc: ���ٶȾ��� (n��6) */
  /*   */
  /*  %     if nargin < 4  % Ĭ�ϲ������� */
  /*  %         tau = 5;    % ����ԭ������ */
  /*  %     end */
  /*   */
  /*  %     tau = 5;    % ����ԭ������ */
  /*      tau = 3;    % ����ԭ������ */
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
  /*          if abs(delta) < 1e-6  % ���������� */
  /*              continue; */
  /*          end */
  /*           */
  /*          % �Ľ���ָ���켣����ʱ�䳣�����ڣ� */
  /*          scale = t / (T/tau);  % ʱ��߶ȱ任 */
  /*          traj(:,i) = P0(i) + delta * (1 - exp(-scale)); */
  /*           */
  /*          % �ٶȼ��㣨һ�׵����� */
  /*          vel(:,i) = delta * (tau/T) * exp(-scale); */
  /*           */
  /*          % ���ٶȼ��㣨���׵����� */
  /*          acc(:,i) = -delta * (tau/T)^2 * exp(-scale); */
  /*      end */
  /*  end */
  /*  �����������ֲ��� (�ⲿ�ָĽ��˹켣�滮���ٶȿ�ʼ������С�������Ļ�����ʱ�䣬��ʹʱ���÷ǳ������Ӷ���Ƭ���ڴ治����) */
  /* ĩ�ٶ� */
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

  /*  S�Ͳ�ֵ���� */
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
    /*  ԭʼ�ٶ����� */
    vi_size[0] = 1;
    vi_size[1] = s_size_idx_1;
    kd = b_v0[b_i];
    for (i = 0; i < s_size_idx_1; i++) {
      vi_data[i] = kd + (0.0 - kd) * s_data[i];
    }

    /*  ԭʼ����λ�� */
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

    /*  ���ź�Ĺ켣���ٶȡ����ٶ� */
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
 * �̶��յ��S���ٶȲ�ֵ�켣������
 *  P0, Pf: 6x1 ��ĩ��̬
 *  v0, vf: 6x1 ��ĩ�ٶ�
 *  T, dt: ʱ��
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

  /*  �����������ֲ��� (�ⲿ�ֹ켣�滮���ٶȿ�ʼ������С�������Ļ�����ʱ�䣬��ʹʱ���÷ǳ������Ӷ���Ƭ���ڴ治����) */
  /*  function [traj, vel, acc] = plan_trajectory(P0, Pf, T) */
  /*  % �Ľ���ָ���켣�滮�����ɵ�ʱ�䳣���� */
  /*  % ����: */
  /*  %   P0: ��ʼλ�� [x0,y0,z0,roll0,pitch0,yaw0] */
  /*  %   Pf: Ŀ��λ�� [xf,yf,zf,rollf,pitchf,yawf] */
  /*  %   T: ��ʱ��(s) */
  /*  %   tau: ʱ�䳣���������ӣ�Ĭ��=5��ֵԽ���ʼԽ�죬˥��Խ���� */
  /*  % ���: */
  /*  %   traj: �켣���� (n��6) */
  /*  %   vel: �ٶȾ��� (n��6) */
  /*  %   acc: ���ٶȾ��� (n��6) */
  /*   */
  /*  %     if nargin < 4  % Ĭ�ϲ������� */
  /*  %         tau = 5;    % ����ԭ������ */
  /*  %     end */
  /*   */
  /*  %     tau = 5;    % ����ԭ������ */
  /*      tau = 3;    % ����ԭ������ */
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
  /*          if abs(delta) < 1e-6  % ���������� */
  /*              continue; */
  /*          end */
  /*           */
  /*          % �Ľ���ָ���켣����ʱ�䳣�����ڣ� */
  /*          scale = t / (T/tau);  % ʱ��߶ȱ任 */
  /*          traj(:,i) = P0(i) + delta * (1 - exp(-scale)); */
  /*           */
  /*          % �ٶȼ��㣨һ�׵����� */
  /*          vel(:,i) = delta * (tau/T) * exp(-scale); */
  /*           */
  /*          % ���ٶȼ��㣨���׵����� */
  /*          acc(:,i) = -delta * (tau/T)^2 * exp(-scale); */
  /*      end */
  /*  end */
  /*  �����������ֲ��� (�ⲿ�ָĽ��˹켣�滮���ٶȿ�ʼ������С�������Ļ�����ʱ�䣬��ʹʱ���÷ǳ������Ӷ���Ƭ���ڴ治����) */
  /* ĩ�ٶ� */
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

  /*  S�Ͳ�ֵ���� */
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
    /*  ԭʼ�ٶ����� */
    i = vi->size[0] * vi->size[1];
    vi->size[0] = 1;
    vi->size[1] = s->size[1];
    emxEnsureCapacity_real_T(vi, i);
    ndbl = b_v0[k];
    for (i = 0; i < nm1d2; i++) {
      vi->data[i] = ndbl + (0.0 - ndbl) * s->data[i];
    }

    /*  ԭʼ����λ�� */
    cumtrapz(b_t, vi, delta_raw);

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 */
    ndbl = (Pf[k] - P0_data[k]) / delta_raw->data[delta_raw->size[1] - 1];

    /*  ���ź�Ĺ켣���ٶȡ����ٶ� */
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
 * STEWART_CONTROL ˹ͼ����ƽ̨�ջ�����������
 *  ����:
 *    sensor_length - ��ǰ�������˳�(1x6)
 *    Pf - Ŀ��λ�� [x,y,z,roll,pitch,yaw] (��λ: mm�ͻ���)
 *  ���:
 *    vec - ��ǰ����ָ��(6x1)
 *    converged - �Ƿ�������־
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
 * STEWART_CONTROL ˹ͼ����ƽ̨�ջ�����������
 *  ����:
 *    sensor_length - ��ǰ�������˳�(1x6)
 *    Pf - Ŀ��λ�� [x,y,z,roll,pitch,yaw] (��λ: mm�ͻ���)
 *  ���:
 *    vec - ��ǰ����ָ��(6x1)
 *    converged - �Ƿ�������־
 * Arguments    : void
 * Return Type  : void
 */
void d_stewart_control_function_V4_p(void)
{
  emxFree_real_T(&traj);
  emxFree_real_T(&vel);
}

/*
 * STEWART_CONTROL ˹ͼ����ƽ̨�ջ�����������
 *  ����:
 *    sensor_length - ��ǰ�������˳�(1x6)
 *    Pf - Ŀ��λ�� [x,y,z,roll,pitch,yaw] (��λ: mm�ͻ���)
 *  ���:
 *    vec - ��ǰ����ָ��(6x1)
 *    converged - �Ƿ�������־
 * Arguments    : const double sensor_length[6]
 *                double Pf[6]
 *                double vec[6]
 * Return Type  : void
 */
void stewart_control_function_V4_part_test_1(double* inputData, int outputDataLenger, int* inputSize,
                                              const double sensor_length[6],double Pf[6], double vec[6])
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

  /*  % �˰汾�ǿ��Դ����仯��Ŀ��λ�ˣ���Pf���Ա仯������ֻ��ֻ����һ��    %%%% */
  /*      ���Ǵ˰汾�ٶȽϵͣ����ܽϿ����Ӧ */
  /*  */
  /*  ����ȫ�ֱ���  (��matlab��ȫ�ֱ����ں�������Ӧ�ã�Ҫ���ں���������ж��壬��Ҫ�õ��ĺ�������Ҫ��������֮�������) */
  /*  �����־ñ����������״̬ */
  /*  ������ʼ�� */
  /*     %% ���滮�켣Ϊ�գ�����ǰһ�θ˳�Ϊ����Ϊһ��������ʼ���������� ���⵱Ŀ��λ�˵�λ�����ı仯����2mm��ʱ����Ҫ���³�ʼ���� */
  /* �ٻ��ߵ�Ŀ����̬�����仯����0.5�ȣ��Ƕȣ���ʱ����Ҫ���³�ʼ�� */
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
    /* ֻ�г�ʼ�������������ʱ��Ž��г�ʼ�� */
    /*  --- ���Ʋ��� --- */
    /*          max_speed = 200;    % �����޷� */
    traj->size[0] = 0;
    traj->size[1] = 0;
    traj_not_empty = false;

    /* ��ʼ��Ϊ�վ��� */
    /*          vel = []; %��ʼ��Ϊ�վ��� */
    vel->size[0] = 0;
    vel->size[1] = 0;

    /*  ��ʼ��ǰ�θ˳� */
    /*          pre_length_last = zeros(1,6); % ǰһ�θ���ǰ�ĸ˳�  ���ֵӦ������������������ó�һ��Ϊ0��ȫ�ֱ���������ÿ�ν����ʼ����������ʹ��Ϊ0���ϴθ��µ�����û������%%%%%%%%%%%%%%%%%% */
    time_idx = 1.0;

    /*  */
    converged = false;

    /* �������Ҫ����Ϊȫ�ֱ��� */
    t = 0.0;
    t1 = 0.0;
    flag = 1.0;

    /*          v0 = [40; 19; 38; deg2rad(12); deg2rad(-6); deg2rad(17)]; % ��ʼ�ٶȣ��Ƕ�תΪ���ȣ� */
    for (i = 0; i < 6; i++) {
      pre_length[i] = 0.0;
      v0[i] = b_dv[i];
    }

    /*  ��ʼ�ٶȣ��Ƕ�תΪ���ȣ� */
    /*  MovingPlat ���� */
    /*  StaticPlat ���� */
    memcpy(&MovingPlat[0], &dv[0], 18U * sizeof(double));
    memcpy(&StaticPlat[0], &dv1[0], 18U * sizeof(double));

    /*  �����޷� */
    /* �Ȱ�����ĽǶȴ���������λ�ǽǶȣ� */
    /*          %% �������Ŀ��λ�˵ĵ�λ�ǽǶ�ת���ɻ��� */
    /*  ǰ����Ԫ�أ����ֲ��䣩 */
    /*  �������Ƕ�ֵ����λ���ȣ� */
    /*  ���Ƕ�ת��Ϊ���� */
    /*  ʹ�� deg2rad ����ת */
    /*  �ϲ���� */
    b_Pf[0] = Pf[0];
    b_Pf[3] = 0.017453292519943295 * Pf[3];
    b_Pf[1] = Pf[1];
    b_Pf[4] = 0.017453292519943295 * Pf[4];
    b_Pf[2] = Pf[2];
    b_Pf[5] = 0.017453292519943295 * Pf[5];

    /* ����   */
    /*  �����ʼλ�� */
    for (b_i = 0; b_i < 6; b_i++) {
      Pf_next[b_i] = Pf[b_i];
      Pf[b_i] = b_Pf[b_i];
      Xn_data[b_i] = sensor_length[b_i] + 550.8875;
    }

    PositiveSolution(MovingPlat, StaticPlat, Xn_data, P0, unusedU0);

    /*          % ����ʱ��������������ڳ�ʼ��ʱִ�У� */
    /*  �ٶ�����ֵ */
    /*  ʱ�䲽�� */
    /*          T_original = 5;         % ��ʼ�滮ʱ�� */
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

    /*  ��ʼ���˲����P0 = [];   % ��ʼ���˲����P0 */
  }

  for (i = 0; i < 6; i++) {
    vec[i] = 0.0;
  }

  if (!converged) {
    /*     %% �ջ����ƺ����߼� */
    /*  ���˳��仯(��ֵ1mm)   ��һ����������˳��仯����3mm��ʱ�������½��й滮����Ҫ���ǵ�Pf��Ŀ��λ�ˣ����������� */
    /*                          �ڶ����������Ŀ��λ���е�λ�ñ仯����2mm��������̬�仯����0.5�ȣ���Ҳ�������¹滮��Ŀ��λ�˱仯��ʱ����Ҫ���¹滮�� */
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
      /*  �˳��仯ʱ���¹滮�켣 */
      /*          v0 = [40; 19; 40; deg2rad(12); deg2rad(-6); deg2rad(17)]; % ��ʼ�ٶȣ��Ƕ�תΪ���ȣ� */
      /* disp('1������ɣ��˳�����ѭ���ٶȣ�  '); */
      /*              disp(speed); */
      /* �����ʼ�˳���Ӧ�ĳ�ʼλ�� */
      /*  �����ʼλ��   ������ĳ�ʼλ��ָ����Ϊ�����6���ɶ�˹ͼ����ƽ̨�����⣬���ɸ˳����λ��ʱ����Ҫ�õ��ĵ����Ľ⣩ */
      for (b_i = 0; b_i < 6; b_i++) {
        Xn_data[b_i] = sensor_length[b_i] + 550.8875;
      }

      PositiveSolution(MovingPlat, StaticPlat, Xn_data, P0, unusedU0);

      /* ����Ƕȵĵ�λ�ǻ��� */
      /*          disp('P0_1���˳�����ѭ���ٶȣ�  '); */
      /*          disp(P0); */
      b_Pf[0] = P0[0];
      b_Pf[3] = 0.017453292519943295 * P0[3];
      b_Pf[1] = P0[1];
      b_Pf[4] = 0.017453292519943295 * P0[4];
      b_Pf[2] = P0[2];
      b_Pf[5] = 0.017453292519943295 * P0[5];
      T -= t;

      /*          disp('Pf���˳�����ѭ���ٶȣ�  '); */
      /*          disp(Pf); */
      /*  ��P0�����˲�����������Ӱ�� */
      /*  �˲�ϵ����0~1��ԽСԽƽ���� */
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

      /*  ��Ҫ�ֳ������֣���һ���֣��ǽ��滮�õĹ켣ִ��������һ�Σ�˵���Ѿ����켣������һ��ʱ�䣬�滮�켣��ʵ�ʹ켣�����ˣ�������Ҫ����ʵ�켣����ƴ�� */
      /*                  �ڶ����֣���Ŀ��λ�˱仯�ˣ��͵�һ��Ŀ��λ�˲��䣬����Ҳ���Ѿ�����һ��ʱ���ˣ�����Ҳ��Ҫ����ʵ�켣����ƴ�� */
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

        /* ���������ֵӦ����ʵ�ʵ���������֮�������ʱ�䣨Ӧ�ú���������ʱ�����й�ϵ�� */
        /* �ж���Ҫ�����ٶ�ƴ�ӻ������¿�ʼ�Գ�ʼ�ٶ����� */
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

          /* ����ʵ��λ�˵��ٶ���ʲô    �����λ���ٶȵ�λ�ǻ���   */
        }

        /*             v0 = Calculateposevelocity(MovingPlat, StaticPlat, P0, pre_vec_actual); %����ʵ��λ�˵��ٶ���ʲô    �����λ���ٶȵ�λ�ǻ���    */
      }

      if (flag != 1.0) {
        /* �ж�������иú���ʱ��û�г�ʼ���������ǣ�����нǶ�ת����  */
        /*  ǰ����Ԫ�أ����ֲ��䣩 */
        /* ���Ƕ�ת��Ϊ���� */
        /*  ʹ�� deg2rad ����ת */
        /* �ϲ���� */
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

      /*          disp('Pf���˳�����ѭ���ٶȣ�  '); */
      /*          disp(Pf); */
      /*          disp('v0���˳�����ѭ���ٶȣ�  '); */
      /*          disp(v0); */
      /*          disp('filtered_P0���˳�����ѭ���ٶȣ�  '); */
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

      memset(inputData, 0, outputDataLenger);
      memset(inputSize, 0, 4);

      inputSize[0] = b_vel->size[0];
      inputSize[1] = 6;
      // vel->size[0] = b_vel->size[0];
      // vel->size[1] = 6;
      emxEnsureCapacity_real_T(vel, b_i);
      idx = b_vel->size[0] * b_vel->size[1];

      outputDataLenger = idx;

      //memcpy(inputData, b_vel->data, sizeof(double) * (int)outputDataLenger);
      for (b_i = 0; b_i < idx; b_i++) {
        inputData[b_i] = b_vel->data[b_i];
      }

      /* %%%%%%%%%%%%%%%%% */
      time_idx = 1.0;

      /*  ����ʱ������ */
      for (b_i = 0; b_i < 6; b_i++) {
        pre_length[b_i] = sensor_length[b_i];
      }

      /*  ���¼�¼ֵ */
      t = 0.0;

      /*  �������е�ʱ������ */
      t1 = 0.0;

      /*  �������е�ʱ������ */
    }

    emxFree_real_T(&b_vel);
    emxFree_real_T(&b_traj);
    emxFree_real_T(&unusedU2);

    /*  ��ʼ��������־ */
    /*      converged = false; %�������Ҫ����Ϊȫ�ֱ��� */
    /*  ִ�е�ǰ�켣 */
    if (traj_not_empty && (time_idx <= traj->size[0])) {
      /* size(traj,1)��ʾtraj������ */
      /*  ��ȡ��ǰ�켣�� */
      idx = traj->size[1];
      for (b_i = 0; b_i < idx; b_i++) {
        Xn_data[b_i] = traj->data[((int)time_idx + traj->size[0] * b_i) - 1];
      }

      /*  ������� */
      /*  ����˹ͼ����ƽ̨��֧���ĸ��� */
      /*  ����: */
      /*    MovingPlat - ��ƽ̨���������꣨3��n���� */
      /*    StaticPlat - ��ƽ̨���������꣨3��n���� */
      /*    X - ��ǰλ�� [t; angles] (6��1����) */
      /*    Vn - ƽ̨λ���ٶ� [v; omega] (6��1����) */
      /*  ���: */
      /*    l_dot - ��֧���ĸ��� (n��1����) */
      /*  ������ٵĺ��� */
      /*  �����ſɱȾ��� */
      /*  �����ſɱȾ����ٶ��ſɱȣ� */
      b_t[0] = Xn_data[0];
      b_t[1] = Xn_data[1];
      b_t[2] = Xn_data[2];

      /*  ������ת���� (ZYXŷ����) */
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

      /*  ������٣�l_dot = J' * Vn */
      for (i = 0; i < 6; i++) {
        /*  ��ƽ̨������ֲ����� */
        /*  ��ƽ̨������ȫ������ */
        /*  ��ƽ̨������ȫ������ */
        /*  ֧������ */
        /*  ��λ�������� */
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

        /*  ƽ�Ʋ����ſɱȣ����������� */
        d = l_i[0] / b_rotmat_tmp;
        l_i[0] = d;
        J[i] = d;
        d = l_i[1] / b_rotmat_tmp;
        l_i[1] = d;
        J[i + 6] = d;
        d = l_i[2] / b_rotmat_tmp;
        J[i + 12] = d;

        /*  ��ת�����ſɱȣ�����������˶���λ�ã� */
        J[i + 18] = p_i_tmp[1] * d - p_i_tmp[2] * l_i[1];
        J[i + 24] = p_i_tmp[2] * l_i[0] - p_i_tmp[0] * d;
        J[i + 30] = p_i_tmp[0] * l_i[1] - p_i_tmp[1] * l_i[0];
        d = 0.0;

        for (int b_i = 0; b_i < 6; b_i++) {
          d += J[i + 6 * b_i] * inputData[((int)time_idx + inputSize[0] * b_i) - 1];
        }
        vec[i] = fmin(fmax(d, -20.0), 20.0);
      }

      /*  �޷� */
      time_idx++;

      /*  �ƽ�ʱ������ */
      t += 0.05;
      t1 += 0.05;
    }

    /*  ������⣨���<2mm�� */
    /*  InverseSolution(MovingPlat,SataticPlat,X) ���������˶�ѧ���� */
    /*  MovingPlat ��ƽ̨������ṹ���� */
    /*  SataticPlat ��ƽ̨������ṹ���� */
    /*  X Ϊ��ƽ̨��Ծ�ƽ̨��λ������ */
    /*  Result Ϊ��֧������ */
    /*  ƽ����α任���� */
    memset(&Pt[0], 0, 16U * sizeof(double));
    Pt[0] = 1.0;
    Pt[5] = 1.0;
    Pt[10] = 1.0;
    Pt[15] = 1.0;
    Pt[12] = Pf_next[0];
    Pt[13] = Pf_next[1];
    Pt[14] = Pf_next[2];

    /* ƽ����α任���� */
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

    /*  RPY��ת��α任���󣨽���ת���֣� */
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

    /* RPY��ת��α任 */
    /* ��ʼλ�� ��ƽ̨��Ծ�ƽ̨��λ����α任���� */
    /* ��ƽ̨�������Ϊ�����ʽ */
    for (k = 0; k < 6; k++) {
      idx = k << 2;
      MovingPlat_Hom[idx] = MovingPlat[3 * k];
      MovingPlat_Hom[idx + 1] = MovingPlat[3 * k + 1];
      idx += 3;
      MovingPlat_Hom[idx - 1] = MovingPlat[3 * k + 2];
      MovingPlat_Hom[idx] = 1.0;
    }

    /* ʵ�ʶ�ƽ̨����Ϊ */
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

      /*  �������� */
      traj->size[0] = 0;
      traj->size[1] = 0;
      traj_not_empty = false;

      /* ��ʼ��Ϊ�վ��� */
    }

    /*  ǰһ�θ���ǰ�ĸ˳� */
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
