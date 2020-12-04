/* Include files */

#include "modelInterface.h"
#include "m_8rrCCyrtiOptd4UQNBcUyG.h"
#include <string.h>
#include "mwmathutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 211,   /* lineNo */
  "CarrierSynchronizer",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 1,   /* lineNo */
  "Helper",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\compiled\\+comm\\+internal\\Helper.p"/* pathName */
};

static emlrtRSInfo c_emlrtRSI = { 1,   /* lineNo */
  "System",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\System.p"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 1,   /* lineNo */
  "SystemProp",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemProp.p"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 1,   /* lineNo */
  "SystemCore",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 1,   /* lineNo */
  "Propagates",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\matlab\\system\\+matlab\\+system\\+mixin\\Propagates.p"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 1,   /* lineNo */
  "CustomIcon",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\matlab\\system\\+matlab\\+system\\+mixin\\CustomIcon.p"/* pathName */
};

static emlrtRSInfo h_emlrtRSI = { 12,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo i_emlrtRSI = { 18,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 19,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 20,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo l_emlrtRSI = { 21,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 27,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 239, /* lineNo */
  "CarrierSynchronizer",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m"/* pathName */
};

static emlrtRSInfo o_emlrtRSI = { 76,  /* lineNo */
  "validateattributes",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\lang\\validateattributes.m"/* pathName */
};

static emlrtRSInfo p_emlrtRSI = { 218, /* lineNo */
  "CarrierSynchronizer",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m"/* pathName */
};

static emlrtRSInfo q_emlrtRSI = { 225, /* lineNo */
  "CarrierSynchronizer",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m"/* pathName */
};

static emlrtRSInfo r_emlrtRSI = { 303, /* lineNo */
  "CarrierSynchronizer",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m"/* pathName */
};

static emlrtRSInfo s_emlrtRSI = { 29,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo t_emlrtRSI = { 35,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo u_emlrtRSI = { 44,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 53,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 61,  /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 499, /* lineNo */
  "CarrierSynchronizer",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m"/* pathName */
};

static emlrtMCInfo emlrtMCI = { 14,    /* lineNo */
  37,                                  /* colNo */
  "validatepositive",                  /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatepositive.m"/* pName */
};

static emlrtMCInfo b_emlrtMCI = { 14,  /* lineNo */
  37,                                  /* colNo */
  "validatefinite",                    /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatefinite.m"/* pName */
};

static emlrtMCInfo c_emlrtMCI = { 14,  /* lineNo */
  37,                                  /* colNo */
  "validatenonnan",                    /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatenonnan.m"/* pName */
};

static emlrtMCInfo d_emlrtMCI = { 13,  /* lineNo */
  37,                                  /* colNo */
  "validateinteger",                   /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validateinteger.m"/* pName */
};

static emlrtMCInfo e_emlrtMCI = { 22,  /* lineNo */
  27,                                  /* colNo */
  "validategt",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validategt.m"/* pName */
};

static emlrtMCInfo f_emlrtMCI = { 22,  /* lineNo */
  27,                                  /* colNo */
  "validatele",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatele.m"/* pName */
};

static emlrtMCInfo g_emlrtMCI = { 1,   /* lineNo */
  1,                                   /* colNo */
  "SystemCore",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"/* pName */
};

static emlrtECInfo emlrtECI = { 1,     /* nDims */
  3,                                   /* lineNo */
  4,                                   /* colNo */
  "",                                  /* fName */
  ""                                   /* pName */
};

static emlrtBCInfo emlrtBCI = { -1,    /* iFirst */
  -1,                                  /* iLast */
  355,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "CarrierSynchronizer",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  368,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "CarrierSynchronizer",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  369,                                 /* lineNo */
  30,                                  /* colNo */
  "",                                  /* aName */
  "CarrierSynchronizer",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  355,                                 /* lineNo */
  25,                                  /* colNo */
  "",                                  /* aName */
  "CarrierSynchronizer",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\comm\\comm\\+comm\\CarrierSynchronizer.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRSInfo y_emlrtRSI = { 3,   /* lineNo */
  "",                                  /* fcnName */
  ""                                   /* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 14, /* lineNo */
  "validatenonnan",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatenonnan.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 14, /* lineNo */
  "validatefinite",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatefinite.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 14, /* lineNo */
  "validatepositive",                  /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatepositive.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 13, /* lineNo */
  "validateinteger",                   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validateinteger.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 22, /* lineNo */
  "validatele",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatele.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 22, /* lineNo */
  "validategt",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validategt.m"/* pathName */
};

/* Function Declarations */
static void cgxe_mdl_start(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance);
static void cgxe_mdl_initialize(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);
static void cgxe_mdl_outputs(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);
static void cgxe_mdl_update(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);
static void cgxe_mdl_enable(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);
static void cgxe_mdl_disable(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);
static void cgxe_mdl_terminate(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);
static void mw__internal__call__setup(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T SamplesPerSymbol, real_T
  DampingFactor, real_T NormalizedLoopBandwidth, emxArray_creal_T *u0);
static comm_CarrierSynchronizer *CarrierSynchronizer_CarrierSynchronizer
  (comm_CarrierSynchronizer *obj);
static void CarrierSynchronizer_set_SamplesPerSymbol(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj, real_T value);
static void CarrierSynchronizer_set_DampingFactor(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj, real_T value);
static void validateattributes(const emlrtStack *sp, real_T a);
static void SystemCore_setup(const emlrtStack *sp, comm_CarrierSynchronizer *obj,
  emxArray_creal_T *varargin_1);
static void SystemCore_checkTunablePropChange(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj);
static void mw__internal__call__reset(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T SamplesPerSymbol, real_T
  DampingFactor, real_T NormalizedLoopBandwidth);
static void mw__internal__call__step(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T SamplesPerSymbol, real_T
  DampingFactor, real_T NormalizedLoopBandwidth, emxArray_creal_T *u0,
  emxArray_creal_T *b_y0);
static void mw__internal__system___fcn(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T varargin_3, real_T varargin_4,
  real_T varargin_5, emxArray_creal_T *varargin_7, emxArray_creal_T *varargout_1);
static void CarrierSynchronizer_stepImpl(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj, emxArray_creal_T *input, emxArray_creal_T
  *output);
static const mxArray *emlrt_marshallOut(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp);
static const mxArray *cgxe_mdl_get_sim_state
  (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance);
static void emlrt_marshallIn(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, const mxArray *u);
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_v, const
  char_T *identifier, int32_T y[2]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, int32_T y[2]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_sysobj,
  const char_T *identifier, comm_CarrierSynchronizer *y);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, comm_CarrierSynchronizer *y);
static int32_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static boolean_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, cell_wrap y[1]);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, uint32_T y[8]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[4]);
static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static creal_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static boolean_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *b_sysobj_not_empty, const char_T *identifier);
static void cgxe_mdl_set_sim_state(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const mxArray *st);
static const mxArray *message(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location);
static const mxArray *getString(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);
static void error(const emlrtStack *sp, const mxArray *b, const mxArray *c,
                  emlrtMCInfo *location);
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, const mxArray *d, const mxArray *e, emlrtMCInfo *location);
static const mxArray *c_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, int32_T ret[2]);
static int32_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static boolean_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, uint32_T ret[8]);
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[4]);
static real_T s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static creal_T t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void b_exp(creal_T *x);
static void emxEnsureCapacity_creal_T(emxArray_creal_T *emxArray, int32_T
  oldNumel);
static void emxInit_creal_T(emxArray_creal_T **pEmxArray, int32_T numDimensions);
static void emxFree_creal_T(emxArray_creal_T **pEmxArray);
static void emxInit_creal_T1(emxArray_creal_T **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_creal_T1(emxArray_creal_T *emxArray, int32_T
  oldNumel);
static void init_simulink_io_address(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance);

/* Function Definitions */
static void cgxe_mdl_start(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  emxArray_creal_T *u_tmp3;
  int32_T i;
  int32_T loop_ub;
  real_T *SamplesPerSymbol;
  real_T *DampingFactor;
  real_T *NormalizedLoopBandwidth;
  init_simulink_io_address(moduleInstance);
  NormalizedLoopBandwidth = (real_T *)cgxertGetRunTimeParamInfoData
    (moduleInstance->S, 2);
  DampingFactor = (real_T *)cgxertGetRunTimeParamInfoData(moduleInstance->S, 1);
  SamplesPerSymbol = (real_T *)cgxertGetRunTimeParamInfoData(moduleInstance->S,
    0);
  st.tls = moduleInstance->emlrtRootTLSGlobal;
  emxInit_creal_T(&u_tmp3, 2);
  cgxertSetGcb(moduleInstance->S, -1, -1);
  i = u_tmp3->size[0] * u_tmp3->size[1];
  u_tmp3->size[0] = (*moduleInstance->u0_sizes)[0];
  u_tmp3->size[1] = (*moduleInstance->u0_sizes)[1];
  emxEnsureCapacity_creal_T(u_tmp3, i);
  loop_ub = (*moduleInstance->u0_sizes)[0] * (*moduleInstance->u0_sizes)[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    u_tmp3->data[i].re = (*moduleInstance->u0_data)[i].re;
    u_tmp3->data[i].im = (*moduleInstance->u0_data)[i].im;
  }

  mw__internal__call__setup(moduleInstance, &st, *SamplesPerSymbol,
    *DampingFactor, *NormalizedLoopBandwidth, u_tmp3);
  cgxertRestoreGcb(moduleInstance->S, -1, -1);
  emxFree_creal_T(&u_tmp3);
}

static void cgxe_mdl_initialize(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T *SamplesPerSymbol;
  real_T *DampingFactor;
  real_T *NormalizedLoopBandwidth;
  NormalizedLoopBandwidth = (real_T *)cgxertGetRunTimeParamInfoData
    (moduleInstance->S, 2);
  DampingFactor = (real_T *)cgxertGetRunTimeParamInfoData(moduleInstance->S, 1);
  SamplesPerSymbol = (real_T *)cgxertGetRunTimeParamInfoData(moduleInstance->S,
    0);
  st.tls = moduleInstance->emlrtRootTLSGlobal;
  emlrtLicenseCheckR2012b(&st, "Communication_Toolbox", 2);
  cgxertSetGcb(moduleInstance->S, -1, -1);
  mw__internal__call__reset(moduleInstance, &st, *SamplesPerSymbol,
    *DampingFactor, *NormalizedLoopBandwidth);
  cgxertRestoreGcb(moduleInstance->S, -1, -1);
}

static void cgxe_mdl_outputs(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  emxArray_creal_T *u_tmp3;
  int32_T i;
  int32_T loop_ub;
  emxArray_creal_T *y_tmp0;
  real_T *SamplesPerSymbol;
  real_T *DampingFactor;
  real_T *NormalizedLoopBandwidth;
  NormalizedLoopBandwidth = (real_T *)cgxertGetRunTimeParamInfoData
    (moduleInstance->S, 2);
  DampingFactor = (real_T *)cgxertGetRunTimeParamInfoData(moduleInstance->S, 1);
  SamplesPerSymbol = (real_T *)cgxertGetRunTimeParamInfoData(moduleInstance->S,
    0);
  st.tls = moduleInstance->emlrtRootTLSGlobal;
  emxInit_creal_T(&u_tmp3, 2);
  cgxertSetGcb(moduleInstance->S, -1, -1);
  cgxertCheckCurrentInputPortDimensions(moduleInstance->S,
    "comm.CarrierSynchronizer", "stepImpl");
  i = u_tmp3->size[0] * u_tmp3->size[1];
  u_tmp3->size[0] = (*moduleInstance->u0_sizes)[0];
  u_tmp3->size[1] = (*moduleInstance->u0_sizes)[1];
  emxEnsureCapacity_creal_T(u_tmp3, i);
  loop_ub = (*moduleInstance->u0_sizes)[0] * (*moduleInstance->u0_sizes)[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    u_tmp3->data[i].re = (*moduleInstance->u0_data)[i].re;
    u_tmp3->data[i].im = (*moduleInstance->u0_data)[i].im;
  }

  emxInit_creal_T(&y_tmp0, 2);
  mw__internal__call__step(moduleInstance, &st, *SamplesPerSymbol,
    *DampingFactor, *NormalizedLoopBandwidth, u_tmp3, y_tmp0);
  cgxertSetCurrentOutputPortDimensions(moduleInstance->S, 0, 0, y_tmp0->size[0]);
  cgxertSetCurrentOutputPortDimensions(moduleInstance->S, 0, 1, 1);
  loop_ub = y_tmp0->size[0] - 1;
  emxFree_creal_T(&u_tmp3);
  for (i = 0; i <= loop_ub; i++) {
    (*moduleInstance->y0_data)[i].re = y_tmp0->data[i].re;
    (*moduleInstance->y0_data)[i].im = y_tmp0->data[i].im;
  }

  emxFree_creal_T(&y_tmp0);
  cgxertRestoreGcb(moduleInstance->S, -1, -1);
}

static void cgxe_mdl_update(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_enable(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_disable(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_terminate(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  cgxertSetGcb(moduleInstance->S, -1, -1);
  cgxertRestoreGcb(moduleInstance->S, -1, -1);
}

static void mw__internal__call__setup(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T SamplesPerSymbol, real_T
  DampingFactor, real_T NormalizedLoopBandwidth, emxArray_creal_T *u0)
{
  emlrtStack st;
  emlrtStack b_st;
  int32_T u0_idx_0;
  emxArray_creal_T b_u0;
  int32_T c_u0[1];
  comm_CarrierSynchronizer *obj;
  emlrtStack c_st;
  boolean_T flag;
  static char_T correctedValue[4] = { 'A', 'u', 't', 'o' };

  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (!moduleInstance->sysobj_not_empty) {
    st.site = &h_emlrtRSI;
    CarrierSynchronizer_CarrierSynchronizer(&moduleInstance->sysobj);
    moduleInstance->sysobj_not_empty = true;
    st.site = &i_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    c_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    for (u0_idx_0 = 0; u0_idx_0 < 4; u0_idx_0++) {
      obj->ModulationPhaseOffset[u0_idx_0] = correctedValue[u0_idx_0];
    }

    st.site = &j_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &j_emlrtRSI;
    CarrierSynchronizer_set_SamplesPerSymbol(&st, &moduleInstance->sysobj,
      SamplesPerSymbol);
    st.site = &k_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &k_emlrtRSI;
    CarrierSynchronizer_set_DampingFactor(&st, &moduleInstance->sysobj,
      DampingFactor);
    st.site = &l_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &l_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &q_emlrtRSI;
    validateattributes(&b_st, NormalizedLoopBandwidth);
    obj->NormalizedLoopBandwidth = NormalizedLoopBandwidth;
  }

  u0_idx_0 = u0->size[0];
  b_u0 = *u0;
  c_u0[0] = u0_idx_0;
  b_u0.size = &c_u0[0];
  b_u0.numDimensions = 1;
  st.site = &m_emlrtRSI;
  SystemCore_setup(&st, &moduleInstance->sysobj, &b_u0);
}

static comm_CarrierSynchronizer *CarrierSynchronizer_CarrierSynchronizer
  (comm_CarrierSynchronizer *obj)
{
  comm_CarrierSynchronizer *b_obj;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  b_obj = obj;
  b_obj->CustomPhaseOffset = 0.0;
  st.site = &emlrtRSI;
  b_st.site = &b_emlrtRSI;
  c_st.site = &c_emlrtRSI;
  d_st.site = &d_emlrtRSI;
  c_st.site = &c_emlrtRSI;
  b_obj->isInitialized = 0;
  b_obj->TunablePropsChanged = false;
  b_obj->CacheInputSizes = false;
  d_st.site = &e_emlrtRSI;
  st.site = &emlrtRSI;
  b_st.site = &f_emlrtRSI;
  st.site = &emlrtRSI;
  b_st.site = &g_emlrtRSI;
  return b_obj;
}

static void CarrierSynchronizer_set_SamplesPerSymbol(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj, real_T value)
{
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  boolean_T p;
  char_T u[23];
  int32_T i;
  const mxArray *y;
  static char_T b_u[23] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'P', 'o', 's', 'i', 't', 'i', 'v', 'e' };

  const mxArray *m;
  static const int32_T iv[2] = { 1, 23 };

  char_T c_u[21];
  char_T d_u[48];
  static char_T e_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'F', 'i', 'n', 'i', 't', 'e' };

  static const int32_T iv1[2] = { 1, 21 };

  const mxArray *b_y;
  static char_T f_u[48] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'P', 'o', 's', 'i', 't', 'i', 'v', 'e' };

  static const int32_T iv2[2] = { 1, 48 };

  char_T g_u[46];
  static char_T h_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'N', 'o', 'n', 'N', 'a', 'N' };

  char_T i_u[16];
  static const int32_T iv3[2] = { 1, 21 };

  char_T j_u[22];
  static char_T k_u[46] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'F', 'i', 'n', 'i', 't', 'e' };

  static const int32_T iv4[2] = { 1, 46 };

  const mxArray *c_y;
  static char_T l_u[16] = { 'S', 'a', 'm', 'p', 'l', 'e', 's', 'P', 'e', 'r',
    'S', 'y', 'm', 'b', 'o', 'l' };

  static const int32_T iv5[2] = { 1, 16 };

  static char_T m_u[22] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'I', 'n', 't', 'e', 'g', 'e', 'r' };

  static const int32_T iv6[2] = { 1, 22 };

  static char_T n_u[46] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'N', 'o', 'n', 'N', 'a', 'N' };

  static const int32_T iv7[2] = { 1, 46 };

  char_T o_u[47];
  static const int32_T iv8[2] = { 1, 16 };

  static char_T p_u[47] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'I', 'n', 't', 'e', 'g', 'e', 'r' };

  static const int32_T iv9[2] = { 1, 47 };

  static const int32_T iv10[2] = { 1, 16 };

  static const int32_T iv11[2] = { 1, 16 };

  st.prev = sp;
  st.tls = sp->tls;
  st.site = &n_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &o_emlrtRSI;
  p = true;
  if (value <= 0.0) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 23; i++) {
      u[i] = b_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&b_st, 23, m, &u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 48; i++) {
      d_u[i] = f_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&b_st, 48, m, &d_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 16; i++) {
      i_u[i] = l_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv5[0]);
    emlrtInitCharArrayR2013a(&b_st, 16, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &cb_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &emlrtMCI),
           &emlrtMCI), &emlrtMCI);
  }

  b_st.site = &o_emlrtRSI;
  p = true;
  if (!((!muDoubleScalarIsInf(value)) && (!muDoubleScalarIsNaN(value)))) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 21; i++) {
      c_u[i] = e_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&b_st, 21, m, &c_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 46; i++) {
      g_u[i] = k_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv4[0]);
    emlrtInitCharArrayR2013a(&b_st, 46, m, &g_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 16; i++) {
      i_u[i] = l_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv8[0]);
    emlrtInitCharArrayR2013a(&b_st, 16, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &bb_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &b_emlrtMCI),
           &b_emlrtMCI), &b_emlrtMCI);
  }

  b_st.site = &o_emlrtRSI;
  p = true;
  if (muDoubleScalarIsNaN(value)) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 21; i++) {
      c_u[i] = h_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&b_st, 21, m, &c_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 46; i++) {
      g_u[i] = n_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv7[0]);
    emlrtInitCharArrayR2013a(&b_st, 46, m, &g_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 16; i++) {
      i_u[i] = l_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv10[0]);
    emlrtInitCharArrayR2013a(&b_st, 16, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &ab_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &c_emlrtMCI),
           &c_emlrtMCI), &c_emlrtMCI);
  }

  b_st.site = &o_emlrtRSI;
  p = true;
  if ((!muDoubleScalarIsInf(value)) && (!muDoubleScalarIsNaN(value)) &&
      (muDoubleScalarFloor(value) == value)) {
  } else {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 22; i++) {
      j_u[i] = m_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv6[0]);
    emlrtInitCharArrayR2013a(&b_st, 22, m, &j_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 47; i++) {
      o_u[i] = p_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv9[0]);
    emlrtInitCharArrayR2013a(&b_st, 47, m, &o_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 16; i++) {
      i_u[i] = l_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv11[0]);
    emlrtInitCharArrayR2013a(&b_st, 16, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &db_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &d_emlrtMCI),
           &d_emlrtMCI), &d_emlrtMCI);
  }

  obj->SamplesPerSymbol = value;
}

static void CarrierSynchronizer_set_DampingFactor(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj, real_T value)
{
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  boolean_T p;
  char_T u[23];
  int32_T i;
  const mxArray *y;
  static char_T b_u[23] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'P', 'o', 's', 'i', 't', 'i', 'v', 'e' };

  const mxArray *m;
  static const int32_T iv[2] = { 1, 23 };

  char_T c_u[21];
  char_T d_u[48];
  static char_T e_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'F', 'i', 'n', 'i', 't', 'e' };

  static const int32_T iv1[2] = { 1, 21 };

  const mxArray *b_y;
  static char_T f_u[48] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'P', 'o', 's', 'i', 't', 'i', 'v', 'e' };

  static const int32_T iv2[2] = { 1, 48 };

  char_T g_u[46];
  static char_T h_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'N', 'o', 'n', 'N', 'a', 'N' };

  char_T i_u[13];
  static const int32_T iv3[2] = { 1, 21 };

  static char_T j_u[46] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'F', 'i', 'n', 'i', 't', 'e' };

  static const int32_T iv4[2] = { 1, 46 };

  const mxArray *c_y;
  static char_T k_u[13] = { 'D', 'a', 'm', 'p', 'i', 'n', 'g', 'F', 'a', 'c',
    't', 'o', 'r' };

  static const int32_T iv5[2] = { 1, 13 };

  static char_T l_u[46] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'N', 'o', 'n', 'N', 'a', 'N' };

  static const int32_T iv6[2] = { 1, 46 };

  static const int32_T iv7[2] = { 1, 13 };

  static const int32_T iv8[2] = { 1, 13 };

  st.prev = sp;
  st.tls = sp->tls;
  st.site = &p_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &o_emlrtRSI;
  p = true;
  if (value <= 0.0) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 23; i++) {
      u[i] = b_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&b_st, 23, m, &u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 48; i++) {
      d_u[i] = f_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&b_st, 48, m, &d_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 13; i++) {
      i_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv5[0]);
    emlrtInitCharArrayR2013a(&b_st, 13, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &cb_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &emlrtMCI),
           &emlrtMCI), &emlrtMCI);
  }

  b_st.site = &o_emlrtRSI;
  p = true;
  if (!((!muDoubleScalarIsInf(value)) && (!muDoubleScalarIsNaN(value)))) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 21; i++) {
      c_u[i] = e_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&b_st, 21, m, &c_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 46; i++) {
      g_u[i] = j_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv4[0]);
    emlrtInitCharArrayR2013a(&b_st, 46, m, &g_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 13; i++) {
      i_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv7[0]);
    emlrtInitCharArrayR2013a(&b_st, 13, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &bb_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &b_emlrtMCI),
           &b_emlrtMCI), &b_emlrtMCI);
  }

  b_st.site = &o_emlrtRSI;
  p = true;
  if (muDoubleScalarIsNaN(value)) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 21; i++) {
      c_u[i] = h_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&b_st, 21, m, &c_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 46; i++) {
      g_u[i] = l_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv6[0]);
    emlrtInitCharArrayR2013a(&b_st, 46, m, &g_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 13; i++) {
      i_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv8[0]);
    emlrtInitCharArrayR2013a(&b_st, 13, m, &i_u[0]);
    emlrtAssign(&c_y, m);
    c_st.site = &ab_emlrtRSI;
    error(&c_st, y, getString(&c_st, message(&c_st, b_y, c_y, &c_emlrtMCI),
           &c_emlrtMCI), &c_emlrtMCI);
  }

  obj->DampingFactor = value;
}

static void validateattributes(const emlrtStack *sp, real_T a)
{
  emlrtStack st;
  emlrtStack b_st;
  boolean_T p;
  char_T u[17];
  int32_T i;
  const mxArray *y;
  static char_T b_u[17] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'o', 't',
    'G', 'r', 'e', 'a', 't', 'e', 'r' };

  const mxArray *m;
  static const int32_T iv[2] = { 1, 17 };

  char_T c_u[19];
  char_T d_u[40];
  static char_T e_u[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'o', 't',
    'L', 'e', 's', 's', 'E', 'q', 'u', 'a', 'l' };

  static const int32_T iv1[2] = { 1, 19 };

  char_T f_u[23];
  const mxArray *b_y;
  static char_T g_u[40] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'v', 'a', 'l',
    'i', 'd', 'a', 't', 'e', 'a', 't', 't', 'r', 'i', 'b', 'u', 't', 'e', 's',
    ':', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd', 'S', 'c', 'a', 'l', 'a', 'r' };

  static const int32_T iv2[2] = { 1, 40 };

  static char_T h_u[23] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'P', 'o', 's', 'i', 't', 'i', 'v', 'e' };

  static const int32_T iv3[2] = { 1, 23 };

  char_T i_u[21];
  static const int32_T iv4[2] = { 1, 40 };

  char_T j_u[48];
  const mxArray *c_y;
  static char_T k_u[23] = { 'N', 'o', 'r', 'm', 'a', 'l', 'i', 'z', 'e', 'd',
    'L', 'o', 'o', 'p', 'B', 'a', 'n', 'd', 'w', 'i', 'd', 't', 'h' };

  static const int32_T iv5[2] = { 1, 23 };

  static char_T l_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'F', 'i', 'n', 'i', 't', 'e' };

  static const int32_T iv6[2] = { 1, 21 };

  static char_T m_u[48] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'P', 'o', 's', 'i', 't', 'i', 'v', 'e' };

  const mxArray *d_y;
  static const int32_T iv7[2] = { 1, 48 };

  char_T n_u[46];
  static const int32_T iv8[2] = { 1, 23 };

  const mxArray *e_y;
  static char_T o_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'e', 'x', 'p',
    'e', 'c', 't', 'e', 'd', 'N', 'o', 'n', 'N', 'a', 'N' };

  static const int32_T iv9[2] = { 1, 21 };

  static char_T p_u[46] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'F', 'i', 'n', 'i', 't', 'e' };

  char_T q_u[2];
  static const int32_T iv10[2] = { 1, 46 };

  static const int32_T iv11[2] = { 1, 23 };

  static char_T r_u[2] = { '<', '=' };

  static const int32_T iv12[2] = { 1, 2 };

  static char_T s_u[46] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'V', 'a', 'l', 'i', 'd', 'a', 't', 'e', 'a', 't', 't',
    'r', 'i', 'b', 'u', 't', 'e', 's', 'e', 'x', 'p', 'e', 'c', 't', 'e', 'd',
    'N', 'o', 'n', 'N', 'a', 'N' };

  static const int32_T iv13[2] = { 1, 46 };

  static const int32_T iv14[2] = { 1, 23 };

  static const int32_T iv15[2] = { 1, 23 };

  st.prev = sp;
  st.tls = sp->tls;
  st.site = &o_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  p = true;
  if (!(a > 0.0)) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 17; i++) {
      u[i] = b_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&st, 17, m, &u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 40; i++) {
      d_u[i] = g_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&st, 40, m, &d_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 23; i++) {
      f_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv5[0]);
    emlrtInitCharArrayR2013a(&st, 23, m, &f_u[0]);
    emlrtAssign(&c_y, m);
    d_y = NULL;
    m = emlrtCreateString1('>');
    emlrtAssign(&d_y, m);
    e_y = NULL;
    m = emlrtCreateString1('0');
    emlrtAssign(&e_y, m);
    b_st.site = &fb_emlrtRSI;
    error(&b_st, y, getString(&b_st, b_message(&b_st, b_y, c_y, d_y, e_y,
            &e_emlrtMCI), &e_emlrtMCI), &e_emlrtMCI);
  }

  st.site = &o_emlrtRSI;
  p = true;
  if (!(a <= 1.0)) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 19; i++) {
      c_u[i] = e_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&st, 19, m, &c_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 40; i++) {
      d_u[i] = g_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv4[0]);
    emlrtInitCharArrayR2013a(&st, 40, m, &d_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 23; i++) {
      f_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv8[0]);
    emlrtInitCharArrayR2013a(&st, 23, m, &f_u[0]);
    emlrtAssign(&c_y, m);
    for (i = 0; i < 2; i++) {
      q_u[i] = r_u[i];
    }

    d_y = NULL;
    m = emlrtCreateCharArray(2, &iv12[0]);
    emlrtInitCharArrayR2013a(&st, 2, m, &q_u[0]);
    emlrtAssign(&d_y, m);
    e_y = NULL;
    m = emlrtCreateString1('1');
    emlrtAssign(&e_y, m);
    b_st.site = &eb_emlrtRSI;
    error(&b_st, y, getString(&b_st, b_message(&b_st, b_y, c_y, d_y, e_y,
            &f_emlrtMCI), &f_emlrtMCI), &f_emlrtMCI);
  }

  st.site = &o_emlrtRSI;
  p = true;
  if (a <= 0.0) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 23; i++) {
      f_u[i] = h_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&st, 23, m, &f_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 48; i++) {
      j_u[i] = m_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv7[0]);
    emlrtInitCharArrayR2013a(&st, 48, m, &j_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 23; i++) {
      f_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv11[0]);
    emlrtInitCharArrayR2013a(&st, 23, m, &f_u[0]);
    emlrtAssign(&c_y, m);
    b_st.site = &cb_emlrtRSI;
    error(&b_st, y, getString(&b_st, message(&b_st, b_y, c_y, &emlrtMCI),
           &emlrtMCI), &emlrtMCI);
  }

  st.site = &o_emlrtRSI;
  p = true;
  if (!((!muDoubleScalarIsInf(a)) && (!muDoubleScalarIsNaN(a)))) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 21; i++) {
      i_u[i] = l_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv6[0]);
    emlrtInitCharArrayR2013a(&st, 21, m, &i_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 46; i++) {
      n_u[i] = p_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv10[0]);
    emlrtInitCharArrayR2013a(&st, 46, m, &n_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 23; i++) {
      f_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv14[0]);
    emlrtInitCharArrayR2013a(&st, 23, m, &f_u[0]);
    emlrtAssign(&c_y, m);
    b_st.site = &bb_emlrtRSI;
    error(&b_st, y, getString(&b_st, message(&b_st, b_y, c_y, &b_emlrtMCI),
           &b_emlrtMCI), &b_emlrtMCI);
  }

  st.site = &o_emlrtRSI;
  p = true;
  if (muDoubleScalarIsNaN(a)) {
    p = false;
  }

  if (!p) {
    for (i = 0; i < 21; i++) {
      i_u[i] = o_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv9[0]);
    emlrtInitCharArrayR2013a(&st, 21, m, &i_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 46; i++) {
      n_u[i] = s_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv13[0]);
    emlrtInitCharArrayR2013a(&st, 46, m, &n_u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 23; i++) {
      f_u[i] = k_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv15[0]);
    emlrtInitCharArrayR2013a(&st, 23, m, &f_u[0]);
    emlrtAssign(&c_y, m);
    b_st.site = &ab_emlrtRSI;
    error(&b_st, y, getString(&b_st, message(&b_st, b_y, c_y, &c_emlrtMCI),
           &c_emlrtMCI), &c_emlrtMCI);
  }
}

static void SystemCore_setup(const emlrtStack *sp, comm_CarrierSynchronizer *obj,
  emxArray_creal_T *varargin_1)
{
  emlrtStack st;
  char_T u[51];
  int32_T ret;
  cell_wrap varSizes[1];
  const mxArray *y;
  static char_T b_u[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  const mxArray *m;
  static const int32_T iv[2] = { 1, 51 };

  const mxArray *b_y;
  emlrtStack b_st;
  static const int32_T iv1[2] = { 1, 51 };

  real_T PhaseRecoveryLoopBandwidth;
  real_T PhaseRecoveryGain;
  char_T c_u[5];
  real_T d;
  char_T a[4];
  const mxArray *c_y;
  static char_T d_u[5] = { 's', 'e', 't', 'u', 'p' };

  static const int32_T iv2[2] = { 1, 5 };

  char_T b[4];
  static char_T b_b[4] = { 'A', 'u', 't', 'o' };

  st.prev = sp;
  st.tls = sp->tls;
  if (obj->isInitialized != 0) {
    for (ret = 0; ret < 51; ret++) {
      u[ret] = b_u[ret];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(sp, 51, m, &u[0]);
    emlrtAssign(&y, m);
    for (ret = 0; ret < 51; ret++) {
      u[ret] = b_u[ret];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(sp, 51, m, &u[0]);
    emlrtAssign(&b_y, m);
    for (ret = 0; ret < 5; ret++) {
      c_u[ret] = d_u[ret];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(sp, 5, m, &c_u[0]);
    emlrtAssign(&c_y, m);
    st.site = &e_emlrtRSI;
    error(&st, y, getString(&st, message(&st, b_y, c_y, &g_emlrtMCI),
           &g_emlrtMCI), &g_emlrtMCI);
  }

  obj->isInitialized = 1;
  st.site = &e_emlrtRSI;
  varSizes[0].f1[0] = (uint32_T)varargin_1->size[0];
  varSizes[0].f1[1] = 1U;
  for (ret = 0; ret < 6; ret++) {
    varSizes[0].f1[ret + 2] = 1U;
  }

  obj->inputVarSize[0] = varSizes[0];
  st.site = &e_emlrtRSI;
  obj->pPhase = 0.0;
  obj->pPreviousSample.re = 0.0;
  obj->pPreviousSample.im = 0.0;
  b_st.site = &r_emlrtRSI;
  PhaseRecoveryLoopBandwidth = obj->NormalizedLoopBandwidth *
    obj->SamplesPerSymbol;
  PhaseRecoveryGain = obj->SamplesPerSymbol;
  PhaseRecoveryLoopBandwidth /= (obj->DampingFactor + 0.25 / obj->DampingFactor)
    * obj->SamplesPerSymbol;
  d = (2.0 * obj->DampingFactor * PhaseRecoveryLoopBandwidth + 1.0) +
    PhaseRecoveryLoopBandwidth * PhaseRecoveryLoopBandwidth;
  obj->pProportionalGain = 4.0 * obj->DampingFactor * PhaseRecoveryLoopBandwidth
    / d / PhaseRecoveryGain;
  obj->pIntegratorGain = 4.0 * PhaseRecoveryLoopBandwidth *
    PhaseRecoveryLoopBandwidth / d / PhaseRecoveryGain;
  for (ret = 0; ret < 4; ret++) {
    a[ret] = obj->ModulationPhaseOffset[ret];
  }

  for (ret = 0; ret < 4; ret++) {
    b[ret] = b_b[ret];
  }

  ret = memcmp(&a[0], &b[0], 4);
  if (ret == 0) {
    obj->pActualPhaseOffset = 0.0;
  } else {
    obj->pActualPhaseOffset = obj->CustomPhaseOffset;
  }

  obj->pDigitalSynthesizerGain = -1.0;
  st.site = &e_emlrtRSI;
  SystemCore_checkTunablePropChange(&st, obj);
  obj->TunablePropsChanged = false;
}

static void SystemCore_checkTunablePropChange(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj)
{
  emlrtStack st;
  char_T u[44];
  int32_T i;
  const mxArray *y;
  static char_T b_u[44] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'i', 'n', 'v', 'a', 'l', 'i', 'd', 'T', 'u', 'n', 'a',
    'b', 'l', 'e', 'M', 'o', 'd', 'A', 'c', 'c', 'e', 's', 's', 'C', 'o', 'd',
    'e', 'g', 'e', 'n' };

  const mxArray *m;
  static const int32_T iv[2] = { 1, 44 };

  const mxArray *b_y;
  static const int32_T iv1[2] = { 1, 44 };

  st.prev = sp;
  st.tls = sp->tls;
  if (obj->TunablePropsChanged) {
    for (i = 0; i < 44; i++) {
      u[i] = b_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(sp, 44, m, &u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 44; i++) {
      u[i] = b_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(sp, 44, m, &u[0]);
    emlrtAssign(&b_y, m);
    st.site = &e_emlrtRSI;
    error(&st, y, getString(&st, c_message(&st, b_y, &g_emlrtMCI), &g_emlrtMCI),
          &g_emlrtMCI);
  }
}

static void mw__internal__call__reset(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T SamplesPerSymbol, real_T
  DampingFactor, real_T NormalizedLoopBandwidth)
{
  emlrtStack st;
  emlrtStack b_st;
  comm_CarrierSynchronizer *obj;
  char_T u[45];
  boolean_T tunablePropChangedBeforeResetImpl;
  int32_T i;
  emlrtStack c_st;
  const mxArray *y;
  static char_T b_u[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  char_T c_u[44];
  const mxArray *m;
  static const int32_T iv[2] = { 1, 45 };

  static char_T d_u[44] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'i', 'n', 'v', 'a', 'l', 'i', 'd', 'T', 'u', 'n', 'a',
    'b', 'l', 'e', 'M', 'o', 'd', 'A', 'c', 'c', 'e', 's', 's', 'C', 'o', 'd',
    'e', 'g', 'e', 'n' };

  static char_T correctedValue[4] = { 'A', 'u', 't', 'o' };

  static const int32_T iv1[2] = { 1, 44 };

  const mxArray *b_y;
  static const int32_T iv2[2] = { 1, 45 };

  char_T e_u[5];
  static const int32_T iv3[2] = { 1, 44 };

  const mxArray *c_y;
  static char_T f_u[5] = { 'r', 'e', 's', 'e', 't' };

  static const int32_T iv4[2] = { 1, 5 };

  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (!moduleInstance->sysobj_not_empty) {
    st.site = &h_emlrtRSI;
    CarrierSynchronizer_CarrierSynchronizer(&moduleInstance->sysobj);
    moduleInstance->sysobj_not_empty = true;
    st.site = &i_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    c_st.site = &d_emlrtRSI;
    tunablePropChangedBeforeResetImpl = (obj->isInitialized == 1);
    if (tunablePropChangedBeforeResetImpl) {
      obj->TunablePropsChanged = true;
    }

    for (i = 0; i < 4; i++) {
      obj->ModulationPhaseOffset[i] = correctedValue[i];
    }

    st.site = &j_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    tunablePropChangedBeforeResetImpl = (obj->isInitialized == 1);
    if (tunablePropChangedBeforeResetImpl) {
      obj->TunablePropsChanged = true;
    }

    st.site = &j_emlrtRSI;
    CarrierSynchronizer_set_SamplesPerSymbol(&st, &moduleInstance->sysobj,
      SamplesPerSymbol);
    st.site = &k_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    tunablePropChangedBeforeResetImpl = (obj->isInitialized == 1);
    if (tunablePropChangedBeforeResetImpl) {
      obj->TunablePropsChanged = true;
    }

    st.site = &k_emlrtRSI;
    CarrierSynchronizer_set_DampingFactor(&st, &moduleInstance->sysobj,
      DampingFactor);
    st.site = &l_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    tunablePropChangedBeforeResetImpl = (obj->isInitialized == 1);
    if (tunablePropChangedBeforeResetImpl) {
      obj->TunablePropsChanged = true;
    }

    st.site = &l_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &q_emlrtRSI;
    validateattributes(&b_st, NormalizedLoopBandwidth);
    obj->NormalizedLoopBandwidth = NormalizedLoopBandwidth;
  }

  st.site = &s_emlrtRSI;
  obj = &moduleInstance->sysobj;
  if (moduleInstance->sysobj.isInitialized == 2) {
    for (i = 0; i < 45; i++) {
      u[i] = b_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&st, 45, m, &u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 45; i++) {
      u[i] = b_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&st, 45, m, &u[0]);
    emlrtAssign(&b_y, m);
    for (i = 0; i < 5; i++) {
      e_u[i] = f_u[i];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv4[0]);
    emlrtInitCharArrayR2013a(&st, 5, m, &e_u[0]);
    emlrtAssign(&c_y, m);
    b_st.site = &e_emlrtRSI;
    error(&b_st, y, getString(&b_st, message(&b_st, b_y, c_y, &g_emlrtMCI),
           &g_emlrtMCI), &g_emlrtMCI);
  }

  tunablePropChangedBeforeResetImpl = obj->TunablePropsChanged;
  if (obj->isInitialized == 1) {
    b_st.site = &e_emlrtRSI;
    obj->pLoopFilterState = 0.0;
    obj->pIntegFilterState = 0.0;
    obj->pDDSPreviousInput = 0.0;
    obj->pPhase = 0.0;
    obj->pPreviousSample.re = 0.0;
    obj->pPreviousSample.im = 0.0;
  }

  if ((int32_T)tunablePropChangedBeforeResetImpl != (int32_T)
      obj->TunablePropsChanged) {
    for (i = 0; i < 44; i++) {
      c_u[i] = d_u[i];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&st, 44, m, &c_u[0]);
    emlrtAssign(&y, m);
    for (i = 0; i < 44; i++) {
      c_u[i] = d_u[i];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&st, 44, m, &c_u[0]);
    emlrtAssign(&b_y, m);
    b_st.site = &e_emlrtRSI;
    error(&b_st, y, getString(&b_st, c_message(&b_st, b_y, &g_emlrtMCI),
           &g_emlrtMCI), &g_emlrtMCI);
  }
}

static void mw__internal__call__step(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T SamplesPerSymbol, real_T
  DampingFactor, real_T NormalizedLoopBandwidth, emxArray_creal_T *u0,
  emxArray_creal_T *b_y0)
{
  emlrtStack st;
  emxArray_creal_T *c_y0;
  int32_T u0_idx_0;
  emxArray_creal_T b_u0;
  int32_T c_u0[1];
  int32_T loop_ub;
  st.prev = sp;
  st.tls = sp->tls;
  emxInit_creal_T1(&c_y0, 1);
  u0_idx_0 = u0->size[0];
  b_u0 = *u0;
  c_u0[0] = u0_idx_0;
  b_u0.size = &c_u0[0];
  b_u0.numDimensions = 1;
  st.site = &y_emlrtRSI;
  mw__internal__system___fcn(moduleInstance, &st, SamplesPerSymbol,
    DampingFactor, NormalizedLoopBandwidth, &b_u0, c_y0);
  u0_idx_0 = c_y0->size[0];
  emlrtDimSizeGeqCheckR2012b(6175, u0_idx_0, &emlrtECI, sp);
  u0_idx_0 = b_y0->size[0] * b_y0->size[1];
  b_y0->size[0] = c_y0->size[0];
  b_y0->size[1] = 1;
  emxEnsureCapacity_creal_T(b_y0, u0_idx_0);
  loop_ub = c_y0->size[0] - 1;
  for (u0_idx_0 = 0; u0_idx_0 <= loop_ub; u0_idx_0++) {
    b_y0->data[u0_idx_0].re = c_y0->data[u0_idx_0].re;
    b_y0->data[u0_idx_0].im = c_y0->data[u0_idx_0].im;
  }

  emxFree_creal_T(&c_y0);
}

static void mw__internal__system___fcn(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, real_T varargin_3, real_T varargin_4,
  real_T varargin_5, emxArray_creal_T *varargin_7, emxArray_creal_T *varargout_1)
{
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  comm_CarrierSynchronizer *obj;
  boolean_T flag;
  char_T u[45];
  int32_T ret;
  static char_T correctedValue[4] = { 'A', 'u', 't', 'o' };

  const mxArray *y;
  static char_T b_u[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  const mxArray *m;
  static const int32_T iv[2] = { 1, 45 };

  cell_wrap varSizes[1];
  int16_T inSize[8];
  emlrtStack d_st;
  real_T PhaseRecoveryLoopBandwidth;
  real_T PhaseRecoveryGain;
  const mxArray *b_y;
  static const int32_T iv1[2] = { 1, 45 };

  real_T d;
  char_T c_u[4];
  boolean_T exitg1;
  const mxArray *c_y;
  static char_T d_u[4] = { 's', 't', 'e', 'p' };

  char_T b[4];
  static const int32_T iv2[2] = { 1, 4 };

  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  if (!moduleInstance->sysobj_not_empty) {
    st.site = &h_emlrtRSI;
    CarrierSynchronizer_CarrierSynchronizer(&moduleInstance->sysobj);
    moduleInstance->sysobj_not_empty = true;
    st.site = &i_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    c_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    for (ret = 0; ret < 4; ret++) {
      obj->ModulationPhaseOffset[ret] = correctedValue[ret];
    }

    st.site = &j_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &j_emlrtRSI;
    CarrierSynchronizer_set_SamplesPerSymbol(&st, &moduleInstance->sysobj,
      varargin_3);
    st.site = &k_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &k_emlrtRSI;
    CarrierSynchronizer_set_DampingFactor(&st, &moduleInstance->sysobj,
      varargin_4);
    st.site = &l_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &l_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &q_emlrtRSI;
    validateattributes(&b_st, varargin_5);
    obj->NormalizedLoopBandwidth = varargin_5;
  }

  if (moduleInstance->sysobj.SamplesPerSymbol != varargin_3) {
    st.site = &t_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &t_emlrtRSI;
    CarrierSynchronizer_set_SamplesPerSymbol(&st, &moduleInstance->sysobj,
      varargin_3);
  }

  if (moduleInstance->sysobj.DampingFactor != varargin_4) {
    st.site = &u_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &u_emlrtRSI;
    CarrierSynchronizer_set_DampingFactor(&st, &moduleInstance->sysobj,
      varargin_4);
  }

  if (moduleInstance->sysobj.NormalizedLoopBandwidth != varargin_5) {
    st.site = &v_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &d_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
    }

    st.site = &v_emlrtRSI;
    obj = &moduleInstance->sysobj;
    b_st.site = &q_emlrtRSI;
    validateattributes(&b_st, varargin_5);
    obj->NormalizedLoopBandwidth = varargin_5;
  }

  st.site = &w_emlrtRSI;
  obj = &moduleInstance->sysobj;
  if (moduleInstance->sysobj.isInitialized == 2) {
    for (ret = 0; ret < 45; ret++) {
      u[ret] = b_u[ret];
    }

    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&st, 45, m, &u[0]);
    emlrtAssign(&y, m);
    for (ret = 0; ret < 45; ret++) {
      u[ret] = b_u[ret];
    }

    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&st, 45, m, &u[0]);
    emlrtAssign(&b_y, m);
    for (ret = 0; ret < 4; ret++) {
      c_u[ret] = d_u[ret];
    }

    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&st, 4, m, &c_u[0]);
    emlrtAssign(&c_y, m);
    b_st.site = &e_emlrtRSI;
    error(&b_st, y, getString(&b_st, message(&b_st, b_y, c_y, &g_emlrtMCI),
           &g_emlrtMCI), &g_emlrtMCI);
  }

  if (obj->isInitialized != 1) {
    b_st.site = &e_emlrtRSI;
    c_st.site = &e_emlrtRSI;
    SystemCore_setup(&c_st, obj, varargin_7);
    c_st.site = &e_emlrtRSI;
    obj->pLoopFilterState = 0.0;
    obj->pIntegFilterState = 0.0;
    obj->pDDSPreviousInput = 0.0;
    obj->pPhase = 0.0;
    obj->pPreviousSample.re = 0.0;
    obj->pPreviousSample.im = 0.0;
  }

  b_st.site = &e_emlrtRSI;
  if (!obj->CacheInputSizes) {
    obj->CacheInputSizes = true;
    c_st.site = &e_emlrtRSI;
    varSizes[0].f1[0] = (uint32_T)varargin_7->size[0];
    varSizes[0].f1[1] = 1U;
    for (ret = 0; ret < 6; ret++) {
      varSizes[0].f1[ret + 2] = 1U;
    }

    obj->inputVarSize[0] = varSizes[0];
  }

  b_st.site = &e_emlrtRSI;
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    c_st.site = &e_emlrtRSI;
    d_st.site = &x_emlrtRSI;
    PhaseRecoveryLoopBandwidth = obj->NormalizedLoopBandwidth *
      obj->SamplesPerSymbol;
    PhaseRecoveryGain = obj->SamplesPerSymbol;
    PhaseRecoveryLoopBandwidth /= (obj->DampingFactor + 0.25 /
      obj->DampingFactor) * obj->SamplesPerSymbol;
    d = (2.0 * obj->DampingFactor * PhaseRecoveryLoopBandwidth + 1.0) +
      PhaseRecoveryLoopBandwidth * PhaseRecoveryLoopBandwidth;
    obj->pProportionalGain = 4.0 * obj->DampingFactor *
      PhaseRecoveryLoopBandwidth / d / PhaseRecoveryGain;
    obj->pIntegratorGain = 4.0 * PhaseRecoveryLoopBandwidth *
      PhaseRecoveryLoopBandwidth / d / PhaseRecoveryGain;
    for (ret = 0; ret < 4; ret++) {
      c_u[ret] = obj->ModulationPhaseOffset[ret];
    }

    for (ret = 0; ret < 4; ret++) {
      b[ret] = correctedValue[ret];
    }

    ret = memcmp(&c_u[0], &b[0], 4);
    if (ret == 0) {
      obj->pActualPhaseOffset = 0.0;
    } else {
      obj->pActualPhaseOffset = obj->CustomPhaseOffset;
    }
  }

  b_st.site = &e_emlrtRSI;
  inSize[0] = (int16_T)varargin_7->size[0];
  inSize[1] = 1;
  for (ret = 0; ret < 6; ret++) {
    inSize[ret + 2] = 1;
  }

  ret = 0;
  exitg1 = false;
  while ((!exitg1) && (ret < 8)) {
    if (obj->inputVarSize[0].f1[ret] != (uint32_T)inSize[ret]) {
      for (ret = 0; ret < 8; ret++) {
        obj->inputVarSize[0].f1[ret] = (uint32_T)inSize[ret];
      }

      exitg1 = true;
    } else {
      ret++;
    }
  }

  b_st.site = &e_emlrtRSI;
  CarrierSynchronizer_stepImpl(&b_st, obj, varargin_7, varargout_1);
  b_st.site = &e_emlrtRSI;
  SystemCore_checkTunablePropChange(&b_st, obj);
}

static void CarrierSynchronizer_stepImpl(const emlrtStack *sp,
  comm_CarrierSynchronizer *obj, emxArray_creal_T *input, emxArray_creal_T
  *output)
{
  emxArray_creal_T *phaseCorrection;
  real_T loopFiltOut;
  real_T DDSOut;
  real_T DDSPreviousInp;
  creal_T previousSample;
  int32_T i;
  int32_T k;
  real_T phErr;
  real_T b;
  creal_T b_b;
  real_T im;
  int32_T i1;
  int32_T i2;
  emxInit_creal_T1(&phaseCorrection, 1);
  loopFiltOut = obj->pLoopFilterState;
  DDSOut = obj->pIntegFilterState;
  DDSPreviousInp = obj->pDDSPreviousInput;
  previousSample.re = obj->pPreviousSample.re;
  previousSample.im = obj->pPreviousSample.im;
  i = output->size[0];
  output->size[0] = input->size[0];
  emxEnsureCapacity_creal_T1(output, i);
  i = phaseCorrection->size[0];
  phaseCorrection->size[0] = input->size[0];
  emxEnsureCapacity_creal_T1(phaseCorrection, i);
  i = input->size[0] - 1;
  for (k = 0; k <= i; k++) {
    phErr = muDoubleScalarSign(previousSample.re) * previousSample.im;
    b = obj->pPhase;
    previousSample.re = b * 0.0;
    previousSample.im = b;
    b_exp(&previousSample);
    i1 = input->size[0];
    i2 = input->size[0];
    b = input->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1, &d_emlrtBCI, sp)
      - 1].re * previousSample.re - input->data[emlrtDynamicBoundsCheckR2012b(k
      + 1, 1, i2, &d_emlrtBCI, sp) - 1].im * previousSample.im;
    i1 = input->size[0];
    i2 = input->size[0];
    im = input->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1, &d_emlrtBCI, sp)
      - 1].re * previousSample.im + input->data[emlrtDynamicBoundsCheckR2012b(k
      + 1, 1, i2, &d_emlrtBCI, sp) - 1].im * previousSample.re;
    i1 = output->size[0];
    output->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1, &emlrtBCI, sp) - 1]
      .re = b;
    i1 = output->size[0];
    output->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1, &emlrtBCI, sp) - 1]
      .im = im;
    loopFiltOut += phErr * obj->pIntegratorGain;
    DDSOut += DDSPreviousInp;
    DDSPreviousInp = phErr * obj->pProportionalGain + loopFiltOut;
    obj->pPhase = obj->pDigitalSynthesizerGain * DDSOut;
    i1 = phaseCorrection->size[0];
    phaseCorrection->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1,
      &b_emlrtBCI, sp) - 1].re = obj->pPhase;
    i1 = phaseCorrection->size[0];
    phaseCorrection->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1,
      &b_emlrtBCI, sp) - 1].im = 0.0;
    i1 = output->size[0];
    previousSample.re = output->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1,
      &c_emlrtBCI, sp) - 1].re;
    i1 = output->size[0];
    previousSample.im = output->data[emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1,
      &c_emlrtBCI, sp) - 1].im;
  }

  emxFree_creal_T(&phaseCorrection);
  b = obj->pActualPhaseOffset;
  b_b.re = b * 0.0;
  b_b.im = b;
  b_exp(&b_b);
  i = output->size[0];
  emxEnsureCapacity_creal_T1(output, i);
  k = output->size[0];
  for (i = 0; i < k; i++) {
    b = output->data[i].re * b_b.re - output->data[i].im * b_b.im;
    im = output->data[i].re * b_b.im + output->data[i].im * b_b.re;
    output->data[i].re = b;
    output->data[i].im = im;
  }

  obj->pLoopFilterState = loopFiltOut;
  obj->pIntegFilterState = DDSOut;
  obj->pPreviousSample.re = previousSample.re;
  obj->pPreviousSample.im = previousSample.im;
  obj->pDDSPreviousInput = DDSPreviousInp;
}

static const mxArray *emlrt_marshallOut(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp)
{
  const mxArray *y;
  const mxArray *b_y;
  const mxArray *m;
  static const int32_T iv[1] = { 2 };

  int32_T *pData;
  int32_T u;
  int32_T i;
  static const char * sv[18] = { "isInitialized", "TunablePropsChanged",
    "inputVarSize", "CacheInputSizes", "ModulationPhaseOffset",
    "CustomPhaseOffset", "SamplesPerSymbol", "DampingFactor",
    "NormalizedLoopBandwidth", "pProportionalGain", "pIntegratorGain",
    "pDigitalSynthesizerGain", "pPhase", "pPreviousSample", "pActualPhaseOffset",
    "pLoopFilterState", "pIntegFilterState", "pDDSPreviousInput" };

  const mxArray *c_y;
  boolean_T b_u;
  cell_wrap c_u[1];
  int32_T iv1[1];
  static const char * sv1[1] = { "f1" };

  const cell_wrap *r;
  uint32_T d_u[8];
  const mxArray *d_y;
  static const int32_T iv2[2] = { 1, 8 };

  uint32_T *b_pData;
  char_T e_u[4];
  static const int32_T iv3[2] = { 1, 4 };

  real_T f_u;
  real_T u_im;
  creal_T *r1;
  y = NULL;
  emlrtAssign(&y, emlrtCreateCellMatrix(3, 1));
  b_y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxINT32_CLASS, mxREAL);
  pData = (int32_T *)emlrtMxGetData(m);
  u = 0;
  for (i = 0; i < 2; i++) {
    pData[u] = (*moduleInstance->v)[i];
    u++;
  }

  emlrtAssign(&b_y, m);
  emlrtSetCell(y, 0, b_y);
  b_y = NULL;
  emlrtAssign(&b_y, emlrtCreateStructMatrix(1, 1, 18, sv));
  u = moduleInstance->sysobj.isInitialized;
  c_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
  *(int32_T *)emlrtMxGetData(m) = u;
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "isInitialized", c_y, 0);
  b_u = moduleInstance->sysobj.TunablePropsChanged;
  c_y = NULL;
  m = emlrtCreateLogicalScalar(b_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "TunablePropsChanged", c_y, 1);
  c_u[0] = moduleInstance->sysobj.inputVarSize[0];
  c_y = NULL;
  iv1[0] = 1;
  emlrtAssign(&c_y, emlrtCreateStructArray(1, iv1, 1, sv1));
  r = &c_u[0];
  for (u = 0; u < 8; u++) {
    d_u[u] = r->f1[u];
  }

  d_y = NULL;
  m = emlrtCreateNumericArray(2, &iv2[0], mxUINT32_CLASS, mxREAL);
  b_pData = (uint32_T *)emlrtMxGetData(m);
  u = 0;
  for (i = 0; i < 8; i++) {
    b_pData[u] = d_u[i];
    u++;
  }

  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(c_y, 0, "f1", d_y, 0);
  emlrtSetFieldR2017b(b_y, 0, "inputVarSize", c_y, 2);
  b_u = moduleInstance->sysobj.CacheInputSizes;
  c_y = NULL;
  m = emlrtCreateLogicalScalar(b_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "CacheInputSizes", c_y, 3);
  for (u = 0; u < 4; u++) {
    e_u[u] = moduleInstance->sysobj.ModulationPhaseOffset[u];
  }

  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv3[0]);
  emlrtInitCharArrayR2013a(sp, 4, m, &e_u[0]);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "ModulationPhaseOffset", c_y, 4);
  f_u = moduleInstance->sysobj.CustomPhaseOffset;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "CustomPhaseOffset", c_y, 5);
  f_u = moduleInstance->sysobj.SamplesPerSymbol;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "SamplesPerSymbol", c_y, 6);
  f_u = moduleInstance->sysobj.DampingFactor;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "DampingFactor", c_y, 7);
  f_u = moduleInstance->sysobj.NormalizedLoopBandwidth;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "NormalizedLoopBandwidth", c_y, 8);
  f_u = moduleInstance->sysobj.pProportionalGain;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pProportionalGain", c_y, 9);
  f_u = moduleInstance->sysobj.pIntegratorGain;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pIntegratorGain", c_y, 10);
  f_u = moduleInstance->sysobj.pDigitalSynthesizerGain;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pDigitalSynthesizerGain", c_y, 11);
  f_u = moduleInstance->sysobj.pPhase;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pPhase", c_y, 12);
  f_u = moduleInstance->sysobj.pPreviousSample.re;
  u_im = moduleInstance->sysobj.pPreviousSample.im;
  c_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxCOMPLEX);
  r1 = (creal_T *)emlrtMxGetData(m);
  r1->re = f_u;
  r1->im = u_im;
  emlrtFreeImagIfZero(m);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pPreviousSample", c_y, 13);
  f_u = moduleInstance->sysobj.pActualPhaseOffset;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pActualPhaseOffset", c_y, 14);
  f_u = moduleInstance->sysobj.pLoopFilterState;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pLoopFilterState", c_y, 15);
  f_u = moduleInstance->sysobj.pIntegFilterState;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pIntegFilterState", c_y, 16);
  f_u = moduleInstance->sysobj.pDDSPreviousInput;
  c_y = NULL;
  m = emlrtCreateDoubleScalar(f_u);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "pDDSPreviousInput", c_y, 17);
  emlrtSetCell(y, 1, b_y);
  b_y = NULL;
  m = emlrtCreateLogicalScalar(moduleInstance->sysobj_not_empty);
  emlrtAssign(&b_y, m);
  emlrtSetCell(y, 2, b_y);
  return y;
}

static const mxArray *cgxe_mdl_get_sim_state
  (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance)
{
  const mxArray *st;
  emlrtStack b_st = { NULL,            /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  b_st.tls = moduleInstance->emlrtRootTLSGlobal;
  st = NULL;
  emlrtAssign(&st, emlrt_marshallOut(moduleInstance, &b_st));
  return st;
}

static void emlrt_marshallIn(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const emlrtStack *sp, const mxArray *u)
{
  emlrtMsgIdentifier thisId;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  thisId.fIdentifier = "v";
  b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, &thisId, u, 0)), "v",
                     *moduleInstance->v);
  thisId.fIdentifier = "sysobj";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, &thisId, u, 1)), "sysobj",
                     &moduleInstance->sysobj);
  thisId.fIdentifier = "sysobj_not_empty";
  moduleInstance->sysobj_not_empty = m_emlrt_marshallIn(sp, emlrtAlias
    (emlrtGetCell(sp, &thisId, u, 2)), "sysobj_not_empty");
  emlrtDestroyArray(&u);
}

static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_v, const
  char_T *identifier, int32_T y[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  c_emlrt_marshallIn(sp, emlrtAlias(b_v), &thisId, y);
  emlrtDestroyArray(&b_v);
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, int32_T y[2])
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_sysobj,
  const char_T *identifier, comm_CarrierSynchronizer *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  e_emlrt_marshallIn(sp, emlrtAlias(b_sysobj), &thisId, y);
  emlrtDestroyArray(&b_sysobj);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, comm_CarrierSynchronizer *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[18] = { "isInitialized", "TunablePropsChanged",
    "inputVarSize", "CacheInputSizes", "ModulationPhaseOffset",
    "CustomPhaseOffset", "SamplesPerSymbol", "DampingFactor",
    "NormalizedLoopBandwidth", "pProportionalGain", "pIntegratorGain",
    "pDigitalSynthesizerGain", "pPhase", "pPreviousSample", "pActualPhaseOffset",
    "pLoopFilterState", "pIntegFilterState", "pDDSPreviousInput" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 18, fieldNames, 0U, &dims);
  thisId.fIdentifier = "isInitialized";
  y->isInitialized = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
    0, 0, "isInitialized")), &thisId);
  thisId.fIdentifier = "TunablePropsChanged";
  y->TunablePropsChanged = g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
    (sp, u, 0, 1, "TunablePropsChanged")), &thisId);
  thisId.fIdentifier = "inputVarSize";
  h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2,
    "inputVarSize")), &thisId, y->inputVarSize);
  thisId.fIdentifier = "CacheInputSizes";
  y->CacheInputSizes = g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
    u, 0, 3, "CacheInputSizes")), &thisId);
  thisId.fIdentifier = "ModulationPhaseOffset";
  j_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "ModulationPhaseOffset")), &thisId, y->ModulationPhaseOffset);
  thisId.fIdentifier = "CustomPhaseOffset";
  y->CustomPhaseOffset = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
    (sp, u, 0, 5, "CustomPhaseOffset")), &thisId);
  thisId.fIdentifier = "SamplesPerSymbol";
  y->SamplesPerSymbol = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
    u, 0, 6, "SamplesPerSymbol")), &thisId);
  thisId.fIdentifier = "DampingFactor";
  y->DampingFactor = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
    0, 7, "DampingFactor")), &thisId);
  thisId.fIdentifier = "NormalizedLoopBandwidth";
  y->NormalizedLoopBandwidth = k_emlrt_marshallIn(sp, emlrtAlias
    (emlrtGetFieldR2017b(sp, u, 0, 8, "NormalizedLoopBandwidth")), &thisId);
  thisId.fIdentifier = "pProportionalGain";
  y->pProportionalGain = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
    (sp, u, 0, 9, "pProportionalGain")), &thisId);
  thisId.fIdentifier = "pIntegratorGain";
  y->pIntegratorGain = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
    u, 0, 10, "pIntegratorGain")), &thisId);
  thisId.fIdentifier = "pDigitalSynthesizerGain";
  y->pDigitalSynthesizerGain = k_emlrt_marshallIn(sp, emlrtAlias
    (emlrtGetFieldR2017b(sp, u, 0, 11, "pDigitalSynthesizerGain")), &thisId);
  thisId.fIdentifier = "pPhase";
  y->pPhase = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 12,
    "pPhase")), &thisId);
  thisId.fIdentifier = "pPreviousSample";
  y->pPreviousSample = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
    u, 0, 13, "pPreviousSample")), &thisId);
  thisId.fIdentifier = "pActualPhaseOffset";
  y->pActualPhaseOffset = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
    (sp, u, 0, 14, "pActualPhaseOffset")), &thisId);
  thisId.fIdentifier = "pLoopFilterState";
  y->pLoopFilterState = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
    u, 0, 15, "pLoopFilterState")), &thisId);
  thisId.fIdentifier = "pIntegFilterState";
  y->pIntegFilterState = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
    (sp, u, 0, 16, "pIntegFilterState")), &thisId);
  thisId.fIdentifier = "pDDSPreviousInput";
  y->pDDSPreviousInput = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
    (sp, u, 0, 17, "pDDSPreviousInput")), &thisId);
  emlrtDestroyArray(&u);
}

static int32_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static boolean_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, cell_wrap y[1])
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[1] = { "f1" };

  static const int32_T dims[1] = { 1 };

  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 1, fieldNames, 1U, dims);
  thisId.fIdentifier = "f1";
  i_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "f1")),
                     &thisId, y[0].f1);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, uint32_T y[8])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[4])
{
  r_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static creal_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  creal_T y;
  y = t_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static boolean_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *b_sysobj_not_empty, const char_T *identifier)
{
  boolean_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = g_emlrt_marshallIn(sp, emlrtAlias(b_sysobj_not_empty), &thisId);
  emlrtDestroyArray(&b_sysobj_not_empty);
  return y;
}

static void cgxe_mdl_set_sim_state(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance, const mxArray *st)
{
  emlrtStack b_st = { NULL,            /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  b_st.tls = moduleInstance->emlrtRootTLSGlobal;
  emlrt_marshallIn(moduleInstance, &b_st, emlrtAlias(st));
  emlrtDestroyArray(&st);
}

static const mxArray *message(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(sp, 1, &m, 2, pArrays, "message", true, location);
}

static const mxArray *getString(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m, 1, &pArray, "getString", true,
    location);
}

static void error(const emlrtStack *sp, const mxArray *b, const mxArray *c,
                  emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = b;
  pArrays[1] = c;
  emlrtCallMATLABR2012b(sp, 0, NULL, 2, pArrays, "error", true, location);
}

static const mxArray *b_message(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, const mxArray *d, const mxArray *e, emlrtMCInfo *location)
{
  const mxArray *pArrays[4];
  const mxArray *m;
  pArrays[0] = b;
  pArrays[1] = c;
  pArrays[2] = d;
  pArrays[3] = e;
  return emlrtCallMATLABR2012b(sp, 1, &m, 4, pArrays, "message", true, location);
}

static const mxArray *c_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m, 1, &pArray, "message", true, location);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, int32_T ret[2])
{
  static const int32_T dims[1] = { 2 };

  int32_T (*r)[2];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 1U, dims);
  r = (int32_T (*)[2])emlrtMxGetData(src);
  for (i = 0; i < 2; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static int32_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  int32_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 0U, &dims);
  ret = *(int32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static boolean_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "logical", false, 0U, &dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, uint32_T ret[8])
{
  static const int32_T dims[2] = { 1, 8 };

  uint32_T (*r)[8];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint32", false, 2U, dims);
  r = (uint32_T (*)[8])emlrtMxGetData(src);
  for (i = 0; i < 8; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[4])
{
  static const int32_T dims[2] = { 1, 4 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "char", false, 2U, dims);
  emlrtImportCharArrayR2015b(sp, src, &ret[0], 4);
  emlrtDestroyArray(&src);
}

static real_T s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static creal_T t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  creal_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", true, 0U, &dims);
  emlrtImportArrayR2015b(sp, src, &ret, 8, true);
  emlrtDestroyArray(&src);
  return ret;
}

static void b_exp(creal_T *x)
{
  real_T r;
  real_T d;
  if (x->im == 0.0) {
    x->re = muDoubleScalarExp(x->re);
    x->im = 0.0;
  } else if (muDoubleScalarIsInf(x->im) && muDoubleScalarIsInf(x->re) && (x->re <
              0.0)) {
    x->re = 0.0;
    x->im = 0.0;
  } else {
    r = muDoubleScalarExp(x->re / 2.0);
    d = x->im;
    x->re = r * (r * muDoubleScalarCos(x->im));
    x->im = r * (r * muDoubleScalarSin(d));
  }
}

static void emxEnsureCapacity_creal_T(emxArray_creal_T *emxArray, int32_T
  oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(creal_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(creal_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (creal_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void emxInit_creal_T(emxArray_creal_T **pEmxArray, int32_T numDimensions)
{
  emxArray_creal_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_creal_T *)emlrtMallocMex(sizeof(emxArray_creal_T));
  emxArray = *pEmxArray;
  emxArray->data = (creal_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)
    numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxFree_creal_T(emxArray_creal_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_creal_T *)NULL) {
    if (((*pEmxArray)->data != (creal_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_creal_T *)NULL;
  }
}

static void emxInit_creal_T1(emxArray_creal_T **pEmxArray, int32_T numDimensions)
{
  emxArray_creal_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_creal_T *)emlrtMallocMex(sizeof(emxArray_creal_T));
  emxArray = *pEmxArray;
  emxArray->data = (creal_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)
    numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxEnsureCapacity_creal_T1(emxArray_creal_T *emxArray, int32_T
  oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(creal_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(creal_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (creal_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void init_simulink_io_address(InstanceStruct_8rrCCyrtiOptd4UQNBcUyG
  *moduleInstance)
{
  moduleInstance->emlrtRootTLSGlobal = (void *)cgxertGetEMLRTCtx
    (moduleInstance->S);
  moduleInstance->u0_data = (creal_T (*)[6175])cgxertGetInputPortSignal
    (moduleInstance->S, 0);
  moduleInstance->u0_sizes = (int32_T (*)[2])cgxertGetCurrentInputPortDimensions
    (moduleInstance->S, 0);
  moduleInstance->y0_data = (creal_T (*)[6175])cgxertGetOutputPortSignal
    (moduleInstance->S, 0);
  moduleInstance->y0_sizes = (int32_T (*)[2])
    cgxertGetCurrentOutputPortDimensions(moduleInstance->S, 0);
  moduleInstance->v = (int32_T (*)[2])cgxertGetDWork(moduleInstance->S, 0);
}

/* CGXE Glue Code */
static void mdlOutputs_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S, int_T tid)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_outputs(moduleInstance);
}

static void mdlInitialize_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_initialize(moduleInstance);
}

static void mdlUpdate_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S, int_T tid)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_update(moduleInstance);
}

static mxArray* getSimState_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
  mxArray* mxSS;
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  mxSS = (mxArray *) cgxe_mdl_get_sim_state(moduleInstance);
  return mxSS;
}

static void setSimState_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S, const mxArray *ss)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_set_sim_state(moduleInstance, emlrtAlias(ss));
}

static void mdlTerminate_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_terminate(moduleInstance);
  free((void *)moduleInstance);
}

static void mdlEnable_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_enable(moduleInstance);
}

static void mdlDisable_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_disable(moduleInstance);
}

static void mdlStart_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
  InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *moduleInstance =
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG *)calloc(1, sizeof
    (InstanceStruct_8rrCCyrtiOptd4UQNBcUyG));
  moduleInstance->S = S;
  cgxertSetRuntimeInstance(S, (void *)moduleInstance);
  ssSetmdlOutputs(S, mdlOutputs_8rrCCyrtiOptd4UQNBcUyG);
  ssSetmdlInitializeConditions(S, mdlInitialize_8rrCCyrtiOptd4UQNBcUyG);
  ssSetmdlUpdate(S, mdlUpdate_8rrCCyrtiOptd4UQNBcUyG);
  ssSetmdlTerminate(S, mdlTerminate_8rrCCyrtiOptd4UQNBcUyG);
  ssSetmdlEnable(S, mdlEnable_8rrCCyrtiOptd4UQNBcUyG);
  ssSetmdlDisable(S, mdlDisable_8rrCCyrtiOptd4UQNBcUyG);
  cgxe_mdl_start(moduleInstance);

  {
    uint_T options = ssGetOptions(S);
    options |= SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE;
    ssSetOptions(S, options);
  }
}

static void mdlProcessParameters_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S)
{
}

void method_dispatcher_8rrCCyrtiOptd4UQNBcUyG(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_8rrCCyrtiOptd4UQNBcUyG(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_8rrCCyrtiOptd4UQNBcUyG(S);
    break;

   case SS_CALL_MDL_GET_SIM_STATE:
    *((mxArray**) data) = getSimState_8rrCCyrtiOptd4UQNBcUyG(S);
    break;

   case SS_CALL_MDL_SET_SIM_STATE:
    setSimState_8rrCCyrtiOptd4UQNBcUyG(S, (const mxArray *) data);
    break;

   default:
    /* Unhandled method */
    /*
       sf_mex_error_message("Stateflow Internal Error:\n"
       "Error calling method dispatcher for module: 8rrCCyrtiOptd4UQNBcUyG.\n"
       "Can't handle method %d.\n", method);
     */
    break;
  }
}

mxArray *cgxe_8rrCCyrtiOptd4UQNBcUyG_BuildInfoUpdate(void)
{
  mxArray * mxBIArgs;
  mxArray * elem_1;
  mxArray * elem_2;
  mxArray * elem_3;
  mxArray * elem_4;
  mxArray * elem_5;
  mxArray * elem_6;
  mxArray * elem_7;
  mxArray * elem_8;
  mxArray * elem_9;
  mxArray * elem_10;
  mxArray * elem_11;
  mxArray * elem_12;
  mxArray * elem_13;
  mxArray * elem_14;
  mxArray * elem_15;
  mxArray * elem_16;
  double * pointer;
  mxBIArgs = mxCreateCellMatrix(1,3);
  elem_1 = mxCreateCellMatrix(1,6);
  elem_2 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,0,elem_2);
  elem_3 = mxCreateCellMatrix(1,4);
  elem_4 = mxCreateString("addIncludeFiles");
  mxSetCell(elem_3,0,elem_4);
  elem_5 = mxCreateCellMatrix(1,1);
  elem_6 = mxCreateString("<string.h>");
  mxSetCell(elem_5,0,elem_6);
  mxSetCell(elem_3,1,elem_5);
  elem_7 = mxCreateCellMatrix(1,1);
  elem_8 = mxCreateString("");
  mxSetCell(elem_7,0,elem_8);
  mxSetCell(elem_3,2,elem_7);
  elem_9 = mxCreateCellMatrix(1,1);
  elem_10 = mxCreateString("__EMLJITSupported__");
  mxSetCell(elem_9,0,elem_10);
  mxSetCell(elem_3,3,elem_9);
  mxSetCell(elem_1,1,elem_3);
  elem_11 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,2,elem_11);
  elem_12 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,3,elem_12);
  elem_13 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,4,elem_13);
  elem_14 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,5,elem_14);
  mxSetCell(mxBIArgs,0,elem_1);
  elem_15 = mxCreateDoubleMatrix(0,0, mxREAL);
  pointer = mxGetPr(elem_15);
  mxSetCell(mxBIArgs,1,elem_15);
  elem_16 = mxCreateCellMatrix(1,0);
  mxSetCell(mxBIArgs,2,elem_16);
  return mxBIArgs;
}

mxArray *cgxe_8rrCCyrtiOptd4UQNBcUyG_fallback_info(void)
{
  const char* fallbackInfoFields[] = { "fallbackType", "incompatiableSymbol" };

  mxArray* fallbackInfoStruct = mxCreateStructMatrix(1, 1, 2, fallbackInfoFields);
  mxArray* fallbackType = mxCreateString("incompatibleVar");
  mxArray* incompatibleSymbol = mxCreateString("u_tmp3:  y_tmp0:  ");
  mxSetFieldByNumber(fallbackInfoStruct, 0, 0, fallbackType);
  mxSetFieldByNumber(fallbackInfoStruct, 0, 1, incompatibleSymbol);
  return fallbackInfoStruct;
}
