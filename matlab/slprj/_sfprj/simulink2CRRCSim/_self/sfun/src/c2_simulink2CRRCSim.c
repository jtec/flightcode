/* Include files */

#include "blascompat32.h"
#include "simulink2CRRCSim_sfun.h"
#include "c2_simulink2CRRCSim.h"
#include <string.h>
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "simulink2CRRCSim_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[17] = { "startByte1", "startByte2",
  "messageIDByte1", "messageIDByte2", "servobyte32Bit1", "servobyte32Bit2",
  "servo32Bit", "checksum", "servoBytes", "checksumBytes", "message",
  "channelIndex", "servoValueinBytes", "nargin", "nargout", "inputs", "y" };

/* Function Declarations */
static void initialize_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance);
static void initialize_params_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance);
static void enable_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance);
static void disable_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance);
static void c2_update_debugger_state_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance);
static void set_sim_state_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance);
static void sf_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance);
static void initSimStructsc2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_y, const char_T *c2_identifier, uint8_T
  c2_b_y[24]);
static void c2_b_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_y[24]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_d_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_y[4]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static uint32_T c2_e_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_f_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  uint32_T c2_y[9]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static uint8_T c2_g_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_startByte2, const char_T *c2_identifier);
static uint8_T c2_h_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_typecast(SFc2_simulink2CRRCSimInstanceStruct *chartInstance,
  uint32_T c2_x, uint8_T c2_y[4]);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_i_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void init_dsm_address_info(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_simulink2CRRCSim = 0U;
}

static void initialize_params_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance)
{
}

static void enable_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  uint8_T c2_u[24];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  uint8_T (*c2_d_y)[24];
  c2_d_y = (uint8_T (*)[24])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  for (c2_i0 = 0; c2_i0 < 24; c2_i0++) {
    c2_u[c2_i0] = (*c2_d_y)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 3, 0U, 1U, 0U, 1, 24), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_simulink2CRRCSim;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  uint8_T c2_uv0[24];
  int32_T c2_i1;
  uint8_T (*c2_y)[24];
  c2_y = (uint8_T (*)[24])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)), "y",
                      c2_uv0);
  for (c2_i1 = 0; c2_i1 < 24; c2_i1++) {
    (*c2_y)[c2_i1] = c2_uv0[c2_i1];
  }

  chartInstance->c2_is_active_c2_simulink2CRRCSim = c2_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_simulink2CRRCSim");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_simulink2CRRCSim(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance)
{
}

static void sf_c2_simulink2CRRCSim(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  real_T c2_inputs[9];
  uint32_T c2_debug_family_var_map[17];
  uint8_T c2_startByte1;
  uint8_T c2_startByte2;
  uint8_T c2_messageIDByte1;
  uint8_T c2_messageIDByte2;
  uint32_T c2_servobyte32Bit1[9];
  uint32_T c2_servobyte32Bit2[9];
  uint32_T c2_servo32Bit[9];
  uint32_T c2_checksum;
  uint8_T c2_servoBytes[4];
  uint8_T c2_checksumBytes[4];
  uint8_T c2_message[24];
  real_T c2_channelIndex;
  uint8_T c2_servoValueinBytes[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  uint8_T c2_y[24];
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i;
  real_T c2_b_i;
  real_T c2_a;
  real_T c2_b_y;
  real_T c2_d0;
  uint32_T c2_u0;
  uint8_T c2_uv1[4];
  int32_T c2_i8;
  uint32_T c2_q0;
  uint32_T c2_qY;
  uint32_T c2_b_q0;
  uint32_T c2_b_qY;
  uint8_T c2_uv2[4];
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_c_i;
  uint8_T c2_uv3[4];
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  uint8_T (*c2_c_y)[24];
  real_T (*c2_b_inputs)[9];
  c2_c_y = (uint8_T (*)[24])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_inputs = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 9; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_b_inputs)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 24; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_c_y)[c2_i3], 1U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i4 = 0; c2_i4 < 9; c2_i4++) {
    c2_inputs[c2_i4] = (*c2_b_inputs)[c2_i4];
  }

  sf_debug_symbol_scope_push_eml(0U, 17U, 17U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c2_startByte1, 0U,
    c2_g_sf_marshallOut, c2_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_startByte2, 1U,
    c2_g_sf_marshallOut, c2_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_messageIDByte1, 2U, c2_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_messageIDByte2, 3U, c2_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_servobyte32Bit1, 4U,
    c2_f_sf_marshallOut, c2_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_servobyte32Bit2, 5U,
    c2_f_sf_marshallOut, c2_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_servo32Bit, 6U,
    c2_f_sf_marshallOut, c2_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_checksum, 7U, c2_e_sf_marshallOut,
    c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_servoBytes, 8U,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_checksumBytes, 9U,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_message, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_channelIndex, 11U,
    c2_c_sf_marshallOut, c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_servoValueinBytes, 12U,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 13U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 14U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c2_inputs, 15U, c2_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_y, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_startByte1 = 85U;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_startByte2 = 85U;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  c2_messageIDByte1 = 83U;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_messageIDByte2 = 83U;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i5 = 0; c2_i5 < 9; c2_i5++) {
    c2_servobyte32Bit1[c2_i5] = 0U;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  for (c2_i6 = 0; c2_i6 < 9; c2_i6++) {
    c2_servobyte32Bit2[c2_i6] = 0U;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  for (c2_i7 = 0; c2_i7 < 9; c2_i7++) {
    c2_servo32Bit[c2_i7] = 0U;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_checksum = 0U;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_checksum = 166U;
  c2_i = 0;
  while (c2_i < 9) {
    c2_b_i = 1.0 + (real_T)c2_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
    c2_a = c2_inputs[_SFD_EML_ARRAY_BOUNDS_CHECK("inputs", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) - 1];
    c2_b_y = c2_a * 32000.0;
    c2_d0 = muDoubleScalarRound(32767.0 + c2_b_y);
    if (c2_d0 < 4.294967296E+9) {
      if (c2_d0 >= 0.0) {
        c2_u0 = (uint32_T)c2_d0;
      } else {
        c2_u0 = 0U;
      }
    } else if (c2_d0 >= 4.294967296E+9) {
      c2_u0 = MAX_uint32_T;
    } else {
      c2_u0 = 0U;
    }

    c2_servo32Bit[_SFD_EML_ARRAY_BOUNDS_CHECK("servo32Bit", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) - 1] = c2_u0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
    c2_typecast(chartInstance, c2_servo32Bit[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "servo32Bit", (int32_T)_SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) - 1],
                c2_uv1);
    for (c2_i8 = 0; c2_i8 < 4; c2_i8++) {
      c2_servoBytes[c2_i8] = c2_uv1[c2_i8];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
    c2_servobyte32Bit1[_SFD_EML_ARRAY_BOUNDS_CHECK("servobyte32Bit1", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) - 1] = c2_servoBytes[0];
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
    c2_servobyte32Bit2[_SFD_EML_ARRAY_BOUNDS_CHECK("servobyte32Bit2", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) - 1] = c2_servoBytes[1];
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
    c2_q0 = c2_checksum;
    c2_qY = c2_q0 + c2_servobyte32Bit1[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "servobyte32Bit1", (int32_T)_SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) -
      1];
    if (c2_qY < c2_q0) {
      c2_qY = MAX_uint32_T;
    }

    c2_b_q0 = c2_qY;
    c2_b_qY = c2_b_q0 + c2_servobyte32Bit2[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "servobyte32Bit2", (int32_T)_SFD_INTEGER_CHECK("i", c2_b_i), 1, 9, 1, 0) -
      1];
    if (c2_b_qY < c2_b_q0) {
      c2_b_qY = MAX_uint32_T;
    }

    c2_checksum = c2_b_qY;
    c2_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
  c2_typecast(chartInstance, c2_checksum, c2_uv2);
  for (c2_i9 = 0; c2_i9 < 4; c2_i9++) {
    c2_checksumBytes[c2_i9] = c2_uv2[c2_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
  for (c2_i10 = 0; c2_i10 < 24; c2_i10++) {
    c2_message[c2_i10] = 0U;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_message[0] = c2_startByte1;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  c2_message[1] = c2_startByte2;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  c2_message[2] = c2_messageIDByte1;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_message[3] = c2_messageIDByte2;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_channelIndex = 1.0;
  c2_c_i = 0;
  while (c2_c_i < 9) {
    c2_b_i = 1.0 + (real_T)c2_c_i * 2.0;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 34);
    c2_typecast(chartInstance, c2_servo32Bit[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "servo32Bit", (int32_T)_SFD_INTEGER_CHECK("channelIndex", c2_channelIndex),
      1, 9, 1, 0) - 1], c2_uv3);
    for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
      c2_servoValueinBytes[c2_i11] = c2_uv3[c2_i11];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
    c2_message[_SFD_EML_ARRAY_BOUNDS_CHECK("message", (int32_T)
      _SFD_INTEGER_CHECK("4+i", 4.0 + c2_b_i), 1, 24, 1, 0) - 1] =
      c2_servoValueinBytes[1];
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
    c2_message[_SFD_EML_ARRAY_BOUNDS_CHECK("message", (int32_T)
      _SFD_INTEGER_CHECK("4+i+1", (4.0 + c2_b_i) + 1.0), 1, 24, 1, 0) - 1] =
      c2_servoValueinBytes[0];
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
    c2_channelIndex++;
    c2_c_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  c2_message[22] = c2_checksumBytes[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 40);
  c2_message[23] = c2_checksumBytes[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
  for (c2_i12 = 0; c2_i12 < 24; c2_i12++) {
    c2_y[c2_i12] = c2_message[c2_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -42);
  sf_debug_symbol_scope_pop();
  for (c2_i13 = 0; c2_i13 < 24; c2_i13++) {
    (*c2_c_y)[c2_i13] = c2_y[c2_i13];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  sf_debug_check_for_state_inconsistency(_simulink2CRRCSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc2_simulink2CRRCSim
  (SFc2_simulink2CRRCSimInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i14;
  uint8_T c2_b_inData[24];
  int32_T c2_i15;
  uint8_T c2_u[24];
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i14 = 0; c2_i14 < 24; c2_i14++) {
    c2_b_inData[c2_i14] = (*(uint8_T (*)[24])c2_inData)[c2_i14];
  }

  for (c2_i15 = 0; c2_i15 < 24; c2_i15++) {
    c2_u[c2_i15] = c2_b_inData[c2_i15];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 3, 0U, 1U, 0U, 1, 24), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_y, const char_T *c2_identifier, uint8_T
  c2_b_y[24])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_y), &c2_thisId, c2_b_y);
  sf_mex_destroy(&c2_y);
}

static void c2_b_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_y[24])
{
  uint8_T c2_uv4[24];
  int32_T c2_i16;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_uv4, 1, 3, 0U, 1, 0U, 1, 24);
  for (c2_i16 = 0; c2_i16 < 24; c2_i16++) {
    c2_y[c2_i16] = c2_uv4[c2_i16];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_y;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_b_y[24];
  int32_T c2_i17;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_y = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_y), &c2_thisId, c2_b_y);
  sf_mex_destroy(&c2_y);
  for (c2_i17 = 0; c2_i17 < 24; c2_i17++) {
    (*(uint8_T (*)[24])c2_outData)[c2_i17] = c2_b_y[c2_i17];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i18;
  real_T c2_b_inData[9];
  int32_T c2_i19;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i18 = 0; c2_i18 < 9; c2_i18++) {
    c2_b_inData[c2_i18] = (*(real_T (*)[9])c2_inData)[c2_i18];
  }

  for (c2_i19 = 0; c2_i19 < 9; c2_i19++) {
    c2_u[c2_i19] = c2_b_inData[c2_i19];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d1;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d1, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i20;
  uint8_T c2_b_inData[4];
  int32_T c2_i21;
  uint8_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i20 = 0; c2_i20 < 4; c2_i20++) {
    c2_b_inData[c2_i20] = (*(uint8_T (*)[4])c2_inData)[c2_i20];
  }

  for (c2_i21 = 0; c2_i21 < 4; c2_i21++) {
    c2_u[c2_i21] = c2_b_inData[c2_i21];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 3, 0U, 1U, 0U, 2, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_d_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_y[4])
{
  uint8_T c2_uv5[4];
  int32_T c2_i22;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_uv5, 1, 3, 0U, 1, 0U, 2, 1, 4);
  for (c2_i22 = 0; c2_i22 < 4; c2_i22++) {
    c2_y[c2_i22] = c2_uv5[c2_i22];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_servoValueinBytes;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y[4];
  int32_T c2_i23;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_servoValueinBytes = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_servoValueinBytes),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_servoValueinBytes);
  for (c2_i23 = 0; c2_i23 < 4; c2_i23++) {
    (*(uint8_T (*)[4])c2_outData)[c2_i23] = c2_y[c2_i23];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  uint32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(uint32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static uint32_T c2_e_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint32_T c2_y;
  uint32_T c2_u1;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u1, 1, 7, 0U, 0, 0U, 0);
  c2_y = c2_u1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_checksum;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint32_T c2_y;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_checksum = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_checksum),
    &c2_thisId);
  sf_mex_destroy(&c2_checksum);
  *(uint32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i24;
  uint32_T c2_b_inData[9];
  int32_T c2_i25;
  uint32_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i24 = 0; c2_i24 < 9; c2_i24++) {
    c2_b_inData[c2_i24] = (*(uint32_T (*)[9])c2_inData)[c2_i24];
  }

  for (c2_i25 = 0; c2_i25 < 9; c2_i25++) {
    c2_u[c2_i25] = c2_b_inData[c2_i25];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 7, 0U, 1U, 0U, 1, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  uint32_T c2_y[9])
{
  uint32_T c2_uv6[9];
  int32_T c2_i26;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_uv6, 1, 7, 0U, 1, 0U, 1, 9);
  for (c2_i26 = 0; c2_i26 < 9; c2_i26++) {
    c2_y[c2_i26] = c2_uv6[c2_i26];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_servo32Bit;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint32_T c2_y[9];
  int32_T c2_i27;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_servo32Bit = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_servo32Bit), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_servo32Bit);
  for (c2_i27 = 0; c2_i27 < 9; c2_i27++) {
    (*(uint32_T (*)[9])c2_outData)[c2_i27] = c2_y[c2_i27];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  uint8_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(uint8_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static uint8_T c2_g_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_startByte2, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_startByte2),
    &c2_thisId);
  sf_mex_destroy(&c2_startByte2);
  return c2_y;
}

static uint8_T c2_h_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u2;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u2, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u2;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_startByte2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_startByte2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_startByte2),
    &c2_thisId);
  sf_mex_destroy(&c2_startByte2);
  *(uint8_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_simulink2CRRCSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[10];
  c2_ResolvedFunctionInfo (*c2_b_info)[10];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i28;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_b_info = (c2_ResolvedFunctionInfo (*)[10])c2_info;
  (*c2_b_info)[0].context = "";
  (*c2_b_info)[0].name = "mtimes";
  (*c2_b_info)[0].dominantType = "double";
  (*c2_b_info)[0].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c2_b_info)[0].fileTimeLo = 1289519692U;
  (*c2_b_info)[0].fileTimeHi = 0U;
  (*c2_b_info)[0].mFileTimeLo = 0U;
  (*c2_b_info)[0].mFileTimeHi = 0U;
  (*c2_b_info)[1].context = "";
  (*c2_b_info)[1].name = "typecast";
  (*c2_b_info)[1].dominantType = "uint32";
  (*c2_b_info)[1].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m";
  (*c2_b_info)[1].fileTimeLo = 1307651236U;
  (*c2_b_info)[1].fileTimeHi = 0U;
  (*c2_b_info)[1].mFileTimeLo = 0U;
  (*c2_b_info)[1].mFileTimeHi = 0U;
  (*c2_b_info)[2].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m!bytes_per_element";
  (*c2_b_info)[2].name = "eml_int_nbits";
  (*c2_b_info)[2].dominantType = "char";
  (*c2_b_info)[2].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_int_nbits.m";
  (*c2_b_info)[2].fileTimeLo = 1286818780U;
  (*c2_b_info)[2].fileTimeHi = 0U;
  (*c2_b_info)[2].mFileTimeLo = 0U;
  (*c2_b_info)[2].mFileTimeHi = 0U;
  (*c2_b_info)[3].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m!bytes_per_element";
  (*c2_b_info)[3].name = "eml_index_rdivide";
  (*c2_b_info)[3].dominantType = "uint8";
  (*c2_b_info)[3].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_rdivide.m";
  (*c2_b_info)[3].fileTimeLo = 1286818780U;
  (*c2_b_info)[3].fileTimeHi = 0U;
  (*c2_b_info)[3].mFileTimeLo = 0U;
  (*c2_b_info)[3].mFileTimeHi = 0U;
  (*c2_b_info)[4].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_rdivide.m";
  (*c2_b_info)[4].name = "eml_index_class";
  (*c2_b_info)[4].dominantType = "";
  (*c2_b_info)[4].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c2_b_info)[4].fileTimeLo = 1286818778U;
  (*c2_b_info)[4].fileTimeHi = 0U;
  (*c2_b_info)[4].mFileTimeLo = 0U;
  (*c2_b_info)[4].mFileTimeHi = 0U;
  (*c2_b_info)[5].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m";
  (*c2_b_info)[5].name = "eml_index_times";
  (*c2_b_info)[5].dominantType = "int32";
  (*c2_b_info)[5].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  (*c2_b_info)[5].fileTimeLo = 1286818780U;
  (*c2_b_info)[5].fileTimeHi = 0U;
  (*c2_b_info)[5].mFileTimeLo = 0U;
  (*c2_b_info)[5].mFileTimeHi = 0U;
  (*c2_b_info)[6].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  (*c2_b_info)[6].name = "eml_index_class";
  (*c2_b_info)[6].dominantType = "";
  (*c2_b_info)[6].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c2_b_info)[6].fileTimeLo = 1286818778U;
  (*c2_b_info)[6].fileTimeHi = 0U;
  (*c2_b_info)[6].mFileTimeLo = 0U;
  (*c2_b_info)[6].mFileTimeHi = 0U;
  (*c2_b_info)[7].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m";
  (*c2_b_info)[7].name = "eml_index_rdivide";
  (*c2_b_info)[7].dominantType = "int32";
  (*c2_b_info)[7].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_rdivide.m";
  (*c2_b_info)[7].fileTimeLo = 1286818780U;
  (*c2_b_info)[7].fileTimeHi = 0U;
  (*c2_b_info)[7].mFileTimeLo = 0U;
  (*c2_b_info)[7].mFileTimeHi = 0U;
  (*c2_b_info)[8].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m";
  (*c2_b_info)[8].name = "eml_index_class";
  (*c2_b_info)[8].dominantType = "";
  (*c2_b_info)[8].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c2_b_info)[8].fileTimeLo = 1286818778U;
  (*c2_b_info)[8].fileTimeHi = 0U;
  (*c2_b_info)[8].mFileTimeLo = 0U;
  (*c2_b_info)[8].mFileTimeHi = 0U;
  (*c2_b_info)[9].context =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/datatypes/typecast.m";
  (*c2_b_info)[9].name = "eml_unsigned_class";
  (*c2_b_info)[9].dominantType = "char";
  (*c2_b_info)[9].resolved =
    "[ILXE]D:/Matlab/R2012a/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  (*c2_b_info)[9].fileTimeLo = 1286818800U;
  (*c2_b_info)[9].fileTimeHi = 0U;
  (*c2_b_info)[9].mFileTimeLo = 0U;
  (*c2_b_info)[9].mFileTimeHi = 0U;
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 10), FALSE);
  for (c2_i28 = 0; c2_i28 < 10; c2_i28++) {
    c2_r0 = &c2_info[c2_i28];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i28);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i28);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_typecast(SFc2_simulink2CRRCSimInstanceStruct *chartInstance,
  uint32_T c2_x, uint8_T c2_y[4])
{
  memcpy(&c2_y[0], &c2_x, 4U);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_i_emlrt_marshallIn(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i29;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i29, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i29;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static void init_dsm_address_info(SFc2_simulink2CRRCSimInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_simulink2CRRCSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3767372435U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4162988101U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1582712915U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1581278641U);
}

mxArray *sf_c2_simulink2CRRCSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("WVUxNvWMCPBb7ja1GFdnOC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(9);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(24);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c2_simulink2CRRCSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c2_simulink2CRRCSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_simulink2CRRCSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
    chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_simulink2CRRCSimMachineNumber_,
           2,
           1,
           1,
           2,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_simulink2CRRCSimMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_simulink2CRRCSimMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_simulink2CRRCSimMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"inputs");
          _SFD_SET_DATA_PROPS(1,2,0,1,"y");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,2,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",147,-1,1307);
        _SFD_CV_INIT_EML_FOR(0,1,0,514,524,794);
        _SFD_CV_INIT_EML_FOR(0,1,1,1011,1024,1221);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 9;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 24;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          real_T (*c2_inputs)[9];
          uint8_T (*c2_y)[24];
          c2_y = (uint8_T (*)[24])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_inputs = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_inputs);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_y);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_simulink2CRRCSimMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "dK2dQufL0fBinHXmefkGEE";
}

static void sf_opaque_initialize_c2_simulink2CRRCSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInstanceVar);
  initialize_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_simulink2CRRCSim(void *chartInstanceVar)
{
  enable_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_simulink2CRRCSim(void *chartInstanceVar)
{
  disable_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_simulink2CRRCSim(void *chartInstanceVar)
{
  sf_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_simulink2CRRCSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_simulink2CRRCSim
    ((SFc2_simulink2CRRCSimInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_simulink2CRRCSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_simulink2CRRCSim(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_simulink2CRRCSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_simulink2CRRCSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_simulink2CRRCSim(S);
}

static void sf_opaque_set_sim_state_c2_simulink2CRRCSim(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_simulink2CRRCSim(S, st);
}

static void sf_opaque_terminate_c2_simulink2CRRCSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_simulink2CRRCSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_simulink2CRRCSim_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_simulink2CRRCSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_simulink2CRRCSim((SFc2_simulink2CRRCSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_simulink2CRRCSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_simulink2CRRCSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1822382716U));
  ssSetChecksum1(S,(1117885479U));
  ssSetChecksum2(S,(2458264162U));
  ssSetChecksum3(S,(1941148266U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_simulink2CRRCSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_simulink2CRRCSim(SimStruct *S)
{
  SFc2_simulink2CRRCSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulink2CRRCSimInstanceStruct *)malloc(sizeof
    (SFc2_simulink2CRRCSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_simulink2CRRCSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_simulink2CRRCSim;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_simulink2CRRCSim;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_simulink2CRRCSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_simulink2CRRCSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_simulink2CRRCSim;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_simulink2CRRCSim;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_simulink2CRRCSim;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_simulink2CRRCSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_simulink2CRRCSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_simulink2CRRCSim;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_simulink2CRRCSim;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_simulink2CRRCSim_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_simulink2CRRCSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_simulink2CRRCSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_simulink2CRRCSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_simulink2CRRCSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
