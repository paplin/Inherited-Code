#define S_FUNCTION_LEVEL               2
#define S_FUNCTION_NAME                sdruqpskrx_OriginalQPSK_cgxe
#include "simstruc.h"
#include "sdruqpskrx_OriginalQPSK_cgxe.h"
#define MDL_START

static void mdlStart(SimStruct* S)
{
  unsigned int success;
  success = cgxe_sdruqpskrx_OriginalQPSK_method_dispatcher(S, SS_CALL_MDL_START,
    NULL);
  if (!success) {
    /* error */
    mexPrintf("ERROR: Failed to dispatch s-function method!\n");
  }
}

#define MDL_INITIALIZE_CONDITIONS

static void mdlInitializeConditions(SimStruct *S)
{
  mexPrintf("ERROR: Calling model mdlInitializeConditions method directly.\n");
}

#define MDL_UPDATE

static void mdlUpdate(SimStruct *S, int_T tid)
{
  mexPrintf("ERROR: Calling model mdlUpdate method directly.\n");
}

static void mdlOutputs(SimStruct* S, int_T tid)
{
  mexPrintf("ERROR: Calling model mdlOutputs method directly.\n");
}

static void mdlTerminate(SimStruct *S)
{
  mexPrintf("ERROR: Calling model mdlTerminate method directly.\n");
}

static void mdlInitializeSizes(SimStruct *S)
{
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
}

static mxArray* cgxe_get_supported_modules(void)
{
  mxArray* mxModules = mxCreateCellMatrix(4, 1);
  mxArray* mxChksum = NULL;
  uint32_T* checksumData = NULL;
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 1323789718;
  checksumData[1] = 207350879;
  checksumData[2] = 3187555614;
  checksumData[3] = 1243374531;
  mxSetCell(mxModules, 0, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 1574239491;
  checksumData[1] = 3905595136;
  checksumData[2] = 2199938747;
  checksumData[3] = 835869757;
  mxSetCell(mxModules, 1, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 2025989489;
  checksumData[1] = 994100710;
  checksumData[2] = 4151493251;
  checksumData[3] = 3483939271;
  mxSetCell(mxModules, 2, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 2652991982;
  checksumData[1] = 40759377;
  checksumData[2] = 53016139;
  checksumData[3] = 1266577320;
  mxSetCell(mxModules, 3, mxChksum);
  return mxModules;
}

static int cgxe_process_get_checksums(int nlhs, mxArray* plhs[], int nrhs, const
  mxArray* prhs[])
{
  const char* checksumFields[] = { "modules", "model", "makefile", "target",
    "overall" };

  mxArray* mxChecksum = mxCreateStructMatrix(1, 1, 5, checksumFields);
  mxSetField(mxChecksum, 0, "modules", cgxe_get_supported_modules());

  {
    mxArray* mxModelChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxModelChksum);
    checksumData[0] = 3349439775;
    checksumData[1] = 3535689605;
    checksumData[2] = 3196406510;
    checksumData[3] = 3746207727;
    mxSetField(mxChecksum, 0, "model", mxModelChksum);
  }

  {
    mxArray* mxMakefileChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxMakefileChksum);
    checksumData[0] = 4135335210;
    checksumData[1] = 3318797983;
    checksumData[2] = 1681312694;
    checksumData[3] = 3539842518;
    mxSetField(mxChecksum, 0, "makefile", mxMakefileChksum);
  }

  {
    mxArray* mxTargetChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxTargetChksum);
    checksumData[0] = 0;
    checksumData[1] = 0;
    checksumData[2] = 0;
    checksumData[3] = 0;
    mxSetField(mxChecksum, 0, "target", mxTargetChksum);
  }

  {
    mxArray* mxOverallChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxOverallChksum);
    checksumData[0] = 3294438020;
    checksumData[1] = 2929798654;
    checksumData[2] = 3777628939;
    checksumData[3] = 1652255522;
    mxSetField(mxChecksum, 0, "overall", mxOverallChksum);
  }

  plhs[0] = mxChecksum;
  return 1;
}

static int cgxe_mex_unlock_call(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  while (mexIsLocked()) {
    mexUnlock();
  }

  return 1;
}

static SimStruct* cgxe_unpack_simstruct(const mxArray *mxS)
{
  uint32_T *uintPtr = (uint32_T*)malloc(sizeof(SimStruct*));
  int nEl = sizeof(SimStruct*)/sizeof(uint32_T);
  uint32_T *uintDataPtr = (uint32_T *)mxGetData(mxS);
  int el;
  SimStruct *S;
  for (el=0; el < nEl; el++) {
    uintPtr[el] = uintDataPtr[el];
  }

  memcpy(&S,uintPtr,sizeof(SimStruct*));
  free(uintPtr);
  return S;
}

static int cgxe_get_sim_state(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  unsigned int success;
  SimStruct *S = cgxe_unpack_simstruct(prhs[1]);
  success = cgxe_sdruqpskrx_OriginalQPSK_method_dispatcher(S,
    SS_CALL_MDL_GET_SIM_STATE, (void *) (plhs));
  if (!success) {
    /* error */
    mexPrintf("ERROR: Failed to dispatch s-function method!\n");
  }

  return 1;
}

static int cgxe_set_sim_state(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  unsigned int success;
  SimStruct *S = cgxe_unpack_simstruct(prhs[1]);
  success = cgxe_sdruqpskrx_OriginalQPSK_method_dispatcher(S,
    SS_CALL_MDL_SET_SIM_STATE, (void *) prhs[2]);
  if (!success) {
    /* error */
    mexPrintf("ERROR: Failed to dispatch s-function method!\n");
  }

  return 1;
}

static int cgxe_get_BuildInfoUpdate(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(tpChksum, "4DoyfzVWrL7ZTU2c84qGl") == 0) {
    extern mxArray *cgxe_4DoyfzVWrL7ZTU2c84qGl_BuildInfoUpdate(void);
    plhs[0] = cgxe_4DoyfzVWrL7ZTU2c84qGl_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "9wL5t4Wqkf4qe0Xm8Vg82C") == 0) {
    extern mxArray *cgxe_9wL5t4Wqkf4qe0Xm8Vg82C_BuildInfoUpdate(void);
    plhs[0] = cgxe_9wL5t4Wqkf4qe0Xm8Vg82C_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "8rrCCyrtiOptd4UQNBcUyG") == 0) {
    extern mxArray *cgxe_8rrCCyrtiOptd4UQNBcUyG_BuildInfoUpdate(void);
    plhs[0] = cgxe_8rrCCyrtiOptd4UQNBcUyG_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "gNp5vVXUljDcfIAvnPLG7F") == 0) {
    extern mxArray *cgxe_gNp5vVXUljDcfIAvnPLG7F_BuildInfoUpdate(void);
    plhs[0] = cgxe_gNp5vVXUljDcfIAvnPLG7F_BuildInfoUpdate();
    return 1;
  }

  return 0;
}

static int cgxe_get_fallback_info(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(tpChksum, "4DoyfzVWrL7ZTU2c84qGl") == 0) {
    extern mxArray *cgxe_4DoyfzVWrL7ZTU2c84qGl_fallback_info(void);
    plhs[0] = cgxe_4DoyfzVWrL7ZTU2c84qGl_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "9wL5t4Wqkf4qe0Xm8Vg82C") == 0) {
    extern mxArray *cgxe_9wL5t4Wqkf4qe0Xm8Vg82C_fallback_info(void);
    plhs[0] = cgxe_9wL5t4Wqkf4qe0Xm8Vg82C_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "8rrCCyrtiOptd4UQNBcUyG") == 0) {
    extern mxArray *cgxe_8rrCCyrtiOptd4UQNBcUyG_fallback_info(void);
    plhs[0] = cgxe_8rrCCyrtiOptd4UQNBcUyG_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "gNp5vVXUljDcfIAvnPLG7F") == 0) {
    extern mxArray *cgxe_gNp5vVXUljDcfIAvnPLG7F_fallback_info(void);
    plhs[0] = cgxe_gNp5vVXUljDcfIAvnPLG7F_fallback_info();
    return 1;
  }

  return 0;
}

#define PROCESS_MEX_SFUNCTION_CMD_LINE_CALL

static int ProcessMexSfunctionCmdLineCall(int nlhs, mxArray* plhs[], int nrhs,
  const mxArray* prhs[])
{
  char commandName[64];
  if (nrhs < 1 || !mxIsChar(prhs[0]))
    return 0;
  mxGetString(prhs[0], commandName, sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName, "get_checksums") == 0) {
    return cgxe_process_get_checksums(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "mex_unlock") == 0) {
    return cgxe_mex_unlock_call(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "get_sim_state") == 0) {
    return cgxe_get_sim_state(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "set_sim_state") == 0) {
    return cgxe_set_sim_state(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "get_BuildInfoUpdate") == 0) {
    return cgxe_get_BuildInfoUpdate(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "get_fallback_info") == 0) {
    return cgxe_get_fallback_info(nlhs, plhs, nrhs, prhs);
  }

  return 0;
}

#include "simulink.c"
