/* Include files */

#include "sdruqpskrx_BPSK_cgxe.h"
#include "m_KTMVWZ2HExLqnZ2FNJOdrC.h"
#include "m_UxOvoSHoDqYMTVOX24wWpF.h"
#include "m_1HhAkxmSPWHD9TGYKVFeaD.h"
#include "m_WWp8y15qiWHT0yMoxQngXF.h"

unsigned int cgxe_sdruqpskrx_BPSK_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 1543320287 &&
      ssGetChecksum1(S) == 3272310553 &&
      ssGetChecksum2(S) == 1891906266 &&
      ssGetChecksum3(S) == 1407701533) {
    method_dispatcher_KTMVWZ2HExLqnZ2FNJOdrC(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2043223478 &&
      ssGetChecksum1(S) == 193077703 &&
      ssGetChecksum2(S) == 645941016 &&
      ssGetChecksum3(S) == 1114361504) {
    method_dispatcher_UxOvoSHoDqYMTVOX24wWpF(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2058956287 &&
      ssGetChecksum1(S) == 2384873423 &&
      ssGetChecksum2(S) == 1212798989 &&
      ssGetChecksum3(S) == 2041892846) {
    method_dispatcher_1HhAkxmSPWHD9TGYKVFeaD(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3363663004 &&
      ssGetChecksum1(S) == 4255815326 &&
      ssGetChecksum2(S) == 1038040466 &&
      ssGetChecksum3(S) == 2922161120) {
    method_dispatcher_WWp8y15qiWHT0yMoxQngXF(S, method, data);
    return 1;
  }

  return 0;
}
