/* Include files */

#include "sdruqpskrx_OriginalQPSK_cgxe.h"
#include "m_UxOvoSHoDqYMTVOX24wWpF.h"
#include "m_1HhAkxmSPWHD9TGYKVFeaD.h"
#include "m_WWp8y15qiWHT0yMoxQngXF.h"
#include "m_eXXzoE6PEnLYV4PxmR72iF.h"

unsigned int cgxe_sdruqpskrx_OriginalQPSK_method_dispatcher(SimStruct* S, int_T
  method, void* data)
{
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

  if (ssGetChecksum0(S) == 3872411235 &&
      ssGetChecksum1(S) == 1231707858 &&
      ssGetChecksum2(S) == 2528044730 &&
      ssGetChecksum3(S) == 2590252956) {
    method_dispatcher_eXXzoE6PEnLYV4PxmR72iF(S, method, data);
    return 1;
  }

  return 0;
}
