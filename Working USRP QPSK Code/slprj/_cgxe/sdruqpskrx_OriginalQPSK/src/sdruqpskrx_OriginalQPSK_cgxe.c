/* Include files */

#include "sdruqpskrx_OriginalQPSK_cgxe.h"
#include "m_jkd5PUo6DxuR6cIHGiSZr.h"
#include "m_4DoyfzVWrL7ZTU2c84qGl.h"
#include "m_9wL5t4Wqkf4qe0Xm8Vg82C.h"
#include "m_gNp5vVXUljDcfIAvnPLG7F.h"

unsigned int cgxe_sdruqpskrx_OriginalQPSK_method_dispatcher(SimStruct* S, int_T
  method, void* data)
{
  if (ssGetChecksum0(S) == 1121851519 &&
      ssGetChecksum1(S) == 851427689 &&
      ssGetChecksum2(S) == 3034969588 &&
      ssGetChecksum3(S) == 3944909566) {
    method_dispatcher_jkd5PUo6DxuR6cIHGiSZr(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 1323789718 &&
      ssGetChecksum1(S) == 207350879 &&
      ssGetChecksum2(S) == 3187555614 &&
      ssGetChecksum3(S) == 1243374531) {
    method_dispatcher_4DoyfzVWrL7ZTU2c84qGl(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 1574239491 &&
      ssGetChecksum1(S) == 3905595136 &&
      ssGetChecksum2(S) == 2199938747 &&
      ssGetChecksum3(S) == 835869757) {
    method_dispatcher_9wL5t4Wqkf4qe0Xm8Vg82C(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2652991982 &&
      ssGetChecksum1(S) == 40759377 &&
      ssGetChecksum2(S) == 53016139 &&
      ssGetChecksum3(S) == 1266577320) {
    method_dispatcher_gNp5vVXUljDcfIAvnPLG7F(S, method, data);
    return 1;
  }

  return 0;
}
