#ifndef _LIBS_TPU_EDGETPU_OP_H_
#define _LIBS_TPU_EDGETPU_OP_H_

struct TfLiteRegistration;

namespace valiant {

TfLiteRegistration* RegisterCustomOp();

}  // namespace valiant

#endif  // _LIBS_TPU_EDGETPU_OP_H_