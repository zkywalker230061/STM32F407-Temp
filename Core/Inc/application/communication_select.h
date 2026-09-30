#ifndef COMMUNICATION_SELECT_H
#define COMMUNICATION_SELECT_H

#include "common/error_code.h"
#include "common/system_config.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t communication_select_initialize(void);

ErrorCode_t communication_select_mode(SystemCommunicationMode_t mode);

ErrorCode_t communication_select_process(void);

#ifdef __cplusplus
}
#endif

#endif  /* COMMUNICATION_SELECT_H */
