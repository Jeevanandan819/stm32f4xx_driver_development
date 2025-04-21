#include <stdint.h>

typedef uint32_t st_status_t;

#define ST_STATUS_SUCCESS                      (st_status_t)0x00000000
#define ST_STATUS_ERROR                        (st_status_t)0x00000001

#define ST_STATUS_OK                           (st_status_t)0x00000000
#define ST_STATUS_FAIL                         (st_status_t)0x00000001
#define ST_STATUS_INVALID_PARAMETER            (st_status_t)0x00000002
#define ST_STATUS_BUSY                         (st_status_t)0x00000003
#define ST_STATUS_INVALID_RANGE                (st_status_t)0x00000004
#define ST_STATUS_NOT_INITIALIZED              (st_status_t)0x00000005                         