#ifndef SRC_GPSERROR_H
#define SRC_GPSERROR_H
#include <common/common_all.h>

BETTER_ENUM(GpsError, int,
            UNDEFID_ERROR = 220000,
            RTK_CHECK_ERROR = 220001,
            RTK_DATA_LOST = 220002,
            POOR_HEADING_DATA = 220003,
            POOR_POS_DATA = 220004
);
#endif //SRC_GPSERROR_
