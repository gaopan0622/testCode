//
// Created by xiaotj on 2/19/20.
//

#ifndef SRC_LOCATIONSERVERERROR_H
#define SRC_LOCATIONSERVERERROR_H
#include "common/enum.h"
BETTER_ENUM(LocationServerError, int,
            UNDEFINED_ERROR = 120000,
            PARAMETER_READ_ERROR= 120001,
            GPS_POSITION_TIMEOUT = 120002,
            GPS_HEADING_TIMEOUT = 120003,
            GPS_POSITION_JITTER = 120004,
            GPS_HEADING_JITTER = 120005,
            IMU_DATA_TIMEOUT = 120006,
            VEHICLE_DATA_TIMEOUT = 120007,
            ILLEGAL_TASK_STATUS= 120008,
            POSITIONING_ERROR = 120009,
            DIRECTIONAL_ERROR = 120010,
            EXPIRING_SOON = 120011,
            IMU_UNUSUAL = 120012,
            VEH_UNUSUAL = 120013,
            ROBOT_HEADING_UNINIT = 120014
);
#endif //SRC_LOCATIONSERVERERROR_H