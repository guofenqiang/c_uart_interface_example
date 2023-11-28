#ifndef GLOBAL_V_H
#define GLOBAL_V_H

#include "protocol_conversion.h"

drone_platform_status_feedback_data_t ProtocolConversion::feedback_data = {0};
int32_t ProtocolConversion::amsl_alt = 0;
list<route_setting_t> ProtocolConversion::_routeList = {};


#endif // GLOBAL_V_H