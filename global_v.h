#ifndef GLOBAL_V_H
#define GLOBAL_V_H

#include "protocol_conversion.h"

drone_platform_status_feedback_data_t ProtocolConversion::feedback_data = {0};
int32_t ProtocolConversion::amsl_alt = 0;
list<route_setting_t> ProtocolConversion::_routeList = {};
uint8_t ProtocolConversion::_base_mode = 0;
uint32_t ProtocolConversion::_custom_mode = 0;
list<mavlink_mission_item_int_t> ProtocolConversion::mission_item_int_list = {};
uint8_t ProtocolConversion::sender_sysid = 0;
uint8_t ProtocolConversion::receiver_sysid = 0;
route_download_t ProtocolConversion::route_download = {};


#endif // GLOBAL_V_H