#include "protocol_conversion.h"
#include "BZConv.h"
#include "CRCCheck.h"
#include <iostream>
#include "global_v.h"


ProtocolConversion::ProtocolConversion(Generic_Port *port_)
{
    port = port_;
    init_px4();
}

ProtocolConversion::ProtocolConversion(Generic_Port *port_, Generic_Port *dest_port_)
{
    port = port_;
    dest_port = dest_port_;
    init_px4();
}

ProtocolConversion::~ProtocolConversion()
{

}

void ProtocolConversion::bz_ground_to_uav(uint8_t *dest, uint8_t *src)
{
    uint8_t UAVBuffer[BZ_UAV_UPSTREAM_LEN] = {0};

    BZ_STATUS result = BZ_Ground2UAV(UAVBuffer, (uint8_t *)src);
    if (result == BZ_OK) {
        memcpy((uint8_t*)dest, UAVBuffer, BZ_UAV_UPSTREAM_LEN);
    } else if (result == BZ_INVALID_ADDRESS){
        printf("BZ_INVALID_ADDRESS\n");
    } else if (result == BZ_CRC_UNPASSED) {
        printf("BZ_CRC_UNPASSED\n");
    }

    return;
}

void ProtocolConversion::bz_uav_to_ground()
{

}

void ProtocolConversion::bz_telecontrol_decode(char *buff, int len)
{
    bz_message_uav_up_t bz_message = {};

    memcpy((uint8_t *)&bz_message.frame_header1, (uint8_t *)buff, sizeof(bz_message));

    switch (bz_message.cmd0 << 8 | bz_message.cmd1) {
        case MANUAL_MODE:
        {
            virtual_rocker_mode(bz_message);
            break;
        }

        case AUTONOMOUS_TAKEOFF:
        {
            autonomous_takeoff(bz_message);
            break;
        }

        case AUTONOMOUS_RETURN:
        {
            autonomous_return(bz_message);
            break;
        }

        case AUTONOMOUS_CRUISE:
        {
            autonomous_cruise(bz_message);
            break;
        }

        case AUTONOMOUS_FLIGHT_AND_STEERING:
        {
            autonomous_flight_and_steering(bz_message);
            break;
        }

        case ROUTE_SETTING:
        {
            route_setting(bz_message);
            break;
        }
        
        case ROUTE_FLIGHT_INSTRUCTIONS:
        {
            route_flight_instructions(bz_message);
            break;
        }

        case GEOGRAPHIC_COORDINATE_GUIDANCE:
        {
            geographic_coordinage_guidance(bz_message);
            break;
        }

        case ROUTE_DOWNLOAD_SWITCH:
        {
            route_downlaod_switch(bz_message);
            break;
        }

        case AUTONOMOUS_PRECISION_LAND:
        {
            autonomous_precision_land(bz_message);
            break;
        }

        case ROUTE_DOWNLOAD_REPLY:
        {
            route_download_reply(bz_message);
            break;
        }

        default :
        {
            invalid_teleconrol_cmd(bz_message);
            break; 
        }
        
        // 每收到一次遥控指令都需要发送一次指令反馈恢复
        command_feedback_response(bz_message);
    }

    // port->dest_port->write_port(buff, len);
}

void ProtocolConversion::handle_message(mavlink_message_t & message)
{
        // Mavlink_Messages current_messages;
    mavlink_heartbeat_t heartbeat;
    mavlink_attitude_t attitude;

    bz_message_ground_down_t bz_message = {0};

    mavlink_mission_count_t mission_count;
    // printf("msgid: %d\n", message.msgid);
    // Handle Message ID
    switch (message.msgid)
    {
        sender_sysid = message.msgid;
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            _base_mode = heartbeat.base_mode;
            _custom_mode = heartbeat.custom_mode;
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&message, &sys_status);

            if (sys_status.current_battery > 0) {
                feedback_data.battery_voltage = sys_status.current_battery/1000 * 10;
            } else {
                feedback_data.battery_voltage = 0;
            }          
            
            if (sys_status.battery_remaining > 0)
            {
                feedback_data.percentage_of_remaining_battery_power = sys_status.battery_remaining;
            } else {
                feedback_data.percentage_of_remaining_battery_power = 0;
            }
            
            // printf("voltage: %d, remain: %d\n", feedback_data.battery_voltage, feedback_data.percentage_of_remaining_battery_power);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
            mavlink_msg_attitude_decode(&message, &attitude);
            feedback_data.roll_angle = attitude.roll * 57.3 * 100;
            feedback_data.pitch_angle = attitude.pitch * 57.3 * 100;
            if (attitude.yaw >= 0) {
                feedback_data.yaw_angle = attitude.yaw * 57.3 * 10;
            } else {
                feedback_data.yaw_angle = (360 + attitude.yaw * 57.3) * 10;
            }

            feedback_data.platform_status = cov_flight_status(flightMode(_base_mode, _custom_mode));

            feedback_data.startup_time = attitude.time_boot_ms / 1000;
            // printf("roll: %d, pitch: %d, yaw: %d\n", 
            //         feedback_data.roll_angle / 100, 
            //         feedback_data.pitch_angle / 100,
            //         feedback_data.yaw_angle / 10);
            break;
        }

        // case MAVLINK_MSG_ID_ALTITUDE:
        // {
        //     mavlink_altitude_t altitude;
        //     mavlink_msg_altitude_decode(&message, &altitude);

        //     feedback_data.relative_height = (int16_t)altitude.altitude_relative * 10;
        // }

        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_gps_raw_int_t gps_raw_int;
            mavlink_msg_gps_raw_int_decode(&message, &gps_raw_int);

            feedback_data.latitude = gps_raw_int.lat;
            feedback_data.longitude = gps_raw_int.lon;
            amsl_alt = gps_raw_int.alt;

            // feedback_data.latitude = 300181923;
            // feedback_data.longitude = 1199695779;

            // printf("latitude: %d, longitude: %d, altitude(mm): %d\n",
            //          feedback_data.latitude, 
            //          feedback_data.longitude, 
            //          amsl_alt);

            break;
        }

        // case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
        // {
        //     mavlink_gps_global_origin_t gps_global_origin;
        //     mavlink_msg_gps_global_origin_decode(&message, &gps_global_origin);

        //     feedback_data.latitude = gps_global_origin.latitude;
        //     feedback_data.longitude = gps_global_origin.longitude;
        //     feedback_data.relative_height = gps_global_origin.altitude;

        //     printf("latitude: %d, longitude: %d, altitude: %d\n",
        //             feedback_data.latitude, 
        //             feedback_data.longitude, 
        //             feedback_data.relative_height);

        //     break;
        // }

        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        {
            mavlink_autopilot_version_t autopilot_version;

            mavlink_msg_autopilot_version_decode(&message, &autopilot_version);
            feedback_data.flight_control_version = 0x10;
            feedback_data.hw_version = 0x32; //临时添加
            printf("flight_control_version: %d, hw_version: %d\n", feedback_data.flight_control_version, feedback_data.hw_version);

            break;
        }
        
        case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            mavlink_message_t       messageOut;
            
            printf("MAVLINK_MSG_ID_MISSION_COUNT\n");
            mavlink_msg_mission_count_decode(&message, &mission_count);
            
            for (int i = 0; i < mission_count.count; i++) {
                mavlink_msg_mission_request_int_pack_chan(mission_count.target_system,
                                                          0,
                                                          0,
                                                          &messageOut,
                                                          message.sysid,
                                                          mission_count.target_component,
                                                          i,
                                                          MAV_MISSION_TYPE_MISSION);
                port->write_message(messageOut);                                           
            }

            break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        {
            mavlink_mission_item_t mission_item;
            mavlink_message_t       messageOut;

            mavlink_msg_mission_item_decode(&message, &mission_item);
            printf("seq: %d, x: %d, y: %d, z: %d,\n", mission_item.seq, mission_item.x, mission_item.y, mission_item.z);

            if (mission_item.seq == mission_count.count - 1) {
                mavlink_msg_mission_ack_pack_chan(mission_item.target_system,
                                                  0,
                                                  0,
                                                  &messageOut,
                                                  message.sysid,
                                                  mission_item.target_component,
                                                  0,
                                                  MAV_MISSION_TYPE_MISSION);
                port->write_message(messageOut);
            }

            break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            printf("MAVLINK_MSG_ID_MISSION_REQUEST\n");
        }
        
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        {
            printf("MAVLINK_MSG_ID_MISSION_REQUEST_INT\n");
            mavlink_message_t       messageOut;
            mavlink_mission_request_int_t missionRequest;
            route_setting_t route;

            mavlink_msg_mission_request_int_decode(&message, &missionRequest);
            printf("message.sysid: %d, target_system: %d, seq: %d\n", 
                    message.sysid,
                    missionRequest.target_system,
                    missionRequest.seq);
            int i = 0;
            for (route_setting_t element: _routeList){
                if (i == missionRequest.seq) {
                    route = element;
                    break;
                }
                ++i;
            }

            mavlink_msg_mission_item_int_pack_chan(missionRequest.target_system,
                                                   0,
                                                   0,
                                                   &messageOut,
                                                   message.sysid,
                                                   missionRequest.target_component,
                                                   missionRequest.seq,
                                                   MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                   MAV_CMD_NAV_WAYPOINT,
                                                   missionRequest.seq == 0,
                                                   1,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   route.latitude,
                                                   route.longitude,
                                                   route.relative_height / 10,
                                                   MAV_MISSION_TYPE_MISSION);
            port->write_message(messageOut);

            break;
        }

        case MAVLINK_MSG_ID_MISSION_ACK:
        {
            printf("MAVLINK_MSG_ID_MISSION_ACK\n");
            mavlink_mission_ack_t missionAck;
            
            mavlink_msg_mission_ack_decode(&message, &missionAck);

            // printf("%d, %d, %d, %d\n", missionAck.target_system, missionAck.target_component, missionAck.type, missionAck.mission_type);
            break;
        }

        case MAVLINK_MSG_ID_ALTITUDE:
        {
            mavlink_altitude_t alt;
            mavlink_msg_altitude_decode(&message, &alt);
            feedback_data.relative_height = alt.altitude_relative * 10;
        } 

        default :
        {
            // printf("msgid: %d\n", message.msgid);
            break;
        }
    }
}

void ProtocolConversion::bz_telemetry_decode(char *buff, int len)
{
    mavlink_message_t message;
    mavlink_status_t r_mavlink_status;

    for (int i = 0; i < len; i++) {
        if (mavlink_parse_char(0, uint8_t(buff[i]), &message, &r_mavlink_status)) {
            // Handle Message ID
            handle_message(message);   
        }
    }

    // port->dest_port->write_port(buff, len);
}

void ProtocolConversion::virtual_rocker_mode(bz_message_uav_up_t bz_message)
{
    printf("***************************************virtual_rocker_mode**************************************\n");

    mavlink_message_t msg;
    manual_mode_t manual_mode;

    memcpy((uint8_t*)&manual_mode.enable, (uint8_t*)&bz_message.pyload[0], sizeof(manual_mode));

    if (manual_mode.enable == 0x01) {
        set_flight_mode(bz_message, "Position");
        // Incoming values are in the range -1:1
        int16_t axesScaling =         1000/100;
        int16_t newRollCommand =      manual_mode.forward_speed * axesScaling;
        int16_t newPitchCommand  =    manual_mode.level_speed * axesScaling;    // Joystick data is reverse of mavlink values
        int16_t newYawCommand    =    manual_mode.uav_head_angle;
        int16_t newThrustCommand =    manual_mode.vtol_speed * axesScaling;

        mavlink_msg_manual_control_pack_chan(bz_message.sender_sysid,
                                             0,
                                             0,
                                             &msg,
                                             bz_message.receiver_sysid,
                                             static_cast<int16_t>(newPitchCommand),
                                             static_cast<int16_t>(newRollCommand),
                                             static_cast<int16_t>(newThrustCommand),
                                             static_cast<int16_t>(newYawCommand),
                                             0);

        dest_port->write_message(msg);
        printf("%d, %d, %d, %d\n", newRollCommand, newPitchCommand, newYawCommand, newThrustCommand);

    }



}
void ProtocolConversion::autonomous_takeoff(bz_message_uav_up_t bz_message)
{
    printf("***************************************autonomous_takeoff**************************************\n");

    mavlink_message_t  msg;
    mavlink_command_int_t  cmd;
    autonomous_takeoff_t mode;
    sender_sysid = bz_message.sender_sysid;
    receiver_sysid = bz_message.receiver_sysid;
    arm_disarm(true);

    memcpy((uint8_t*)&mode.enable, (uint8_t*)&bz_message.pyload[0], sizeof(mode));
    if (mode.enable == 0x01) {
        memset(&cmd, 0, sizeof(cmd));
        cmd.target_system = bz_message.receiver_sysid;
        cmd.target_component = 0;
        cmd.command = MAV_CMD_NAV_TAKEOFF;
        cmd.frame = MAV_FRAME_GLOBAL;
        cmd.param1 = -1;
        cmd.param2 = 0;
        cmd.param3 = 0;
        cmd.param4 = NAN;
        cmd.x = feedback_data.latitude;
        cmd.y = feedback_data.longitude;
        cmd.z = (float)mode.height / 10 + amsl_alt / 1000;

        mavlink_msg_command_int_encode_chan(bz_message.sender_sysid,
                                            0,
                                            0,
                                            &msg,
                                            &cmd);
        dest_port->write_message(msg);
        printf("yaw: %f, lat: %d, lon: %d, alt: %f\n", cmd.param4, cmd.x, cmd.y, cmd.z);
    }

}
void ProtocolConversion::autonomous_return(bz_message_uav_up_t bz_message)
{
    printf("***************************************autonomous_return**************************************\n");

    mavlink_message_t  msg;
    mavlink_command_int_t  cmd;

    autonomous_return_t mode;
    memcpy((uint8_t*)&mode.enable, (uint8_t*)&bz_message.pyload[0], sizeof(mode));

    if (mode.enable == 0x01) {
        bool flightModeChanged = false;
        set_flight_mode(bz_message, "Return");
    }

}
void ProtocolConversion::autonomous_cruise(bz_message_uav_up_t bz_message)
{
    printf("***************************************autonomous_cruise**************************************\n");

    route_flight_t route_flight;
    memcpy((uint8_t*)&route_flight.route_switch, (uint8_t*)&bz_message.pyload[0], sizeof(route_flight));

    if (route_flight.start_mode == 0) {
        set_flight_mode(bz_message, "Mission");
    } else {
        printf("not support start mode\n");
    }

}

void ProtocolConversion::autonomous_flight_and_steering(bz_message_uav_up_t bz_message)
{

    printf("***************************************autonomous_flight_and_steering**************************************\n");

    mavlink_message_t msg;
    autonomous_flight_t manual_mode;

    memcpy((uint8_t*)&manual_mode.act, (uint8_t*)&bz_message.pyload[0], sizeof(manual_mode));

    int16_t axesScaling =         100;
    int16_t angleScaling =        10;
    int16_t newRollCommand =      0;
    int16_t newPitchCommand  =    0;
    int16_t newYawCommand    =    0;
    int16_t newThrustCommand =    0;

    // set_flight_mode(bz_message, "Stabilized");

    if (manual_mode.act == 0x01) {
        newPitchCommand = manual_mode.virt_joy;
    } else if (manual_mode.act == 0x02) {
        newPitchCommand = -1 * manual_mode.virt_joy;
    } else if (manual_mode.act == 0x03) {
        newRollCommand = -1 * manual_mode.virt_joy;
    } else if (manual_mode.act == 0x04) {
        newRollCommand = manual_mode.virt_joy;
    } else if (manual_mode.act == 0x05) {
        newThrustCommand = manual_mode.virt_joy;
    } else if (manual_mode.act == 0x06) {
        newThrustCommand = -1 * manual_mode.virt_joy;
    } else if (manual_mode.act == 0x07) {
        newYawCommand = -1 * manual_mode.virt_joy;
    } else if (manual_mode.act == 0x08) {
        newYawCommand = manual_mode.virt_joy;
    }

    mavlink_msg_manual_control_pack_chan(bz_message.sender_sysid,
                                            0,
                                            0,
                                            &msg,
                                            bz_message.receiver_sysid,
                                            static_cast<int16_t>(newPitchCommand),
                                            static_cast<int16_t>(newRollCommand),
                                            static_cast<int16_t>(newThrustCommand),
                                            static_cast<int16_t>(newYawCommand),
                                            0);

    dest_port->write_message(msg);
    printf("%d, %d, %d, %d\n", newRollCommand, newPitchCommand, newYawCommand, newThrustCommand);

}

void ProtocolConversion::autonomous_obstacle_avoidance(bz_message_uav_up_t bz_message)
{
    printf("***************************************autonomous_obstacle_avoidance**************************************\n");
}
void ProtocolConversion::route_setting(bz_message_uav_up_t bz_message)
{
    printf("***************************************route_setting**************************************\n");
    mavlink_message_t       message;
    route_setting_t route;
    double latitude;
    double longtitude;
    double altitude;
    static int currentMissionIndex = 0;

    memcpy((uint8_t*)&route.upload, (uint8_t*)&bz_message.pyload[0], sizeof(route));
    latitude = (double)(route.latitude / 10e6);
    longtitude = (double)(route.longitude / 10e6);
    altitude = (double)(route.relative_height / 10);

    switch (route.route_type) {
        case SINGLE_ROUTE:
            printf("upload: %d, waypoint_numbers: %d, waypoint_id: %d, route_number: %d, \
                    route_type: %d, waypoint_type: %d, stay_time: %d, speed: %d\n", \
                    route.upload, route.waypoint_numbers, route.waypoint_id, route.route_number, 
                    route.route_type, route.waypoint_type, route.stay_time, route.relative_ground_speed);
            printf("lat: %d, lon: %d, alt: %d\n", route.latitude, route.longitude, route.relative_height);
            // 清除链表里保存的航线记录
            if (currentMissionIndex == 0) {
                _routeList.clear();
            }
            currentMissionIndex++;

            _routeList.push_back(route);
            if (route.waypoint_id == route.waypoint_numbers - 1) {
                if (_routeList.size() != route.waypoint_numbers) {
                    printf("_routeList error: %d\n", _routeList.size());
                    _routeList.clear();
                    currentMissionIndex = 0;
                    receiver_status = 0x02;
                    cmd_status = 0x02;
                    return;    
                }

                // 清除飞机上的航线
                mavlink_msg_mission_clear_all_pack_chan(bz_message.sender_sysid,
                                                        0,
                                                        0,
                                                        &message,
                                                        bz_message.receiver_sysid,
                                                        MAV_COMP_ID_AUTOPILOT1,
                                                        route.route_type);
                dest_port->write_message(message);

                // 请求发送当前绘制的航线个数，具体参照Mission Protocol
                mavlink_msg_mission_count_pack_chan(bz_message.sender_sysid,
                                    0,
                                    0,
                                    &message,
                                    bz_message.receiver_sysid,
                                    MAV_COMP_ID_AUTOPILOT1,
                                    route.waypoint_numbers,
                                    route.route_type);
                dest_port->write_message(message);

                currentMissionIndex = 0;
            }

            break;
        
        case TAKEOFF_ROUTE:
            break;
        
        case LAND_ROUTE:
            break;
        
        case CRUISE_ROUTE:
            break;
        
        case LEVEL_ROUTE:
            break;

        case HOMEWARD_ROUTE:
            break;

        case EMERGENCY_ROUTE:

            break;

        default:
        
            break; 
    }

    receiver_status = 0x01;
    cmd_status = 0x01;
}
void ProtocolConversion::route_flight_instructions(bz_message_uav_up_t bz_message)
{
    printf("***************************************route_flight_instructions**************************************\n");
}
void ProtocolConversion::geographic_coordinage_guidance(bz_message_uav_up_t bz_message)
{
    printf("***************************************geographic_coordinage_guidance**************************************\n");
}
void ProtocolConversion::route_downlaod_switch(bz_message_uav_up_t bz_message)
{
    printf("***************************************route_downlaod_switch**************************************\n");
    mavlink_message_t  message;
    route_download_t download;
    memcpy((uint8_t*)&download.route_number, (uint8_t*)&bz_message.pyload[0], sizeof(download));

    mavlink_msg_mission_request_list_pack_chan(bz_message.sender_sysid,
                                               0,
                                               0,
                                               &message,
                                               bz_message.receiver_sysid,
                                               MAV_COMP_ID_AUTOPILOT1,
                                               MAV_MISSION_TYPE_MISSION);

    dest_port->write_message(message);

}
void ProtocolConversion::autonomous_precision_land(bz_message_uav_up_t bz_message)
{
    printf("***************************************autonomous_precision_land**************************************\n");

    mavlink_message_t  msg;
    mavlink_command_int_t  cmd;
    autonomous_takeoff_t mode;

    memcpy((uint8_t*)&mode.enable, (uint8_t*)&bz_message.pyload[0], sizeof(mode));
    if (mode.enable == 0x01) {
        memset(&cmd, 0, sizeof(cmd));
        cmd.target_system = bz_message.receiver_sysid;
        cmd.target_component = 0;
        cmd.command = MAV_CMD_NAV_LAND;
        cmd.frame = MAV_FRAME_GLOBAL;
        cmd.param1 = -1;
        cmd.param2 = 0;
        cmd.param3 = 0;
        cmd.param4 = NAN;
        cmd.x = NAN;
        cmd.y = NAN;
        cmd.z = mode.height / 10;


        mavlink_msg_command_int_encode_chan(bz_message.sender_sysid,
                                            0,
                                            0,
                                            &msg,
                                            &cmd);
        dest_port->write_message(msg);
    }

}
void ProtocolConversion::route_download_reply(bz_message_uav_up_t bz_message)
{
    printf("***************************************route_download_reply**************************************\n");
}

void ProtocolConversion::command_feedback_response(bz_message_uav_up_t bz_message)
{
    command_feedback_response_t command_feedback_response;
	bz_message_ground_down_t message = {0};
    char buff[300];
	unsigned len;

    command_feedback_response.reciver_status = receiver_status;
    command_feedback_response.cmd_id = (bz_message.cmd0 << 8) | bz_message.cmd1;
    command_feedback_response.sender_sysid = bz_message.sender_sysid;
    command_feedback_response.reciver_sysid = bz_message.receiver_sysid;
    command_feedback_response.reciver_time = feedback_data.startup_time;
    command_feedback_response.cmd_status = cmd_status;


    uav_command_feedback(&message, bz_message.sender_sysid, bz_message.receiver_sysid, command_feedback_response);
    ground_down_t_to_qbyte(buff, 
                           &len,
                           &message);
    for (int i = 0; i < 3; i++) {
        port->write_bz_message(buff, len);
    }

    
}

void ProtocolConversion::invalid_teleconrol_cmd(bz_message_uav_up_t bz_message)
{
    printf("***************************************invalid_cmd**************************************\n");
    printf("cmd: 0x%04x\n", bz_message.cmd0 << 8 | bz_message.cmd1);
}

void ProtocolConversion::init_px4()
{
    struct Modes2Name {
        uint8_t     main_mode;
        uint8_t     sub_mode;
        bool        canBeSet;   ///< true: Vehicle can be set to this flight mode
        bool        fixedWing;  /// fixed wing compatible
        bool        multiRotor;  /// multi rotor compatible
    };

    static const struct Modes2Name rgModes2Name[] = {
        //main_mode                         sub_mode                                canBeSet  FW      MC
        { PX4_CUSTOM_MAIN_MODE_MANUAL,      0,                                      true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_STABILIZED,  0,                                      true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_ACRO,        0,                                      true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_RATTITUDE,   0,                                      true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_ALTCTL,      0,                                      true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_OFFBOARD,    0,                                      true,   false,  true },
        { PX4_CUSTOM_MAIN_MODE_SIMPLE,      0,                                      false,  false,  true },
        { PX4_CUSTOM_MAIN_MODE_POSCTL,      PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL,      true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_POSCTL,      PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT,       false,  false,   false },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_LOITER,        true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_MISSION,       true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_RTL,           true,   true,   true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET, true,   false,  true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_LAND,          false,  true,   true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND,      true,  false,  true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_READY,         false,  true,   true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_RTGS,          false,  true,   true },
        { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,       false,  true,   true },
    };

    // Must be in same order as above structure
    const char * rgModeNames[] = {
        "Manual",
        "Stabilized",
        "Acro",
        "Rattitude",
        "Altitude",
        "Offboard",
        "Simple",
        "Position",
        "Orbit",
        "Hold",
        "Mission",
        "Return",
        "Follow Me",
        "Land",
        "Precision Land",
        "Ready",
        "Return to Groundstation",
        "Takeoff",
    };

    // Convert static information to dynamic list. This allows for plugin override class to manipulate list.
    for (size_t i=0; i<sizeof(rgModes2Name)/sizeof(rgModes2Name[0]); i++) {
        const struct Modes2Name* pModes2Name = &rgModes2Name[i];

        FlightModeInfo_t info;

        info.main_mode =    pModes2Name->main_mode;
        info.sub_mode =     pModes2Name->sub_mode;
        info.name =         rgModeNames[i];
        info.canBeSet =     pModes2Name->canBeSet;
        info.fixedWing =    pModes2Name->fixedWing;
        info.multiRotor =   pModes2Name->multiRotor;

        _flightModeInfoList.push_back(info);
    }
}

string ProtocolConversion::flightMode(uint8_t base_mode, uint32_t custom_mode)
{
    string flightMode = "Unknown";
    char c[100];

    if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        union px4_custom_mode px4_mode;
        px4_mode.data = custom_mode;

        bool found = false;
        for (auto info = _flightModeInfoList.begin(); info != _flightModeInfoList.end(); ++info){
            if (info->main_mode == px4_mode.main_mode && info->sub_mode == px4_mode.sub_mode) {
                flightMode = info->name;
                found = true;
                break;
            }
        }

        if (!found) {
            printf("Unknown flight mode: %d, %d\n", base_mode, custom_mode);
            return flightMode;
        }
    }
    return flightMode;
}

bool ProtocolConversion::setFlightMode(string &flightMode, uint8_t* base_mode, uint32_t* custom_mode)
{
    *base_mode = 0;
    *custom_mode = 0;

    bool found = false;
    for (auto info = _flightModeInfoList.begin(); info != _flightModeInfoList.end(); ++info) {
        if (flightMode.compare(info->name) == 0) {
            union px4_custom_mode px4_mode;

            px4_mode.data = 0;
            px4_mode.main_mode = info->main_mode;
            px4_mode.sub_mode = info->sub_mode;

            *base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            *custom_mode = px4_mode.data;

            found = true;
            break;
        }
    }

    if (!found) {
       cout << "Unknown flight Mode: " << flightMode << endl;
    }

    return found;
}


uint8_t ProtocolConversion::cov_flight_status(string flightMode)
{
    uint8_t flight_mode;

    if (flightMode == "Takeoff") {
        flight_mode = TAKEOFF;
    } else if (flightMode == "Land") {
        flight_mode = LAND;
    } else if (flightMode == "Hold") {
        flight_mode = HOLD;
    } else if (flightMode == "Return") {
        flight_mode = RETURN;
    } else if (flightMode == "Mission") {
        flight_mode = ROUTE_READY;
    } else if (flightMode == "") {
        flight_mode = ROUTE_FLIGHT;
    } else if (flightMode == "") {
        flight_mode = AUTO_CRUISE;
    } else if (flightMode == "Manual") {
        flight_mode = VIRTUAL_JOYSTICK_FLIGHT;
    } else if (flightMode == "") {
        flight_mode = AUTO_RETURN;
    }else if (flightMode == "Position" || flightMode == "Stabilized") {
        flight_mode = AUTO_MAUAL_FLIGHT;
    } else if (flightMode == "") {
        flight_mode = EMERENCY_MODE;
    } else if (flightMode == "") {
        flight_mode = STOP_SLURRY;
    } else {
        // cout << "unkown flight mode " << flightMode << endl;
        flight_mode = 0xff;
    }
    return flight_mode;
}

void ProtocolConversion::bz_finalize_message_encode(bz_message_ground_down_t *msg, uint8_t length, uint8_t sender_sysid, uint8_t receiver_sysid, uint16_t cmd)
{
    static uint8_t count = 0;
    uint16_t crc;


    msg->frame_header1 = 0xA5;
    msg->frame_header2 = 0x5A;
    msg->frame_length = length;
    msg->system_type = 0x1;
    msg->seq = count++;
    msg->sender_sysid = sender_sysid;
    msg->receiver_sysid = receiver_sysid;
    msg->cmd0 = (cmd >> 8) & 0xff;
    msg->cmd1 = cmd & 0xff;

    crc = GetCRC16((uint8_t*)&msg->frame_header1, length - 3);
    msg->ck0 = crc & 0x00ff;
    msg->ck1 = crc >> 0x8;
    msg->frame_tail = 0xAA;
}

void ProtocolConversion::uav_platform_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, drone_platform_status_feedback_data_t feedback_data)
{
    memcpy((uint8_t*)&msg->pyload[0], (uint8_t*)&feedback_data.flight_control_version, 63-9+1);

    bz_finalize_message_encode(msg, BZ_GROUND_DOWNSTREAM_LEN, sender_sysid, receiver_sysid, DRONE_PLATFORM_STATUS_FEEDBACK_DATA);

    return;
}

void ProtocolConversion::ground_down_t_to_qbyte(char *buff, unsigned *len, bz_message_ground_down_t *msg)
{

    memcpy(buff, (char*)&msg->frame_header1, BZ_GROUND_DOWNSTREAM_LEN);
    *len = BZ_GROUND_DOWNSTREAM_LEN;
}

void ProtocolConversion::print_mavlink(mavlink_message_t message)
{
    printf("message.magic: 0x%02x\n", message.magic);
    printf("message.len: 0x%02x\n", message.len);
    printf("message.magic: 0x%02x\n", message.incompat_flags);
    printf("message.incompat_flags: 0x%02x\n", message.compat_flags);
    printf("message.seq: 0x%02x\n", message.seq);
    printf("message.sysid: 0x%02x\n", message.sysid);
    printf("message.compid: 0x%02x\n", message.compid);
    printf("message.msgid: %d\n", message.msgid);
    printf("message.checksum: 0x%04x\n", message.checksum);
}

void ProtocolConversion::set_flight_mode(bz_message_uav_up_t bz_message, string flight_mode)
{
    mavlink_message_t message;

    uint8_t     base_mode;
    uint32_t    custom_mode;

    if (setFlightMode(flight_mode, &base_mode, &custom_mode)) {
        uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;
        newBaseMode |= base_mode;

        mavlink_msg_set_mode_pack_chan(bz_message.sender_sysid,
                                    0,
                                    0,
                                    &message,
                                    bz_message.receiver_sysid,
                                    newBaseMode,
                                    custom_mode);
        dest_port->write_message(message);
    } else {
        cout << "FirmwarePlugin::setFlightMode failed, flightMode:" << flight_mode << endl;
    }
}

void ProtocolConversion::guidedModeTakeoff(bz_message_uav_up_t bz_message, string flight_mode)
{

}

void ProtocolConversion::sendMavCommand(int compId, MAV_CMD command, bool showError, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{

}

void ProtocolConversion::uav_command_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, command_feedback_response_t feedback_data)
{
    memcpy((uint8_t*)&msg->pyload[0], (uint8_t*)&feedback_data.reciver_status, 63-9+1);

    bz_finalize_message_encode(msg, BZ_GROUND_DOWNSTREAM_LEN, sender_sysid, receiver_sysid, DRONE_PLATFORM_STATUS_FEEDBACK_DATA);

    return;
}

int ProtocolConversion::arm_disarm( bool flag )
{
    uint8_t component_id = 0;
    uint8_t target_compnent = 0;
	if(flag)
	{
		printf("ARM ROTORS\n");
	}
	else
	{
		printf("DISARM ROTORS\n");
	}

	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = receiver_sysid;
	com.target_component = target_compnent;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = (float) flag;
	com.param2           = 21196;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(sender_sysid, component_id, &message, &com);

	// Send the message
	int len = dest_port->write_message(message);

	// Done!
	return len;
}