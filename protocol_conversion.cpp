#include "protocol_conversion.h"
#include "BZConv.h"
#include "CRCCheck.h"
#include <iostream>


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
    uint8_t UAVBuffer[BZ_UAV_UPSTREAM_LEN];

    BZ_Ground2UAV(UAVBuffer, (uint8_t *)src);
    memcpy((uint8_t*)dest, UAVBuffer, BZ_UAV_UPSTREAM_LEN);

    return;
}

void ProtocolConversion::bz_uav_to_ground()
{

}

void ProtocolConversion::bz_telecontrol_decode(char *buff, int len)
{
    uint8_t uav_buff[BZ_UAV_UPSTREAM_LEN];
    bz_message_uav_up_t bz_message = {};

    bz_ground_to_uav(uav_buff, (uint8_t *)buff);
    memcpy((uint8_t *)&bz_message.frame_header1, (uint8_t *)&uav_buff[0], sizeof(bz_message));

    switch (bz_message.cmd0 << 8 | bz_message.cmd1) {
        case AUTONOMOUS_FLIGHT_AND_STEERING:
            autonomous_flight_and_steering(bz_message);
            break;
        default :
            invalid_teleconrol_cmd(bz_message);
            break; 
        
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

    drone_platform_status_feedback_data_t feedback_data = {0};

    // Handle Message ID
    switch (message.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            _base_mode = heartbeat.base_mode;
            _custom_mode = heartbeat.custom_mode;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
            uint8_t sender_sysid = message.sysid;
            uint8_t receiver_sysid = 10;

            mavlink_msg_attitude_decode(&message, &attitude);
            feedback_data.roll_angle = attitude.roll * 57.3 * 100;
            feedback_data.pitch_angle = attitude.pitch * 57.3 * 100;
            if (attitude.yaw >= 0) {
                feedback_data.yaw_angle = attitude.yaw * 57.3 * 10;
            } else {
                feedback_data.yaw_angle = (360 + attitude.yaw * 57.3) * 10;
            }

            feedback_data.platform_status = cov_flight_status(flightMode(_base_mode, _custom_mode));
            uav_platform_feedback(&bz_message, sender_sysid, receiver_sysid, feedback_data);
            ground_down_t_to_qbyte(send_buff, &send_len, &bz_message);
            // printf("roll: %f, pitch: %f, yaw: %f, roll1: %d, pitch1: %d, yaw1: %d\n", 
            // attitude.roll, attitude.pitch, attitude.yaw, feedback_data.roll_angle, feedback_data.pitch_angle, feedback_data.yaw_angle);
        }

        // case MAVLINK_MSG_ID_ALTITUDE:
        // {
        //     mavlink_altitude_t altitude;
        //     mavlink_msg_altitude_decode(&message, &altitude);

        //     feedback_data.relative_height = (int16_t)altitude.altitude_relative * 10;
        // }

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


void ProtocolConversion::autonomous_flight_and_steering(bz_message_uav_up_t bz_message)
{
    mavlink_message_t message;

    uint8_t     base_mode;
    uint32_t    custom_mode;
    string flight_mode = "Stabilized";

    if (setFlightMode(flight_mode, &base_mode, &custom_mode)) {
        uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;
        newBaseMode |= base_mode;

        printf("***************************************autonomous_flight_and_steering**************************************\n");

        mavlink_msg_set_mode_pack_chan(bz_message.sender_sysid,
                                    0,
                                    0,
                                    &message,
                                    bz_message.receiver_sysid,
                                    newBaseMode,
                                    custom_mode);
        dest_port->write_message(message);
        printf("newBaseMode: 0x%02x\n", newBaseMode);
        printf("custom_mode: 0x%08x\n", custom_mode);
        print_mavlink(message);
    } else {
        cout << "FirmwarePlugin::setFlightMode failed, flightMode:" << flight_mode << endl;
    }
}

void ProtocolConversion::command_feedback_response(bz_message_uav_up_t bz_message)
{
    command_feedback_response_t command_feedback_response;
    
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