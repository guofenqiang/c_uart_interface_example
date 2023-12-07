#ifndef PROTOCOL_CONVERSION_H
#define PROTOCOL_CONVERSION_H

#include "serial_port.h"
#include "protocol_type.h"
#include <string>
#include <list>
using namespace std;

class ProtocolConversion
{
public:

    ProtocolConversion(Generic_Port *port_);
    ProtocolConversion(Generic_Port *port_, Generic_Port *dest_port_);
    virtual ~ProtocolConversion();

    void bz_ground_to_uav(uint8_t *dest, uint8_t *src);
    void bz_uav_to_ground();
    void bz_telecontrol_decode(char *buff, int len);
    void bz_telemetry_decode(char *buff, int len);
    void handle_message(mavlink_message_t &message);

    void virtual_rocker_mode(bz_message_uav_up_t bz_message);
    void autonomous_takeoff(bz_message_uav_up_t bz_message);
    void autonomous_return(bz_message_uav_up_t bz_message);
    void autonomous_cruise(bz_message_uav_up_t bz_message);
    void autonomous_flight_and_steering(bz_message_uav_up_t bz_message);
    void autonomous_obstacle_avoidance(bz_message_uav_up_t bz_message);
    void route_setting(bz_message_uav_up_t bz_message);
    void route_flight_instructions(bz_message_uav_up_t bz_message);
    void geographic_coordinage_guidance(bz_message_uav_up_t bz_message);
    void route_downlaod_switch(bz_message_uav_up_t bz_message);
    void route_inquiry(bz_message_uav_up_t bz_message);
    void autonomous_precision_land(bz_message_uav_up_t bz_message);
    void route_download_reply();


    void command_feedback_response(bz_message_uav_up_t bz_message);
    void invalid_teleconrol_cmd(bz_message_uav_up_t bz_message);

    Generic_Port *port;
    Generic_Port *dest_port;

    typedef struct {
        uint8_t         main_mode;
        uint8_t         sub_mode;
        const char*  name;       ///< Name for flight mode
        bool            canBeSet;   ///< true: Vehicle can be set to this flight mode
        bool            fixedWing;  /// fixed wing compatible
        bool            multiRotor; /// multi rotor compatible
    } FlightModeInfo_t;

    list<FlightModeInfo_t> _flightModeInfoList;

    list<string> flightModes();
    string flightMode(uint8_t base_mode, uint32_t custom_mode);
    bool setFlightMode(string &flightMode, uint8_t* base_mode, uint32_t* custom_mode);
    void init_px4();

    uint8_t cov_flight_status(string flightMode);

    static uint8_t _base_mode;
	static uint32_t _custom_mode;

    void bz_finalize_message_encode(bz_message_ground_down_t *msg, uint8_t length, uint8_t sender_sysid, uint8_t receiver_sysid, uint16_t cmd);
    void uav_platform_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, drone_platform_status_feedback_data_t feedback_data);
    void ground_down_t_to_qbyte(char *buff, unsigned *len, bz_message_ground_down_t *msg);

    void print_mavlink(mavlink_message_t message);

    //需要发送的数据
    char send_buff[300];
	unsigned send_len;
    static drone_platform_status_feedback_data_t feedback_data;
    static list<route_setting_t> _routeList;
    static int32_t amsl_alt;
    static list<mavlink_mission_item_int_t> mission_item_int_list;
    static uint8_t sender_sysid; // uav id
    static uint8_t receiver_sysid; // bz id
    uint8_t receiver_status;
    uint8_t cmd_status;
    static route_download_t route_download;

    void set_flight_mode(bz_message_uav_up_t bz_message, string flight_mode);
    void guidedModeTakeoff(bz_message_uav_up_t bz_message, string flight_mode);
    void sendMavCommand(int compId, MAV_CMD command, bool showError, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
    void uav_command_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, command_feedback_response_t feedback_data);
    void uav_route_download_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, route_download_reply_t feedback_data);
    void uav_route_inquiry_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, route_inquiry_reply_t feedback_data);

    int arm_disarm( bool flag );
    void startMission(bz_message_uav_up_t bz_message);
    bool setFlightModeAndValidate(bz_message_uav_up_t bz_message, const string& flight_mode);
    bool armVehicleAndValidate();
    bool armed              () const{ return _armed; }
    void updateArmed(bool armed);
    void route_inquiry_reply();
    mavlink_mission_count_t mission_count;

private:
    bool _armed = false;

};


#endif // PROTOCOL_CONVERSION_H