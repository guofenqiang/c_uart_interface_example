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

    void autonomous_flight_and_steering(bz_message_uav_up_t bz_message);
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

    string flightMode(uint8_t base_mode, uint32_t custom_mode);
    bool setFlightMode(string &flightMode, uint8_t* base_mode, uint32_t* custom_mode);
    void init_px4();

    uint8_t cov_flight_status(string flightMode);

    uint8_t _base_mode;
	uint32_t _custom_mode;

    void bz_finalize_message_encode(bz_message_ground_down_t *msg, uint8_t length, uint8_t sender_sysid, uint8_t receiver_sysid, uint16_t cmd);
    void uav_platform_feedback(bz_message_ground_down_t *msg, uint8_t sender_sysid, uint8_t receiver_sysid, drone_platform_status_feedback_data_t feedback_data);
    void ground_down_t_to_qbyte(char *buff, unsigned *len, bz_message_ground_down_t *msg);

    void print_mavlink(mavlink_message_t message);

    //需要发送的数据
    char send_buff[300];
	unsigned send_len;

private:


};


#endif // PROTOCOL_CONVERSION_H