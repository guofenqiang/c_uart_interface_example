#ifndef PROTOCOL_TYPE_H
#define PROTOCOL_TYPE_H

#include <stdio.h>
#include <stdlib.h>

/*up: UAV<-Ground*/
#define BZ_UAV_UPSTREAM_LEN 64
#define BZ_GROUND_UPSTREAM_LEN 53

/*down: UAV->Ground*/
#define BZ_UAV_DOWNSTREAM_LEN 128
#define BZ_GROUND_DOWNSTREAM_LEN 115


typedef struct __bz_message_t {
    uint8_t frame_header1;
    uint8_t frame_header2;
    uint8_t frame_length;
    uint8_t system_type;
    uint8_t seq;
    uint8_t sender_sysid;
    uint8_t receiver_sysid;
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t pyload[BZ_UAV_DOWNSTREAM_LEN - 12];
    uint8_t ck0;
    uint8_t ck1;
    uint8_t frame_tail;
} bz_message_t; //bz_message_uav_down_t

typedef struct __bz_message_uav_up_t {
    uint8_t frame_header1;
    uint8_t frame_header2;
    uint8_t frame_length;
    uint8_t system_type;
    uint8_t seq;
    uint8_t sender_sysid;
    uint8_t receiver_sysid;
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t pyload[BZ_UAV_UPSTREAM_LEN - 12];
    uint8_t ck0;
    uint8_t ck1;
    uint8_t frame_tail;
} bz_message_uav_up_t;


typedef struct __bz_message_ground_down_t {
    uint8_t frame_header1;
    uint8_t frame_header2;
    uint8_t frame_length;
    uint8_t system_type;
    uint8_t seq;
    uint8_t sender_sysid;
    uint8_t receiver_sysid;
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t pyload[BZ_GROUND_DOWNSTREAM_LEN - 12];
    uint8_t ck0;
    uint8_t ck1;
    uint8_t frame_tail;
} bz_message_ground_down_t;

typedef struct __bz_message_ground_up_t {
    uint8_t frame_header1;
    uint8_t frame_header2;
    uint8_t frame_length;
    uint8_t system_type;
    uint8_t seq;
    uint8_t sender_sysid;
    uint8_t receiver_sysid;
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t pyload[BZ_GROUND_UPSTREAM_LEN - 12];
    uint8_t ck0;
    uint8_t ck1;
    uint8_t frame_tail;
} bz_message_ground_up_t;

enum {
    /* 遥控命令子定义*/
    WORK_MODE = 0x0001,
    LIGHT_SWITCHING,
    IMAGE_STABILIZATION_INSTRUCTION,
    ZOOM,
    LASER_IRRADIATION,
    PHOTOGRAPHY,
    CENTERING,
    VIDEO,
    TARGET_DETECTION_AND_RECOGNITION,
    AUTOMATIC_TARGET_TRACKING,
    //    GEOGRAPHIC_COORDINATE_GUIDANCE, //该指令弃用
    PAN_TILT_CONTROL_MODE = 0X000C,
    OPTOELECTRONIC_LOAD_STATUS_QUERY,

    GJ_WORK_MODE = 0x0021,
    ATTACK_MODE_SWITCHING,
    INSURANCE_MODE,
    ATTACK_STATUS,
    GJ_AUTOMATIC_TARGET_TRACKING,
    AUTONOMOUS_STRIKE,
    GJ_LOAD_STATUS_QUERY,
    ZDB_CONTROL = 0x0055,

    MANUAL_MODE = 0x00A1,
    AUTONOMOUS_TAKEOFF,
    AUTONOMOUS_RETURN,
    AUTONOMOUS_CRUISE,
    AUTONOMOUS_FLIGHT_AND_STEERING,
    AUTONOMOUS_OBSTACLE_AVOIDANCE,
    ROUTE_INQUIRY,
    SPEED_SETTING,
    ROUTE_SETTING,
    DIFFERENTIAL_DATA_SETTING,
    ROUTE_FLIGHT_INSTRUCTIONS,
    GEOGRAPHIC_COORDINATE_GUIDANCE,
    ROUTE_DOWNLOAD_SWITCH,
    AUTONOMOUS_NAVIGATION_POSITIONING_SETTING,
    AUTONOMOUS_PRECISION_LAND,

    FORMATION_FLIGHT = 0x00D1,
    FORMATION_FORMATION_TRANSFORMATION,
    ONE_CLICK_TAKEOFF_COMMAND_FOR_FORMATION,
    ONE_CLICK_RETURN_TO_LANDING_COMMOND_FOR_FORMATION,
    NAVIGATOR_WAYPOING_SETTING,

    /* 遥测命令子定义*/
    IMAGE_STATUS_FEEDBACK_DATA = 0x1001,
    VIDEO_STATUS_FEEDBACK,
    LASER_RANGING_FEEDBACK,
    LASEER_IRRADIATION_FEEDBACK,

    ATTACK_PAYLOAD_FEEDBACK_DATA = 0x1021,
    ZDB_FEEDBACK_DATA,

    DRONE_PLATFORM_STATUS_FEEDBACK_DATA = 0x1031,
    TASK_STATUS_FEEDBACK_DATA,
    FORMATION_STATUS_FEEDBACK_DATA,

    COMMAND_FEEDBACK_RESPONSE = 0x1051, //每收到一次遥控指令都需发送一次指令反馈回复（0X1051）

    ROUTE_INQUIRY_REPLY = 0x1052,
    ROUTE_DOWNLOAD_REPLY,
    ROUTE_CONFIRMATION_REPLY,
};



enum PX4_CUSTOM_MAIN_MODE {
	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	PX4_CUSTOM_MAIN_MODE_ALTCTL,
	PX4_CUSTOM_MAIN_MODE_POSCTL,
	PX4_CUSTOM_MAIN_MODE_AUTO,
	PX4_CUSTOM_MAIN_MODE_ACRO,
	PX4_CUSTOM_MAIN_MODE_OFFBOARD,
	PX4_CUSTOM_MAIN_MODE_STABILIZED,
	PX4_CUSTOM_MAIN_MODE_RATTITUDE,
	PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
	PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
	PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
};

enum PX4_CUSTOM_SUB_MODE_POSCTL {
    PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL = 0,
    PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT
};

union px4_custom_mode {
	struct {
		uint16_t reserved;
		uint8_t main_mode;
		uint8_t sub_mode;
	};
	uint32_t data;
	float data_float;
	struct {
		uint16_t reserved_hl;
		uint16_t custom_mode_hl;
	};
};


#pragma pack(push)
#pragma pack(1)

typedef struct __drone_platform_status_feedback_data {
    uint8_t flight_control_version;
    uint8_t hw_version;
    int32_t longitude;
    int32_t latitude;
    int16_t relative_height;
    int16_t yaw_angle; //偏航角
    int16_t pitch_angle; //俯仰角
    int16_t roll_angle; //横滚角
    int16_t northbound_velocity;
    int16_t westbound_velocity;
    int16_t vertical_velocity;
    uint16_t ground_speed;
    uint16_t battery_voltage;
    uint8_t percentage_of_remaining_battery_power;
    uint8_t flight_status;
    uint8_t satellite_positioning_mode;
    uint8_t loss_satellite_time;
    uint16_t airline_bus;
    uint8_t	total_number_of_routes;
    uint8_t	current_route_number;
    uint8_t	total_number_of_waypoints;
    uint8_t	current_waypoint_number;
    int32_t	longitude_of_the_next_waypoint;
    int32_t	latitude_of_the_next_waypoint;
    int16_t	relative_altitude_of_the_next_waypoint;
    uint8_t	autonomous_obstacle_avoidance_state;
    uint8_t	platform_status;
    uint16_t flight_time;
    uint16_t startup_time;
    uint8_t	flight_confirmation;
} drone_platform_status_feedback_data_t;

typedef struct __command_feedback_response {
    uint8_t reciver_status;
    uint16_t cmd_id;
    uint8_t sender_sysid;
    uint8_t reciver_sysid;
    uint16_t reciver_time;
    uint8_t cmd_status;
} command_feedback_response_t;

#pragma pack(pop)

enum {
    TAKEOFF = 0x01,
    LAND,
    HOLD,
    RETURN,
    EMERENCY_HOVER,
    ROUTE_READY,
    WAYPOINT_FLIGHT, //当前没有航点飞行
    ROUTE_FLIGHT,
    AUTO_CRUISE,
    VIRTUAL_JOYSTICK_FLIGHT,
    AUTO_RETURN,
    AUTO_MAUAL_FLIGHT,
    EMERENCY_MODE,
    STOP_SLURRY,
};


#endif // PROTOCOL_TYPE_H