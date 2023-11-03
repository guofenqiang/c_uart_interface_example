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
    uint8_t pyload[BZ_GROUND_UPSTREAM_LEN - 12];
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

enum {
    SINGLE_ROUTE = 0x00,
    TAKEOFF_ROUTE = 0xD0,
    LAND_ROUTE,
    CRUISE_ROUTE,
    LEVEL_ROUTE,
    HOMEWARD_ROUTE,
    EMERGENCY_ROUTE,
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

typedef struct __photoelectric_load_work_mode {
    uint8_t enable;
} photoelectric_load_work_mode_t;

typedef struct __photoelectric_load_dual_light_switching {
    uint8_t light_mode; /* 0x01: visible light, 0x02: infrared, 0x03: Picture in Picture */
} photoelectric_load_dual_light_switching_t;

typedef struct __photoelectric_load_image_stabilizing {
    uint8_t enbale;
} photoelectric_load_image_stabilizing_t;

typedef struct __photoelectric_load_zoom {
    uint8_t dir; /* 0x01: add, 0x02: reduce */
} photoelectric_load_zoom_t;

typedef struct __photoelectric_load_laser_irradiation {
    uint8_t enable;
} photoelectric_load_laser_irradiation_t;

typedef struct __photoelectric_load_take_photos {
    uint8_t mode; /* 0x01: singale take photos, 0x02: continuous take photos*/
} _photoelectric_load_take_photos_t;

typedef struct __photoelectric_load_center {
    uint8_t enable;
} photoelectric_load_center_t;

typedef struct __photoelectric_load_video {
    uint8_t enable;
} photoelectric_load_video_t;

typedef struct __photoelectric_load_target_detection_and_recognition {
    uint8_t enable;
} photoelectric_load_target_detection_and_recognition_t;

typedef struct __photoelectric_load_automatic_target_tracking {
    uint8_t enable;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} photoelectric_load_automatic_target_tracking_t;

typedef struct __photoelectric_load_pan_tilt_control_mode{
    uint8_t mode; /* 0x01: angle, 0x02: velocity, 0x03: scan, 0x04: stop*/
    int16_t angle;
    int16_t dir;
    int16_t pitch;
    int16_t yaw;
    uint16_t scan_angle_velocity;
} photoelectric_load_pan_tilt_control_mode_t;

typedef struct __optoelectronic_load_status_query {
    uint8_t status; /* 0x01: image, 0x02: vido, 0x03: laser ranging, 0x04: laser irradiation*/
} optoelectronic_load_status_query_t;

typedef struct __GJ_work_mode {
    uint8_t enable;
} GJ_work_mode_t;

typedef struct __GJ_attack_mode {
    uint8_t mode; /* 0x01: 抛投, 0x02: 射流, 0x03: 定时引爆*/
} attack_mode_t;

typedef struct __GJ_insurance_mode {
    uint8_t enable;
} GJ_insurance_mode_t;

typedef struct __GJ_attack_status {
    uint8_t enable;
} GJ_attack_status_t;

typedef struct __GJ_automatic_target_tracking {
    uint8_t enable;
    uint16_t left_pixel_x;
    uint16_t left_pixel_y;
    uint16_t target_width;
    uint16_t target_height;
} GJ_automatic_target_tracking_t;

typedef struct __GJ_autonomous_strike {
    uint8_t enable;
    uint16_t left_pixel_x;
    uint16_t left_pixel_y;
    uint16_t target_width;
    uint16_t target_height;
} GJ_autonomous_strike_t;

typedef struct __GJ_load_status_query {
    uint8_t mode; /* 0x01: DJ载荷, 0x02: zdb */
} GJ_load_status_query_t;

typedef struct __GJ_ZDB_control {
    uint8_t reserved[2]; /* reserved */
    uint8_t mode; /* 0x01: 解保I, 0x02: 解保II, 0x03: 发火, 0x04: 检弹, 0xFF: 状态查询*/
} GJ_ZDB_control_t;

typedef struct __manual_mode {
    uint8_t enable;
    int16_t forward_speed;
    int16_t level_speed;
    int16_t vtol_speed;
    int16_t uav_head_angle;
} manual_mode_t;

typedef struct __autonomous_takeoff {
    uint8_t enable;
    int16_t height;
} autonomous_takeoff_t;

typedef struct __autonomous_return {
    uint8_t enable;
} autonomous_return_t;

typedef struct __autonomous_cruise {
    uint8_t enable;
} autonomous_cruise_t;

typedef struct __autonomous_flight {
    uint8_t act; /* 0x01: 前向, 0x02: 后向, 0x03: 左向, 0x04: 右向, 0x05: 升高, 0x06: 降低, 0x07: 左旋, 0x08: 右旋, 0xff: 关闭*/
    uint16_t virt_joy;
} autonomous_flight_t;

typedef struct __autonomous_obstacle_avoidance {
    uint8_t enable;
} autonomous_obstacle_avoidance_t;

typedef struct __route_inquiry {
    uint8_t route_number;
} route_inquiry_t;

typedef struct  __speed_setting {
    uint8_t mode; /* 0X01-巡航速度设置，0X02-最大速度设置 0X03-平飞速度设置 */
    uint16_t speed; /* 范围：0~60m/s 单位：0.1 m/s  ×10 上传*/
} speed_setting_t;

typedef struct __route_setting {
    uint8_t upload;
    int32_t longitude;
    int32_t latitude;
    int16_t relative_height;
    uint8_t waypoint_numbers;
    uint8_t waypoint_id;
    uint8_t route_number;
    uint8_t route_type;
    uint8_t waypoint_type;
    uint16_t stay_time;
    uint16_t relative_ground_speed;
} route_setting_t;

typedef struct __differential_data_setting {
    uint8_t enable;
    uint8_t data[51];
} differential_data_setting_t;

typedef struct __route_flight {
    uint8_t route_switch; /*0x00: signal route on, 0xff: route off, 0x01-0x0fe: other route */
    uint8_t start_mode; /* 0: restart, 1: last history, 2: designated_waypoint*/
    uint8_t start_id; /* start_id is valid in start_mode 2*/
} route_flight_t;

typedef struct __geographic_coordinage_guidance {
    uint8_t enable;
    int32_t longitude;
    int32_t latitude;
    int16_t altitude;
    int16_t angle;
} geographic_coordinage_guidance_t;

typedef struct __route_download {
    uint8_t route_number;
} route_download_t;

typedef struct __autonomous_navigation_positioning_setting {
    uint8_t enable;
    int32_t ref_longitude;
    int32_t ref_latitude;
    int16_t ref_altitude;
    int16_t ref_angle;
} autonomous_navigation_positioning_setting_t;

typedef struct __autonomous_land {
    uint8_t enable;
} autonomous_land_t;

typedef struct __formation_flight {
    uint8_t enable;
} formation_flight_t;

typedef struct __formation_changes {
    uint8_t mode; /* 0X00-维持原队形，0X01-一字型队形，0X02-人字形队形，0X03-圆形队形*/
} formation_changes_t;

typedef struct __formation_takeoff {
    uint8_t enable;
} formation_takeoff_t;

typedef struct __formation_land {
    uint8_t enable;
} formation_land_t;

typedef struct __formation_leader_set {
    uint8_t enable;
    uint8_t uav_id;
    int32_t target_longitude;
    int32_t target_latitude;
    int16_t target_relative_altitude;
} formation_leader_set_t;

typedef struct __image_status_feedback_data {
    uint8_t work_mode; /* 0X01-可见光图像，0X02-红外图像 */
    uint8_t photgraph_mode; /* 0X00-关闭，0X01-单拍， 0X02-连拍*/
    uint8_t ppi; /*0X01-1080P，0X02-720P，0X03-640×512，0X04-320×256*/
} image_status_feedback_data_t;

typedef struct __video_status_feedback{
    uint8_t work_mode; /* 0X01-可见光图像，0X02-红外图像，0X03-画中画*/
    uint8_t ppi; /* 0X01-1080P，0X02-720P，0X03-640×512，0X04-320×256*/
    uint8_t video_bitrate; /* 单位：0.1M，x10上传*/
    uint8_t video_frame_rate; /*单位：0.5HZ，x2上传*/
} video_status_feedback;

typedef struct __laser_ranging_feedback{
    uint8_t work_mode;
    int16_t range_distance; /*单位 0.1米,负数表示测距无效，x10上传*/
    uint16_t left_pixel_x;
    uint16_t left_pixel_y;
    uint16_t width;
    uint16_t height;
    int32_t longitude;
    int32_t latitude;
    int16_t altitude;
} laser_ranging_feedback;

typedef struct __laseer_irradiation_feedback{
    uint8_t status;
    uint16_t end_time;
    uint16_t start_time;
} laseer_irradiation_feedback;

typedef struct __attack_payload_feedback_data{
    uint8_t status;
    uint8_t attack_mode; /*0X01-抛射，0X02-射流，0X03-定时引爆（zdb）*/
    uint8_t insure_mode;
    uint8_t GJ_status;
    uint8_t self_test;
} attack_payload_feedback_data;

typedef struct __zdb_feedback_data {
    uint8_t status;
    uint8_t attack_mode;
    uint8_t insure_mode;
    uint8_t GJ_status;
    uint8_t self_test;
} zdb_feedback_data_t;

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

/*Zdb状态反馈指令 0x56*/

typedef struct __task_status_feedback_data {
    uint8_t work_mode; /* 0X01-视频，0X02-目标检测识别，0X03-自主跟踪*/
    uint8_t target_numbers;
    int16_t laser_ranging_returns_distance_information; /*单位 0.1米,负数表示测距无效，x10上传*/
    uint16_t countdown_of_laser_cooling; /*0:无计时;  >0 冷却倒计时中,单位秒*/
    uint16_t laser_on_time; /*单位秒*/
    uint8_t target_properties; /*0X01-人员，0X02-轿车，0X03-tank，0X04-堡垒*/
    uint16_t left_pixel_x;
    uint16_t left_pixel_y;
    uint16_t width;
    uint16_t height;
    int32_t longitude;
    int32_t latitude;
    int16_t altitude;
} task_status_feedback_data_t;

typedef struct __formation_status_feedback_data {
    uint8_t status;
    uint8_t formation; /*0X01-一字型队形，0X02-人字形队形，0X03-圆形队形*/
    uint16_t leader_angle;
    int32_t leader_longitude;
    int32_t leader_latitude;
    int32_t leader_altitude;
    int32_t leader_ground_speed;
} formation_status_feedback_data_t;

typedef struct __command_feedback_response {
    uint8_t reciver_status;
    uint16_t cmd_id;
    uint8_t sender_sysid;
    uint8_t reciver_sysid;
    uint16_t reciver_time;
    uint8_t cmd_status;
} command_feedback_response_t;

typedef struct __route_inquiry_reply {
    uint8_t route_number;
    int32_t waypoint_longitude;
    int32_t waypoint_latitude;
    int16_t waypoint_altitude;
    uint8_t waypoint_numbers;
    uint8_t waypoint_id;
    uint8_t route_id;
    uint8_t route_type;
    uint8_t waypoint_type;
    uint16_t stay_time;
    uint16_t ground_speed;
} route_inquiry_reply_t;

typedef struct __route_download_reply {
    uint8_t route_number;
    int32_t waypoint_longitude;
    int32_t waypoint_latitude;
    int16_t waypoint_altitude;
    uint8_t waypoint_numbers;
    uint8_t waypoint_id;
    uint8_t route_id;
    uint8_t route_type;
    uint8_t waypoint_type;
    uint16_t stay_time;
    uint16_t ground_speed;
} route_download_reply_t;

typedef struct __route_confirmation_reply {
    uint8_t route_id;
    uint8_t route_type;
    uint8_t waypoint_numbers;
    uint8_t pre_correct_waypoint_id;
    uint8_t waypoint_type;
    uint8_t status;
} route_confirmation_reply_t;

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