#include "AP_Periph.h"
#include <AP_Param/AP_Param.h>
#include <AP_GPS/RTCM3_Parser.h>
#include <AP_GPS/AP_GPS_UBLOX_CFGv2.h>

#ifdef ENABLE_BASE_MODE
struct date_time {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t utc_sec;
};
class GPS_Base {
public:
    GPS_Base();
    void update();
    void handle_param_set(const mavlink_message_t &msg);
    void handle_param_request_list(const mavlink_message_t &msg);
    void handle_param_request_read(const mavlink_message_t &msg);
    bool enabled() const { return _enabled; } 

    static const struct AP_Param::GroupInfo var_info[];
    static void gps_week_time(struct date_time &dt, const uint16_t week, const uint32_t tow);
    void prepare_ubx_base_cfg();
private:
    void parse_runtime_ubx(uint8_t byte);
    void update_leds();

    void handle_ubx_msg();


    bool parse_ubx(uint8_t c);
    uint8_t _class;
    uint8_t _msg_id;
    uint8_t _step;
    uint8_t _ck_b;
    uint8_t _ck_a;
    uint16_t _payload_length;
    uint16_t _payload_counter;

    struct PACKED ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };

    struct PACKED ubx_nav_svin {
        uint8_t version;
        uint8_t reserved0[3];
        uint32_t iTOW;
        uint32_t dur;
        int32_t meanX;
        int32_t meanY;
        int32_t meanZ;
        int8_t meanXHP;
        int8_t meanYHP;
        int8_t meanZHP;
        uint8_t reserved1;
        uint32_t meanAcc;
        uint32_t obs;
        uint8_t valid;
        uint8_t active;
        uint8_t reserved2[2];
    };

    struct PACKED ubx_raw_rawx {
        double rcvTow;
        uint16_t week;
        int8_t leapS;
        uint8_t numMeas;
        uint8_t recStat;
        uint8_t version;
        uint8_t reserved1[2];
        // per sat
        uint8_t data[32*40];
    };

    union {
        DEFINE_BYTE_ARRAY_METHODS
        ubx_nav_svin nav_svin;
        ubx_raw_rawx raw_rawx;
    } _buffer;

    struct ubx_nav_svin curr_svin;
    uint32_t _start_mean_acc;

    ByteBuffer gps_buffer{256};
    ByteBuffer gcs_buffer{256};

    int ubx_file = -1;

    struct date_time dt;
    bool gps_received_preamble;
    bool gcs_received_preamble;
    uint8_t gps_length_counter;
    uint8_t gcs_length_counter;

    uint16_t gps_num_bytes_to_rx;
    uint16_t gcs_num_bytes_to_rx;

    AP_Int8 _enabled;
    AP_Int8 _logging;
    AP_Int8 _s_in_enabled;
    AP_Float _s_in_time;
    AP_Float _s_in_acc;
    AP_Float _s_in_lat;
    AP_Float _s_in_lon;
    AP_Float _s_in_alt;

    int ubx_log_fd = -1;
    char _ubx_log_filename[48];
    bool connected_to_gcs;
    bool _ppk_config_finished;

    AP_HAL::UARTDriver* gps_uart;
    AP_HAL::UARTDriver* gcs_uart;
    RTCM3_Parser rtcm3_parser;

    uint8_t ppk_config_data[200];
    AP_GPS_UBLOX_CFGv2::UBXPackedCfg ppk_config{ppk_config_data, sizeof(ppk_config_data)};
};

#endif
