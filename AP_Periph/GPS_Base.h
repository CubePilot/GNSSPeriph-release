#include "AP_Periph.h"
#include <AP_Param/AP_Param.h>
#include <AP_GPS/RTCM3_Parser.h>

#ifdef ENABLE_BASE_MODE

class GPS_Base {
public:
    GPS_Base();
    void update();
    void handle_param_set(const mavlink_message_t &msg);
    void handle_param_request_list(const mavlink_message_t &msg);
    void handle_param_request_read(const mavlink_message_t &msg);

    static const struct AP_Param::GroupInfo var_info[];

private:
    void gps_week_time(const uint16_t week, const uint32_t tow);
    void parse_runtime_ubx(uint8_t byte);
    void _update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b);
    bool _send_message(uint8_t msg_class, uint8_t msg_id, const void *msg, uint16_t size);
    void do_configurations();
    bool configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    bool request_message_rate(uint8_t msg_class, uint8_t msg_id);

    void handle_ubx_msg();


    bool parse_ubx(uint8_t c);
    uint8_t _class;
    uint8_t _msg_id;
    uint8_t _step;
    uint8_t _ck_b;
    uint8_t _ck_a;
    uint16_t _payload_length;
    uint16_t _payload_counter;
    bool _update_setting;
    uint32_t _last_save_config_ms;

    struct PACKED ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };

    struct PACKED ubx_mon_ver {
        char swVersion[30];
        char hwVersion[10];
        char extension[180]; // extensions are not enabled
    };

    struct PACKED ubx_cfg_prt {
        uint8_t portID;
        uint8_t reserved0;
        uint16_t txReady;
        uint32_t mode;
        uint32_t baudRate;
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t flags;
        uint16_t reserved1;
    };
    struct PACKED ubx_cfg_nav_rate {
        uint16_t measure_rate_ms;
        uint16_t nav_rate;
        uint16_t timeref;
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

    struct PACKED ubx_cfg_msg {
        uint8_t msg_class;
        uint8_t msg_id;
    };

    struct PACKED ubx_cfg_msg_rate {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rate;
    };
    struct PACKED ubx_cfg_msg_rate_6 {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rates[6];
    };
    struct PACKED ubx_cfg_cfg {
        uint32_t clearMask;
        uint32_t saveMask;
        uint32_t loadMask;
    };

    struct PACKED ubx_ack_ack {
        uint8_t msg_class;
        uint8_t msg_id;
    };

    struct PACKED ubx_cfg_tmode3 {
        uint8_t version;
        uint8_t reserved0;
        uint16_t flags;
        int32_t ecefXOrLat;
        int32_t ecefYOrLon;
        int32_t ecefZOrAlt;
        int8_t ecefXOrLatHP;
        int8_t ecefYOrLonHP;
        int8_t ecefZOrAltHP;
        uint8_t reserved;
        uint32_t fixedPosAcc;
        uint32_t svinMinDur;
        uint32_t svinAccLimit;
        uint8_t reserved1[8];
    };

    union {
        DEFINE_BYTE_ARRAY_METHODS
        ubx_mon_ver mon_ver;
        ubx_cfg_nav_rate cfg_nav_rate;
        ubx_cfg_msg_rate cfg_msg_rate;
        ubx_ack_ack ack_ack;
        ubx_cfg_prt cfg_prt;
        ubx_cfg_msg_rate_6 cfg_msg_rate_6;
        ubx_nav_svin nav_svin;
        ubx_raw_rawx raw_rawx;
    } _buffer;

    struct ubx_cfg_msg_rate curr_msg;

    enum {
        SETTING_BAUD,
        CHECKING_VERSION,
        WAITING_FOR_VERSION,
        GETTING_PORT_INDEX,
        SETTING_NAV_RATE,
        SETTING_SURVEY_IN_RATE,
        SETTING_PVT_RATE,
        SETTING_1005_RATE,
        SETTING_1074_RATE,
        SETTING_1084_RATE,
        SETTING_1094_RATE,
        SETTING_1124_RATE,
        SETTING_1230_RATE,
        SETTING_RXM_RAWX,
        SETTING_RXM_SFRBX,
        SETTING_SAVE_CONFIG,
        SETTING_SURVEYIN_CONFIG,
        SETTING_FINISHED
    };
    uint8_t ubx_config_state;

    uint32_t _last_config_ms;
    uint8_t _ublox_port;
    uint32_t _last_surveyin_config_ms;

    ByteBuffer gps_buffer{256};
    ByteBuffer gcs_buffer{256};

    int ubx_file = -1;

    struct date_time {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint32_t utc_sec;
    } dt;

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
};

#endif
