#include "AP_Periph.h"
#include <AP_Param/AP_Param.h>

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
    void parse_time_ubx();

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
    AP_Float _s_in_time;
    AP_Float _s_in_acc;
    AP_Float _s_in_lat;
    AP_Float _s_in_lon;
    AP_Float _s_in_alt;

    int ubx_log_fd = -1;
    char _ubx_log_filename[48];
    bool connected_to_gcs;

    AP_HAL::UARTDriver* gps_uart;
    AP_HAL::UARTDriver* gcs_uart;
};

#endif
