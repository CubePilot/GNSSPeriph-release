/*
 * Copyright (c) 2022 Siddharth B Purohit, CubePilot Pty Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

// system includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <fcntl.h>

// include the canard C++ APIs
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <canard/interface.h>

// include the base canard API
#include <canard.h>
#include <canard_internals.h>
#include <dronecan_msgs.h>
#include <rtklib.h>

DEFINE_HANDLER_LIST_HEADS();

DEFINE_TRANSFER_OBJECT_HEADS();

class CanardInterface : public Canard::Interface {
public:
    CanardInterface(uint8_t iface_index) :
        Interface(iface_index) {}
    
    void init(const char *file_name);

    // implement required interface functions
    bool broadcast(const Canard::Transfer &bcast_transfer) override;
    bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;
    bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

    void run();

    uint8_t get_node_id() const override { return canard.node_id; }
    void set_node_id(uint8_t node_id) {
        canardSetLocalNodeID(&canard, node_id);
    }


    static void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
        CanardInterface* iface = (CanardInterface*) ins->user_reference;
        iface->handle_message(*transfer);
    }

    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                            uint64_t* out_data_type_signature,
                                            uint16_t data_type_id,
                                            CanardTransferType transfer_type,
                                            uint8_t source_node_id) {
        CanardInterface* iface = (CanardInterface*) ins->user_reference;
        return iface->accept_message(data_type_id, *out_data_type_signature);
    }
private:
    uint8_t memory_pool[2048];
    CanardInstance canard;
    CanardTxTransfer tx_transfer;
    FILE* fd;

    struct __attribute__((__packed__)) FrameHeader {
        uint16_t magic;
        uint64_t timestamp;
        uint16_t crc;
        uint16_t length;
        uint16_t flags;
        uint32_t message_id;
    } frame_header;
};

void CanardInterface::init(const char *file_name) {
    // initialize the canard library
    canardInit(&canard, memory_pool, sizeof(memory_pool), onTransferReception, shouldAcceptTransfer, this);
    // set canard node id
    canardSetLocalNodeID(&canard, 10);
    // open the file
    fd = fopen(file_name, "rb");
    if (fd == NULL) {
        fprintf(stderr, "Failed to open file %s: %s\n", file_name, strerror(errno));
        exit(1);
    }
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    return true;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    return true;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    return true;
}

// Frame format in binary
//   - MAGIC 0x2934 16 bit
//    - 64 bit monotonic timestamp (microseconds)
//    - 16 bit CRC CRC-16-CCITT (over all bytes after CRC)
//    - 16 bit length
//    - 16 bit flags
//    - 32 bit message ID
//    - data[]
void CanardInterface::run() {
    CanardCANFrame frame;
    while (true) {
        // read the magic
        size_t ret = fread(&frame_header, sizeof(frame_header), 1, fd);
        if (frame_header.magic != 0x2934) {
            fprintf(stderr, "Invalid magic: 0x%04x\n", frame_header.magic);
            exit(1);
        }
        if (frame_header.length > 8) {
            fprintf(stderr, "Invalid length: %d\n", frame_header.length);
            exit(1);
        }
        frame.data_len = frame_header.length;
        frame.id = frame_header.message_id;
        memset(frame.data, 0, sizeof(frame.data));
        // read the data
        ret = fread(frame.data, frame_header.length, 1, fd);
        // check the CRC
        uint16_t crc2 = crcAdd(0xFFFF, frame.data, frame_header.length);
        if (frame_header.crc != crc2) {
            fprintf(stderr, "CRC mismatch: 0x%04x != 0x%04x\n", frame_header.crc, crc2);
            exit(1);
        }
        // send the frame
        int rxret = canardHandleRxFrame(&canard, &frame, frame_header.timestamp);
    }
}

static raw_t ubx_data;
static rtcm_t rtcm_data;
static rtk_t rtk;
static void handle_MovingBaselineData(const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData &msg);
static Canard::StaticCallback<ardupilot_gnss_MovingBaselineData> MovingBaselineData_callback{&handle_MovingBaselineData};
static Canard::Subscriber<ardupilot_gnss_MovingBaselineData> MovingBaselineData_sub{MovingBaselineData_callback, 0};
static void handle_UBXRawData(const CanardRxTransfer& transfer, const ardupilot_gnss_UBXRawData &msg);
static Canard::StaticCallback<ardupilot_gnss_UBXRawData> UBXRawData_callback{&handle_UBXRawData};
static Canard::Subscriber<ardupilot_gnss_UBXRawData> UBXRawData_sub{UBXRawData_callback, 0};
static void handle_GPSFix2(const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Fix2 &msg);
static Canard::StaticCallback<uavcan_equipment_gnss_Fix2> GPSFix2_callback{&handle_GPSFix2};
static Canard::Subscriber<uavcan_equipment_gnss_Fix2> GPSFix2_sub{GPSFix2_callback, 0};


uint64_t curr_time;
gtime_t get_curr_time() {
    gtime_t time;
    time.time = curr_time/1000000; //secs
    time.sec = (((double)curr_time)/1000000) - time.time; // remaining millis
    return time;
}

static void handle_GPSFix2(const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Fix2 &msg) {
    if (msg.gnss_time_standard == UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_UTC) {
        curr_time = msg.gnss_timestamp.usec;
        // printf("Timestamp: %s\n", time_str(get_curr_time(), 3));
    }
}

static void handle_MovingBaselineData(const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData &msg) {
    for (uint16_t i=0; i<msg.data.len; i++) {
        rtcm_data.time = utc2gpst(timeget());
        input_rtcm3(&rtcm_data, msg.data.data[i]);
    }
}

static void handle_UBXRawData(const CanardRxTransfer& transfer, const ardupilot_gnss_UBXRawData &msg) {
    for (uint16_t i=0; i<msg.data.len; i++) {
        if (input_ubx(&ubx_data, msg.data.data[i])) {
            sortobs(&ubx_data.obs);
            sortobs(&rtcm_data.obs);

            for (uint16_t i = 0; i < ubx_data.obs.n; i++) {
                ubx_data.obs.data[i].rcv = 1; //rover
            }
            for (uint8_t i = 0; i < rtcm_data.obs.n; i++) {
                memcpy(&ubx_data.obs.data[ubx_data.obs.n + i], &rtcm_data.obs.data[i], sizeof(obsd_t));
                ubx_data.obs.data[ubx_data.obs.n + i].rcv = 2; //base
            }
            // feed in the data to rtkpos
            rtkpos(&rtk, ubx_data.obs.data, ubx_data.obs.n + rtcm_data.obs.n, &ubx_data.nav);
            // free the data from ubx_data
            for (uint16_t i = 0; i < rtcm_data.obs.n; i++) {
                memset(&ubx_data.obs.data[ubx_data.obs.n + i], 0, sizeof(obsd_t));
            }
            uint64_t rtcm_time_ms = rtcm_data.obs.data[0].time.time*1000 + rtcm_data.obs.data[0].time.sec*1000;
            uint64_t ubx_time_ms = ubx_data.obs.data[0].time.time*1000 + ubx_data.obs.data[0].time.sec*1000;
            printf("RTKLIB: %d [%lld][%d] [%lld][%d]\n", rtk.sol.stat, rtcm_time_ms, rtcm_data.obs.n, ubx_time_ms, ubx_data.obs.n);
            printf("RTKLIB: %d AR: %f/%f\n", rtk.sol.stat, (float)rtk.sol.ratio, rtk.opt.thresar[0]);
            if (rtk.sol.stat) {
                // convert solution to position
                double pos[3], diff_pos[3];
                ecef2pos(rtk.sol.rr, pos);
                // can_printf("RTKLIB Pos: %f %f %f\n", pos[0]*R2D, pos[1]*R2D, pos[2]);
                // print relative position
                // printf("RelPos: %.3f %.3f %.3f\n", rtk.sol.rr[0] - rtk.rb[0], rtk.sol.rr[1] - rtk.rb[1], rtk.sol.rr[2] - rtk.rb[2]);
                // print relative heading
                diff_pos[0] = rtk.sol.rr[0] - rtk.rb[0];
                diff_pos[1] = rtk.sol.rr[1] - rtk.rb[1];
                diff_pos[2] = rtk.sol.rr[2] - rtk.rb[2];
                double relpos[3];
                ecef2enu(pos, diff_pos, relpos);
                double heading = atan2(relpos[1], relpos[0]);
                printf("RelPos: %.3f %.3f %.3f RelHeading: %.3f\n", relpos[0], relpos[1], relpos[2], heading*R2D);
                // can_printf("ROVER Pos: %f %f %f\n", rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]);
                // can_printf("BASE Pos: %f %f %f\n", rtk.rb[0], rtk.rb[1], rtk.rb[2]);
                // print relative heading
                // double heading = atan2(rtk.sol.rr[1] - rtk.rb[1], rtk.sol.rr[0] - rtk.rb[0]);
            }
        }
    }
}


static const prcopt_t prcopt={ /* defaults processing options */
    // PMODE_KINEMA,SOLTYPE_FORWARD, /* mode,soltype */
    PMODE_MOVEB, SOLTYPE_FORWARD, /* mode,soltype */
    1,SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_CMP,  /* nf, navsys */
    15.0*D2R,{{0,0}},           /* elmin,snrmask */
    0,ARMODE_FIXHOLD,ARMODE_CONT,1,ARMODE_CONT,1,                /* sateph,modear,glomodear,gpsmodear,bdsmodear,arfilter */
    100,0,4,5,10,100,             /* maxout,minlock,minfixsats,minholdsats,mindropsats,minfix */
    4,1,1,0,0,                  /* armaxiter,estion,esttrop,dynamics,tidecorr */
    1,0,0,0,0,                  /* niter,codesmooth,intpref,sbascorr,sbassatsel */
    0,0,                        /* rovpos,refpos */
    {300.0, 100.0},        /* eratio[] */
    {100.0,0.003,0.003,0.0,1.0,52.0,0.0,0.0}, /* err[-,base,el,bl,dop,snr_max,snr,rcverr] */
    {30.0,0.03,0.3},            /* std[] */
    {1E-4,1E-3,1E-4,1E-1,1E-2,0.0}, /* prn[] */
    5E-12,                      /* sclkstab */
    {3.0,0.04,0.0,1E-9,1E-5,3.0,3.0,0.0}, /* thresar */
	15.0*D2R,0.0,0.05,0,             /* elmaskar,elmaskhold,thresslip,thresdop, */
	0.1,0.01,0.5,              /* varholdamb,gainholdamb,maxtdif */
    {3000.0,30.0},                 /* maxinno {phase,code} */
    {0},{0},{0},                /* baseline,ru,rb */
    {"",""},                    /* anttype */
    {{0}},{{0}},{0},            /* antdel,pcv,exsats */
    1,1                         /* maxaveep,initrst */
};

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <file>\n", argv[0]);
        return 1;
    }
    char buff[2048];
    int id;

    memset(&rtk, 0, sizeof(rtk_t));
    memset(&ubx_data, 0, sizeof(raw_t));
    memset(&rtcm_data, 0, sizeof(rtcm_t));

    if (init_raw(&ubx_data, STRFMT_UBX) == 0) {
        fprintf(stderr, "UBX init failed\n");
        return 1;
    }
    if (!init_rtcm(&rtcm_data)) {
        fprintf(stderr, "RTCM init failed\n");
        return 1;
    }
    rtkinit(&rtk, &prcopt);

    CanardInterface iface(0);
    iface.init(argv[1]);
    iface.run();
    return 0;
}
