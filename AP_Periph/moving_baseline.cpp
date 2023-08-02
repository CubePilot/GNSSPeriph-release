/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

#include <dronecan_msgs.h>
#include <stm32_util.h>
#ifdef ENABLE_RTKLIB

extern const AP_HAL::HAL &hal;

#if 0
#define debug(fmt, ...) can_printf("MB:" fmt "\n", ##__VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

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


gtime_t get_curr_time() {
    gtime_t time;
    time.time = hal.util->get_hw_rtc()/1000000; //secs
    time.sec = (((double)hal.util->get_hw_rtc())/1000000) - time.time; // remaining millis
    return time;
}

void AP_Periph_FW::rtklib_handle_rtcm_fragment(const uint8_t *data, uint16_t len)
{
    if (rtcm_data == nullptr) {
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        rtcm_data->time = utc2gpst(timeget());
        input_rtcm3(rtcm_data, data[i]);
    }
}

void AP_Periph_FW::rtklib_init()
{
    if (rtcm_data == nullptr) {
        rtcm_data = (rtcm_t*)malloc(sizeof(rtcm_t));
        if (rtcm_data == nullptr) {
            return;
        }
    }
    if (ubx_data == nullptr) {
        ubx_data = (raw_t*)malloc(sizeof(raw_t));
        if (ubx_data == nullptr) {
            return;
        }
    }

    if (init_raw(ubx_data, STRFMT_UBX) == 0) {
        return;
    }
    if (!init_rtcm(rtcm_data)) {
        return;
    }
    if (rtk == nullptr) {
        rtk = (rtk_t*)malloc(sizeof(rtk_t));
        if (rtk == nullptr) {
            return;
        }
    }
    rtkinit(rtk, &prcopt);
    rtklib_initialised = true;
}

void AP_Periph_FW::rtklib_update()
{
    if (!rtklib_initialised) {
        return;
    }
    int8_t uart_num = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_GPS, 0);
    if (uart_num == -1) {
        return;
    }
    auto *uart = hal.serial(uart_num);
    if (uart == nullptr) {
        return;
    }

    if (rtklib_raw_buffer == nullptr) {
        rtklib_raw_buffer = new ByteBuffer(2048);
        if (rtklib_raw_buffer == nullptr) {
            return;
        }
        hal.serial(uart_num)->set_monitor_read_buffer(rtklib_raw_buffer);
    }

    uint8_t byte;
    // uint16_t ubx_len;
    // uint16_t ubx_offset=0;
    while (rtklib_raw_buffer->read_byte(&byte)) {
        // test_rawdata.data.data[test_rawdata_offset++] = byte;
        // test_rawdata.data.len = test_rawdata_offset;
        // if (test_rawdata_offset >= sizeof(test_rawdata.data.data)) {
        //     dronecan->ubx_raw_pub.broadcast(test_rawdata);
        //     test_rawdata_offset = 0;
        // }
        if (input_ubx(ubx_data, byte)) {
            // dronecan->ubx_raw_pub.broadcast(test_rawdata);
            // test_rawdata_offset = 0;
            sortobs(&ubx_data->obs);
            sortobs(&rtcm_data->obs);
            for (uint16_t i = 0; i < ubx_data->obs.n; i++) {
                ubx_data->obs.data[i].rcv = 1; //rover
            }
            for (uint8_t i = 0; i < rtcm_data->obs.n; i++) {
                memcpy(&ubx_data->obs.data[ubx_data->obs.n + i], &rtcm_data->obs.data[i], sizeof(obsd_t));
                ubx_data->obs.data[ubx_data->obs.n + i].rcv = 2; //base
            }
            // feed in the data to rtkpos
            rtkpos(rtk, ubx_data->obs.data, ubx_data->obs.n + rtcm_data->obs.n, &ubx_data->nav);
            // free the data from ubx_data
            for (uint16_t i = 0; i < rtcm_data->obs.n; i++) {
                memset(&ubx_data->obs.data[ubx_data->obs.n + i], 0, sizeof(obsd_t));
            }
            // uint64_t rtcm_time_ms = rtcm_data->obs.data[0].time.time*1000 + rtcm_data->obs.data[0].time.sec*1000;
            // uint64_t ubx_time_ms = ubx_data->obs.data[0].time.time*1000 + ubx_data->obs.data[0].time.sec*1000;
            // can_printf("RTKLIB: %d [%lld][%d] [%lld][%d] Stack Free[%ld]\n", rtk->sol.stat, rtcm_time_ms, rtcm_data->obs.n, ubx_time_ms, ubx_data->obs.n, stack_free(&__main_thread_stack_base__));
            debug("RTKLIB: %d AR: %f/%f\n", rtk->sol.stat, (float)rtk->sol.ratio, rtk->opt.thresar[0]);
            if (rtk->sol.stat) {
                double pos[3], diff_pos[3];
                ecef2pos(rtk->sol.rr, pos);
                // can_printf("RTKLIB Pos: %f %f %f\n", pos[0]*R2D, pos[1]*R2D, pos[2]);
                // print relative position
                // printf("RelPos: %.3f %.3f %.3f\n", rtk.sol.rr[0] - rtk.rb[0], rtk.sol.rr[1] - rtk.rb[1], rtk.sol.rr[2] - rtk.rb[2]);
                // print relative heading
                diff_pos[0] = rtk->sol.rr[0] - rtk->rb[0];
                diff_pos[1] = rtk->sol.rr[1] - rtk->rb[1];
                diff_pos[2] = rtk->sol.rr[2] - rtk->rb[2];
                double relpos[3];
                ecef2enu(pos, diff_pos, relpos);
                double heading = atan2(relpos[0], relpos[1]);
                debug("RelPos: %.3f %.3f %.3f RelHeading: %.3f\n", relpos[0], relpos[1], relpos[2], heading*R2D);
            }
        }
    }
}
#endif
