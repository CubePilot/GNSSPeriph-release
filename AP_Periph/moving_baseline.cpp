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
#ifdef ENABLE_RTKLIB

extern const AP_HAL::HAL &hal;

static const prcopt_t prcopt={ /* defaults processing options */
    // PMODE_KINEMA,SOLTYPE_FORWARD, /* mode,soltype */
    PMODE_MOVEB, SOLTYPE_FORWARD, /* mode,soltype */
    1,SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_CMP,  /* nf, navsys */
    15.0*D2R,{{0,0}},           /* elmin,snrmask */
    0,3,3,1,0,1,                /* sateph,modear,glomodear,gpsmodear,bdsmodear,arfilter */
    20,0,4,5,10,20,             /* maxout,minlock,minfixsats,minholdsats,mindropsats,minfix */
    1,0,0,0,0,                  /* armaxiter,no-estion,no-esttrop,dynamics,tidecorr */
    1,0,0,0,0,                  /* niter,codesmooth,intpref,sbascorr,sbassatsel */
    0,0,                        /* rovpos,refpos */
    {300.0},        /* eratio[] */
    {100.0,0.003,0.003,0.0,1.0,52.0,0.0,0.0}, /* err[-,base,el,bl,dop,snr_max,snr,rcverr] */
    {30.0,0.03,0.3},            /* std[] */
    {1E-4,1E-3,1E-4,1E-1,1E-2,0.0}, /* prn[] */
    5E-12,                      /* sclkstab */
    {3.0,0.25,0.0,1E-9,1E-5,3.0,3.0,0.0}, /* thresar */
	0.0,0.0,0.05,0,             /* elmaskar,elmaskhold,thresslip,thresdop, */
	0.1,0.01,30.0,              /* varholdamb,gainholdamb,maxtdif */
    {30.0,60.0},                 /* maxinno {phase,code} */
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
    while (rtklib_raw_buffer->read_byte(&byte)) {
        if (input_ubx(ubx_data, byte) > 0) {
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
            can_printf("RTKLIB: %d RTCM_Obs[%d] UBX_Obs[%d]\n", rtk->sol.stat, rtcm_data->obs.n, ubx_data->obs.n);
            if (rtk->sol.stat) {
                // convert solution to position
                double pos[3];
                ecef2pos(rtk->sol.rr, pos);
                // can_printf("RTKLIB Pos: %f %f %f\n", pos[0]*R2D, pos[1]*R2D, pos[2]);
                // print relative position
                can_printf("RelPos: %.3f %.3f %.3f\n", rtk->sol.rr[0] - rtk->rb[0], rtk->sol.rr[1] - rtk->rb[1], rtk->sol.rr[2] - rtk->rb[2]);
                // print relative heading
                double heading = atan2(rtk->sol.rr[1] - rtk->rb[1], rtk->sol.rr[0] - rtk->rb[0]);
                can_printf("RelHeading: %.3f\n", heading*R2D);
                // can_printf("ROVER Pos: %f %f %f\n", rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2]);
                // can_printf("BASE Pos: %f %f %f\n", rtk->rb[0], rtk->rb[1], rtk->rb[2]);
                // print relative heading
                // double heading = atan2(rtk->sol.rr[1] - rtk->rb[1], rtk->sol.rr[0] - rtk->rb[0]);

            }
        }
    }
}
#endif
