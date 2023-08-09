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
    0,ARMODE_FIXHOLD,ARMODE_FIXHOLD,1,ARMODE_CONT,1,                /* sateph,modear,glomodear,gpsmodear,bdsmodear,arfilter */
    100,5,4,5,10,100,             /* maxout,minlock,minfixsats,minholdsats,mindropsats,minfix */
    4,1,1,0,0,                  /* armaxiter,estion,esttrop,dynamics,tidecorr */
    1,0,0,0,0,                  /* niter,codesmooth,intpref,sbascorr,sbassatsel */
    0,0,                        /* rovpos,refpos */
    {300.0},             /* eratio[] */
    {100.0,0.003,0.003,0.0,1.0,52.0,0.0,0.0}, /* err[-,base,el,bl,dop,snr_max,snr,rcverr] */
    {30.0,0.03,0.3},            /* std[] */
    {1E-4,1E-3,1E-4,1E-1,1E-2,0.0}, /* prn[] */
    5E-12,                      /* sclkstab */
    {3.0,0.04,0.0,1E-9,1E-5,3.0,30.0,0.0}, /* thresar */
	20.0*D2R,0.0,0.05,0,             /* elmaskar,elmaskhold,thresslip,thresdop, */
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
    int ret;
    if (!rtklib_initialised) {
        return;
    }
    if (rtcm_data == nullptr) {
        return;
    }
    uint64_t rtcm_time_ms = 0;
    for (uint16_t i = 0; i < len; i++) {
        rtcm_data->time = utc2gpst(timeget());
        ret = input_rtcm3(rtcm_data, data[i]);
        if (ret == 1) {
            rtcm_time_ms = rtcm_data->obs.data[0].time.time*1000 + rtcm_data->obs.data[0].time.sec*1000;
            if ((rtcm_data->obs.n > 4) && (rtcm_time_ms != last_rtcm_obs_time)) {
                new_rtcm_obs = true;
                if (last_rtcm_obs_time != 0) {
                    int32_t rtcm_obs_time_diff = rtcm_time_ms - last_rtcm_obs_time;
                    if (rtcm_obs_time_diff > 250) {
                        debug("RTCM: High Time diff %ld", rtcm_obs_time_diff);
                    }
                }
                last_rtcm_obs_time = rtcm_time_ms;
            }
        }
    }
}

void AP_Periph_FW::rtklib_init()
{
    AP_Param *vp;
    enum ap_var_type ptype;
    vp = AP_Param::find("GPS_RAW_DATA", &ptype);

    if (gps.get_type(0) != AP_GPS::GPS_TYPE_UBLOX_RTK_ROVER) {
        ((AP_Int8 *)vp)->set_and_save_ifchanged(0);
        return;
    }
    ((AP_Int8 *)vp)->set_and_save_ifchanged(1);
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

static void soltocov(const sol_t *sol, double *P)
{
    P[0]     =sol->qr[0]; /* xx or ee */
    P[4]     =sol->qr[1]; /* yy or nn */
    P[8]     =sol->qr[2]; /* zz or uu */
    P[1]=P[3]=sol->qr[3]; /* xy or en */
    P[5]=P[7]=sol->qr[4]; /* yz or nu */
    P[2]=P[6]=sol->qr[5]; /* zx or ue */
}

/* sqrt of covariance --------------------------------------------------------*/
static double sqvar(double covar)
{
    return covar<0.0?-sqrt(-covar):sqrt(covar);
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
        uint64_t ubx_time_ms;
        // test_rawdata.data.data[test_rawdata_offset++] = byte;
        // test_rawdata.data.len = test_rawdata_offset;
        // if (test_rawdata_offset >= sizeof(test_rawdata.data.data)) {
        //     dronecan->ubx_raw_pub.broadcast(test_rawdata);
        //     test_rawdata_offset = 0;
        // }
        if (input_ubx(ubx_data, byte)) {
            // dronecan->ubx_raw_pub.broadcast(test_rawdata);
            // test_rawdata_offset = 0;
            // schedule rtkpos calc in 50ms from now if we have new set of observations
            ubx_time_ms = ubx_data->obs.data[0].time.time*1000 + ubx_data->obs.data[0].time.sec*1000;
            if ((ubx_data->obs.n > 4) && (ubx_time_ms != last_ubx_obs_time)) {
                new_ubx_obs = true;
                if (last_ubx_obs_time != 0) {
                    int32_t ubx_obs_time_diff = ubx_time_ms - last_ubx_obs_time;
                    if (ubx_obs_time_diff > 250) {
                        debug("UBX: High Time diff %ld", ubx_obs_time_diff);
                    }
                }
                last_ubx_obs_time = ubx_time_ms;
            }
        }
    }
    if (new_rtcm_obs && new_ubx_obs) {
        new_ubx_obs = false;
        new_rtcm_obs = false;
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
        debug("RTKLIB: %d AR: %f Sats: R:%d B:%d P[%lu] Pp[%lu] %lu", rtk->sol.stat, (float)rtk->sol.ratio, ubx_data->obs.n, rtcm_data->obs.n, get_RTK_P_consumed(), get_RTK_Pp_consumed(), hal.util->available_memory());
        // debug("RTKLIB: Available Memory: %lu", hal.util->available_memory());
        uint64_t rtcm_mean_time_ms = 0, rtcm_max_time_ms = 0, rtcm_min_time_ms = UINT64_MAX;
        uint64_t ubx_mean_time_ms = 0, ubx_max_time_ms = 0, ubx_min_time_ms = UINT64_MAX;
        for (uint16_t i = 0; i < rtcm_data->obs.n; i++) {
            uint64_t rtcm_time_ms = rtcm_data->obs.data[i].time.time*1000 + rtcm_data->obs.data[i].time.sec*1000;
            rtcm_mean_time_ms += rtcm_time_ms;
            if (rtcm_time_ms > rtcm_max_time_ms) {
                rtcm_max_time_ms = rtcm_time_ms;
            }
            if (rtcm_time_ms < rtcm_min_time_ms) {
                rtcm_min_time_ms = rtcm_time_ms;
            }
        }
        rtcm_mean_time_ms /= rtcm_data->obs.n;
        for (uint16_t i = 0; i < ubx_data->obs.n; i++) {
            uint64_t ubx_time_ms = ubx_data->obs.data[i].time.time*1000 + ubx_data->obs.data[i].time.sec*1000;
            ubx_mean_time_ms += ubx_time_ms;
            if (ubx_time_ms > ubx_max_time_ms) {
                ubx_max_time_ms = ubx_time_ms;
            }
            if (ubx_time_ms < ubx_min_time_ms) {
                ubx_min_time_ms = ubx_time_ms;
            }
        }
        ubx_mean_time_ms /= ubx_data->obs.n;

        // debug("Time: RTCM[%llu] [%ld] UBX[%llu]", rtcm_mean_time_ms, int32_t(rtcm_mean_time_ms - ubx_mean_time_ms), ubx_mean_time_ms);
        // debug("UBX Process Time: %lu", AP_HAL::millis());

        if (rtk->sol.stat == 1) {
            double pos[3], diff_pos[3];
            double P[9],Q[9];
            ecef2pos(rtk->sol.rr, pos);
            // can_printf("RTKLIB Pos: %f %f %f\n", pos[0]*R2D, pos[1]*R2D, pos[2]);
            // print relative position
            // printf("RelPos: %.3f %.3f %.3f\n", rtk.sol.rr[0] - rtk.rb[0], rtk.sol.rr[1] - rtk.rb[1], rtk.sol.rr[2] - rtk.rb[2]);
            // print relative heading
            diff_pos[0] = rtk->sol.rr[0] - rtk->rb[0];
            diff_pos[1] = rtk->sol.rr[1] - rtk->rb[1];
            diff_pos[2] = rtk->sol.rr[2] - rtk->rb[2];
            soltocov(&rtk->sol,P);
            covenu(pos,P,Q);
            double relpos[3];
            ecef2enu(pos, diff_pos, relpos);
            relPosLength = sqrt(sq(relpos[0]) + sq(relpos[1]));
            relPosD = relpos[2];
            relPosHeading = atan2(relpos[0], relpos[1]);
            // accuracy is calculated using std deviation of N and E and NE vector length 
            relPosAccHeading = atan2(sqvar(Q[0] + Q[4]), relPosLength - sqvar(Q[1]));
            relPosTimestamp = rtk->sol.time.time*1000 + rtk->sol.time.sec*1000;
            // debug("Time: %.3f RelPosLength: %.3f  RelHeading: %.3f Accuracy: %.3f", rtk->sol.time.time%1000 + rtk->sol.time.sec, relPosLength, relPosHeading*R2D, relPosAccHeading*R2D);
        }
        ubx_data->obs.n = 0;
        rtcm_data->obs.n = 0;
    }
}
#endif
