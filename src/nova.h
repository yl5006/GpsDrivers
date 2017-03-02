/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
//  Novatel/Tersus/ComNav GPS driver for px4 Firmware.
//  Code by yaoling
//  Derived from http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf
 */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"


class GPSDriverNova : public GPSHelper
{
public:
	GPSDriverNova(GPSCallbackPtr callback, void *callback_user,
		     struct vehicle_gps_position_s *gps_position);
	virtual ~GPSDriverNova();
	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

private:

    void decodeInit(void);
    bool parseChar(uint8_t temp);
    bool handleMessage();
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);

    static const uint8_t NOVA_PREAMBLE1 = 0xaa;
    static const uint8_t NOVA_PREAMBLE2 = 0x44;
    static const uint8_t NOVA_PREAMBLE3 = 0x12;

    // do we have new position information?
    bool            _new_position:1;
    // do we have new speed information?
    bool            _new_speed:1;

    uint32_t        _last_vel_time;

    uint32_t crc_error_counter = 0;

	struct vehicle_gps_position_s *_gps_position;

    typedef struct {
        // 0
        uint8_t preamble[3];
        // 3
        uint8_t headerlength;
        // 4
        uint16_t messageid;
        // 6
        uint8_t messagetype;
        //7
        uint8_t portaddr;
        //8
        uint16_t messagelength;
        //10
        uint16_t sequence;
        //12
        uint8_t idletime;
        //13
        uint8_t timestatus;
        //14
        uint16_t week;
        //16
        uint32_t tow;
        //20
        uint32_t recvstatus;
        // 24
        uint16_t resv;
        //26
        uint16_t recvswver;
    } nova_header;

    typedef struct {
        float gdop;
        float pdop;
        float hdop;
        float htdop;
        float tdop;
        float cutoff;
        uint32_t svcount;
        // extra data for individual prns
    } psrdop;

    typedef struct {
        uint32_t solstat;
        uint32_t postype;
        double lat;
        double lng;
        double hgt;
        float undulation;
        uint32_t datumid;
        float latsdev;
        float lngsdev;
        float hgtsdev;
        // 4 bytes
        uint8_t stnid[4];
        float diffage;
        float sol_age;
        uint8_t svstracked;
        uint8_t svsused;
        uint8_t svsl1;
        uint8_t svsmultfreq;
        uint8_t resv;
        uint8_t extsolstat;
        uint8_t galbeisigmask;
        uint8_t gpsglosigmask;
    } bestpos;

    typedef struct {
        uint32_t solstat;
        uint32_t veltype;
        float latency;
        float age;
        double horspd;
        double trkgnd;
        // + up
        double vertspd;
        float resv;
    } bestvel;

    typedef  union {
        bestvel bestvelu;
        bestpos bestposu;
        psrdop psrdopu;
        uint8_t bytes[256];
    } msgbuffer;

    typedef  union {
        nova_header nova_headeru;
        uint8_t data[28];
    } msgheader;

    typedef  struct {
        enum
        {
            PREAMBLE1 = 0,
            PREAMBLE2,
            PREAMBLE3,
            HEADERLENGTH,
            HEADERDATA,
            DATA,
            CRC1,
            CRC2,
            CRC3,
            CRC4,
        } nova_state;

        msgbuffer data;
        uint32_t crc;
        msgheader header;
        uint16_t read;
    }nova_msg_parser;
    nova_msg_parser nova_msg;
};
