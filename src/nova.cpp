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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>

#include "nova.h"

GPSDriverNova::GPSDriverNova(GPSCallbackPtr callback, void *callback_user,
				   struct vehicle_gps_position_s *gps_position):
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	decodeInit();
	nova_msg.nova_state=nova_msg_parser::PREAMBLE1;
}

GPSDriverNova::~GPSDriverNova()
{
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

bool
GPSDriverNova::parseChar(uint8_t temp)
{
    switch (nova_msg.nova_state)
    {
        default:
        case nova_msg_parser::PREAMBLE1:
            if (temp == NOVA_PREAMBLE1)
                nova_msg.nova_state = nova_msg_parser::PREAMBLE2;
            nova_msg.read = 0;
            break;
        case nova_msg_parser::PREAMBLE2:
            if (temp == NOVA_PREAMBLE2)
            {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE3;
            }
            else
            {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
            }
            break;
        case nova_msg_parser::PREAMBLE3:
            if (temp == NOVA_PREAMBLE3)
            {
                nova_msg.nova_state = nova_msg_parser::HEADERLENGTH;
            }
            else
            {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
            }
            break;
        case nova_msg_parser::HEADERLENGTH:
            nova_msg.header.data[0] = NOVA_PREAMBLE1;
            nova_msg.header.data[1] = NOVA_PREAMBLE2;
            nova_msg.header.data[2] = NOVA_PREAMBLE3;
            nova_msg.header.data[3] = temp;
            nova_msg.header.nova_headeru.headerlength = temp;
            nova_msg.nova_state = nova_msg_parser::HEADERDATA;
            nova_msg.read = 4;
            break;
        case nova_msg_parser::HEADERDATA:
            if (nova_msg.read >= sizeof(nova_msg.header.data)) {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
                break;
            }
            nova_msg.header.data[nova_msg.read] = temp;
            nova_msg.read++;
            if (nova_msg.read >= nova_msg.header.nova_headeru.headerlength)
            {
                nova_msg.nova_state = nova_msg_parser::DATA;
            }
            break;
        case nova_msg_parser::DATA:
            if (nova_msg.read >= sizeof(nova_msg.data)) {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
                break;
            }
            nova_msg.data.bytes[nova_msg.read - nova_msg.header.nova_headeru.headerlength] = temp;
            nova_msg.read++;
            if (nova_msg.read >= (nova_msg.header.nova_headeru.messagelength + nova_msg.header.nova_headeru.headerlength))
            {
               nova_msg.nova_state = nova_msg_parser::CRC1;
            }
            break;
        case nova_msg_parser::CRC1:
            nova_msg.crc = (uint32_t) (temp << 0);
            nova_msg.nova_state = nova_msg_parser::CRC2;
            break;
        case nova_msg_parser::CRC2:
            nova_msg.crc += (uint32_t) (temp << 8);
            nova_msg.nova_state = nova_msg_parser::CRC3;
            break;
        case nova_msg_parser::CRC3:
            nova_msg.crc += (uint32_t) (temp << 16);
            nova_msg.nova_state = nova_msg_parser::CRC4;
            break;
        case nova_msg_parser::CRC4:
            nova_msg.crc += (uint32_t) (temp << 24);
            nova_msg.nova_state = nova_msg_parser::PREAMBLE1;

      //      uint32_t crc = CalculateBlockCRC32((uint32_t)nova_msg.header.nova_headeru.headerlength, (uint8_t *)&nova_msg.header.data, (uint32_t)0);
      //      crc = CalculateBlockCRC32((uint32_t)nova_msg.header.nova_headeru.messagelength, (uint8_t *)&nova_msg.data, crc);

     //       if (nova_msg.crc == crc)
     //       {
                return handleMessage();
    //        }
    //        else
    //        {
    //        	GPS_WARN("Nova: crc failed");
    //            crc_error_counter++;
    //        }
            break;
    }

    return false;
}

bool
GPSDriverNova::handleMessage(void)
{
    uint16_t messageid = nova_msg.header.nova_headeru.messageid;

    if (messageid == 42) // bestpos
    {
        const bestpos &bestposu = nova_msg.data.bestposu;

        _gps_position->timestamp = gps_absolute_time();
        _gps_position->lat = (int32_t) (bestposu.lat * (double)1e7);
        _gps_position->lon = (int32_t) (bestposu.lng * (double)1e7);
        _gps_position->alt = (int32_t) (bestposu.hgt * 1000);

        _gps_position->satellites_used = bestposu.svsused;
		_gps_position->eph = sqrtf(static_cast<float>(bestposu.latsdev) * static_cast<float>(bestposu.latsdev)
					   + static_cast<float>(bestposu.lngsdev) * static_cast<float>(bestposu.lngsdev));

		_gps_position->epv = static_cast<float>(bestposu.hgtsdev);

		_gps_position->s_variance_m_s = 0;

        if (bestposu.solstat == 0) // have a solution
        {
            switch (bestposu.postype)
            {
                case 16:
                	_gps_position->fix_type = 3;
                    break;
                case 17: // psrdiff
                case 18: // waas
                case 20: // omnistar
                case 68: // ppp_converg
                case 69: // ppp
                	_gps_position->fix_type = 4;//GPS_OK_FIX_3D_DGPS;
                    break;
                case 32: // l1 float
                case 33: // iono float
                case 34: // narrow float
                	_gps_position->fix_type = 5;//GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 48: // l1 int
                case 50: // narrow int
                	_gps_position->fix_type = 6;//GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 0: // NONE
                case 1: // FIXEDPOS
                case 2: // FIXEDHEIGHT
                default:
                	_gps_position->fix_type = 0;
                    break;
            }
        }
        else
        {
        	_gps_position->fix_type = 0;
        }

        _new_position = true;
    }

    if (messageid == 99) // bestvel
    {
        const bestvel &bestvelu = nova_msg.data.bestvelu;

		float track_rad = static_cast<float>(bestvelu.trkgnd) * M_PI_F / 180.0f;

		float velocity_ms = static_cast<float>(bestvelu.horspd);			/** knots to m/s */
		float velocity_north = static_cast<float>(velocity_ms) * cosf(track_rad);
		float velocity_east  = static_cast<float>(velocity_ms) * sinf(track_rad);

		_gps_position->vel_m_s = velocity_ms;				/** GPS ground speed (m/s) */
		_gps_position->vel_n_m_s = velocity_north;			/** GPS ground speed in m/s */
		_gps_position->vel_e_m_s = velocity_east;			/** GPS ground speed in m/s */

        _gps_position->vel_n_m_s = -(float) bestvelu.vertspd;   /** GPS ground speed in m/s */
        _gps_position->vel_ned_valid = true;				/** Flag to indicate if NED speed is valid */
        _gps_position->cog_rad = track_rad;
        _gps_position->c_variance_rad = 0.1f;
        _new_speed = true;
    }

    if (messageid == 174) // psrdop
    {
        const psrdop &psrdopu = nova_msg.data.psrdopu;
        _gps_position->hdop = psrdopu.hdop;
        _gps_position->vdop = sqrtf(psrdopu.pdop* psrdopu.pdop-psrdopu.hdop*psrdopu.hdop);
    }

    // ensure out position and velocity stay insync
    if (_new_position && _new_speed ) {
        _new_speed = _new_position = false;

        return true;
    }

    return false;
}

int GPSDriverNova::receive(unsigned timeout)
{

		uint8_t buf[GPS_READ_BUFFER_SIZE];

		/* timeout additional to poll */
		uint64_t time_started = gps_absolute_time();

		int j = 0;
		ssize_t bytes_count = 0;

		while (true) {

			/* pass received bytes to the packet decoder */
			while (j < bytes_count) {

				if (parseChar(buf[j])) {
					/* return to configure during configuration or to the gps driver during normal work
					 * if a packet has arrived */
						return 1;
					}

				j++;
			}

			/* everything is read */
			j = bytes_count = 0;

			/* then poll or read for new data */
			int ret = read(buf, sizeof(buf), timeout * 2);

			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */

			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				bytes_count = ret;
			}

			/* in case we get crap from GPS or time out */
			if (time_started + timeout * 1000 * 2 < gps_absolute_time()) {
				return -1;
			}
		}
}


void GPSDriverNova::decodeInit()
{

}

/*
 * nove board configuration script
 */
const char comm[] = "\r\n\r\nunlogall\r\n"\
		  "log bestposb ontime 0.2 nohold\r\n"\
		  "log bestvelb ontime 0.2 nohold\r\n"\
		  "log psrdopb onchanged\r\n"\
		  "log psrdopb ontime 0.2\r\n";

int GPSDriverNova::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("ASHTECH: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	/* try different baudrates */
	const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};


	for (unsigned int baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++) {
		baudrate = baudrates_to_try[baud_i];
		setBaudrate(baudrate);

		if (write(comm, sizeof(comm)) != sizeof(comm)) {
			return -1;
		}
	}

	return setBaudrate(115200);
}
