/*
 * MIT License
 * Copyright (c) 2019, 2018 - present OMRON Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/* includes */
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdbool.h>
#include <time.h>
#include <linux/i2c.h> //add

/* defines */
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02

#define N_ROW 4
#define N_PIXEL (4 * 4)
#define N_READ ((N_PIXEL + 1) * 2 + 1)

#define SAMPLE_TIME_0045MS	45
#define SAMPLE_TIME_0090MS	90
#define SAMPLE_TIME_0130MS	130
#define SAMPLE_TIME_0175MS	175
#define SAMPLE_TIME_0215MS	215
#define SAMPLE_TIME_0260MS	260
#define SAMPLE_TIME_0300MS	300
#define SAMPLE_TIME_0345MS	345
#define SAMPLE_TIME_0390MS	390
#define SAMPLE_TIME_0430MS	430
#define SAMPLE_TIME_0475MS	475
#define SAMPLE_TIME_0515MS	515
#define SAMPLE_TIME_0560MS	560
#define SAMPLE_TIME_0600MS	600
#define SAMPLE_TIME_0645MS	645
#define SAMPLE_TIME_0690MS	690
#define SAMPLE_TIME_0730MS	730
#define SAMPLE_TIME_0775MS	775
#define SAMPLE_TIME_0815MS	815
#define SAMPLE_TIME_0860MS	860
#define SAMPLE_TIME_0900MS	900
#define SAMPLE_TIME_0945MS	945
#define SAMPLE_TIME_0990MS	990
#define SAMPLE_TIME_1030MS	1030
#define SAMPLE_TIME_1070MS	1070
#define SAMPLE_TIME_1115MS	1115
#define SAMPLE_TIME_1160MS	1160
#define SAMPLE_TIME_1200MS	1200
#define SAMPLE_TIME_1245MS	1245
#define SAMPLE_TIME_1290MS	1290
#define SAMPLE_TIME_1330MS	1330
#define SAMPLE_TIME_1370MS	1370

#define PARA_0045MS	((uint8_t)0x01)
#define PARA_0090MS	((uint8_t)0x02)
#define PARA_0130MS	((uint8_t)0x03)
#define PARA_0175MS	((uint8_t)0x04)
#define PARA_0215MS	((uint8_t)0x05)
#define PARA_0260MS	((uint8_t)0x06)
#define PARA_0300MS	((uint8_t)0x07)
#define PARA_0345MS	((uint8_t)0x08)
#define PARA_0390MS	((uint8_t)0x09)
#define PARA_0430MS	((uint8_t)0x0A)
#define PARA_0475MS	((uint8_t)0x0B)
#define PARA_0515MS	((uint8_t)0x0C)
#define PARA_0560MS	((uint8_t)0x0D)
#define PARA_0600MS	((uint8_t)0x0E)
#define PARA_0645MS	((uint8_t)0x0F)
#define PARA_0690MS	((uint8_t)0x10)
#define PARA_0730MS	((uint8_t)0x11)
#define PARA_0775MS	((uint8_t)0x12)
#define PARA_0815MS	((uint8_t)0x13)
#define PARA_0860MS	((uint8_t)0x14)
#define PARA_0900MS	((uint8_t)0x15)
#define PARA_0945MS	((uint8_t)0x16)
#define PARA_0990MS	((uint8_t)0x17)
#define PARA_1030MS	((uint8_t)0x18)
#define PARA_1070MS	((uint8_t)0x19)
#define PARA_1115MS	((uint8_t)0x1A)
#define PARA_1160MS	((uint8_t)0x1B)
#define PARA_1200MS	((uint8_t)0x1C)
#define PARA_1245MS	((uint8_t)0x1D)
#define PARA_1290MS	((uint8_t)0x1E)
#define PARA_1330MS	((uint8_t)0x1F)
#define PARA_1370MS	((uint8_t)0x20)

/***** Setting Parameter *****/
#define comparingNumInc 5 // x300 ms  (range: 1 to 39) (example) 5 -> 1.5 sec
#define comparingNumDec 5  // x300 ms (range: 1 to 39)  (example) 5 -> 1.5 sec
#define threshHoldInc 10 //  /10 degC   (example) 10 -> 1.0 degC
#define threshHoldDec 10 //  /10 degC   (example) 10 -> 1.0 degC
bool  enablePix[16] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
/****************************/

/***** Setting Parameter 2 *****/
#define samplingTime SAMPLE_TIME_0300MS //ms (Can select only, 45ms, 90ms, 130ms, 175ms, 215ms, 260ms, 300ms, 435ms, 390ms, 430ms, 475ms, 515ms, 560ms, 600ms, 645ms, 690ms, 730ms, 775ms, 815ms, 860ms, 900ms, 945ms, 990ms, 1030ms, 1070ms, 1115ms, 1160ms, 1200ms, 1245ms, 1290ms, 1330ms, 1370ms)
/****************************/

#define RASPBERRY_PI_I2C    "/dev/i2c-1"
#define I2CDEV              RASPBERRY_PI_I2C

uint8_t rbuf[N_READ];
int16_t pix_data[16] = {0};
int16_t seqData[16][40] = {0};
bool  occuPix[16] = {0};
bool  occuPixFlag = false;
uint8_t  resultOccupancy = 0;
uint16_t  totalCount = 0;

/** JUDGE_occupancy: judge occupancy*/
bool judge_seatOccupancy(void) { 
  int i = 0;
  int j = 0; 
  for (i = 0; i < 16; i++){
    for (j = 0; j < 39; j++){
      seqData[i][39 - j] = seqData[i][38 - j];
    }
    seqData[i][0] = pix_data[i];            
  }
  if (totalCount <= comparingNumInc){
    totalCount++;
  }
  if (totalCount > comparingNumInc){
    for (i = 0; i < 16; i++){
      if (enablePix[i] == true){
        if (occuPix[i] == false){
          if ((int16_t)(seqData[i][0] - seqData[i][comparingNumInc]) >= (int16_t)threshHoldInc){
            occuPix[i] = true;
          }
        }
        else{   
          if ((int16_t)(seqData[i][comparingNumDec] - seqData[i][0]) >= (int16_t)threshHoldDec){
            occuPix[i] = false;
          }
        }
      }
    }
    if (resultOccupancy == 0) {
      for (i = 0; i < 16; i++){                   
        if(occuPix[i] == true){
          resultOccupancy = 1;
          break;
        }
      }
    }
    else{  //resultOccupancy == true
      occuPixFlag = false;
      for (i = 0; i < 16; i++){
        if (occuPix[i] == true){
          occuPixFlag = true;
          break;
        }
        else{                            
        }
      }
      if (occuPixFlag == false){
        resultOccupancy = 0;
      }
    }
  }
  return true;
}

void delay(int msec) {
    struct timespec ts = {.tv_sec = msec / 1000,
                          .tv_nsec = (msec % 1000) * 1000000};
    nanosleep(&ts, NULL);
}

/* I2C functions */
/** <!-- i2c_read_reg8 {{{1 --> I2C read function for bytes transfer.
 */
uint32_t i2c_read_reg8(uint8_t devAddr, uint8_t regAddr,
                       uint8_t *data, int length
) {
    int fd = open(I2CDEV, O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return 21;
    }
    int err = 0;
    do {
        if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
            fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
            err = 22; break;
        }
        if (write(fd, &regAddr, 1) != 1) {
            err = 23; break;
        }
	delay(1); //add
        int count = read(fd, data, length);
        if (count < 0) {
            err = 24; break;
        } else if (count != length) {
            fprintf(stderr, "Short read  from device, expected %d, got %d\n",
                    length, count);
            err = 25; break;
        }
    } while (false);
    close(fd);
    return err;
}

/** <!-- i2c_write_reg8 {{{1 --> I2C read function for bytes transfer.
 */
uint32_t i2c_write_reg8(uint8_t devAddr,
                        uint8_t *data, int length
) {
    int fd = open(I2CDEV, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return 21;
    }
    int err = 0;
    do {
        if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
            fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
            err = 22; break;
        }
        if (write(fd, data, length) != length) {
            fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
            err = 23; break;
        }
    } while (false);
    close(fd);
    return err;
}

uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
        fprintf(stderr,
                "PEC check failed: %02X(cal)-%02X(get)\n", crc, buf[n]);
    }
    return ret;
}


/** <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}




void initialSetting(void) {
	uint8_t para = 0;
	switch(samplingTime){
		case SAMPLE_TIME_0045MS:
			para = PARA_0045MS;
			break;
		case SAMPLE_TIME_0090MS:
			para = PARA_0090MS;
			break;
		case SAMPLE_TIME_0130MS:
			para = PARA_0130MS;
			break;
		case SAMPLE_TIME_0175MS:
			para = PARA_0175MS;
			break;
		case SAMPLE_TIME_0215MS:
			para = PARA_0215MS;
			break;
		case SAMPLE_TIME_0260MS:
			para = PARA_0260MS;
			break;
		case SAMPLE_TIME_0300MS:
			para = PARA_0300MS;
			break;
		case SAMPLE_TIME_0345MS:
			para = PARA_0345MS;
			break;
		case SAMPLE_TIME_0390MS:
			para = PARA_0390MS;
			break;
		case SAMPLE_TIME_0430MS:
			para = PARA_0430MS;
			break;
		case SAMPLE_TIME_0475MS:
			para = PARA_0475MS;
			break;
		case SAMPLE_TIME_0515MS:
			para = PARA_0515MS;
			break;
		case SAMPLE_TIME_0560MS:
			para = PARA_0560MS;
			break;
		case SAMPLE_TIME_0600MS:
			para = PARA_0600MS;
			break;
		case SAMPLE_TIME_0645MS:
			para = PARA_0645MS;
			break;
		case SAMPLE_TIME_0690MS:
			para = PARA_0690MS;
			break;
		case SAMPLE_TIME_0730MS:
			para = PARA_0730MS;
			break;
		case SAMPLE_TIME_0775MS:
			para = PARA_0775MS;
			break;
		case SAMPLE_TIME_0815MS:
			para = PARA_0815MS;
			break;
		case SAMPLE_TIME_0860MS:
			para = PARA_0860MS;
			break;
		case SAMPLE_TIME_0900MS:
			para = PARA_0900MS;
			break;
		case SAMPLE_TIME_0945MS:
			para = PARA_0945MS;
			break;
		case SAMPLE_TIME_0990MS:
			para = PARA_0990MS;
			break;
		case SAMPLE_TIME_1030MS:
			para = PARA_1030MS;
			break;
		case SAMPLE_TIME_1070MS:
			para = PARA_1070MS;
			break;
		case SAMPLE_TIME_1115MS:
			para = PARA_1115MS;
			break;
		case SAMPLE_TIME_1160MS:
			para = PARA_1160MS;
			break;
		case SAMPLE_TIME_1200MS:
			para = PARA_1200MS;
			break;
		case SAMPLE_TIME_1245MS:
			para = PARA_0945MS;
			break;
		case SAMPLE_TIME_1290MS:
			para = PARA_1290MS;
			break;
		case SAMPLE_TIME_1330MS:
			para = PARA_1330MS;
			break;
		case SAMPLE_TIME_1370MS:
			para = PARA_1370MS;
		default:
			para = PARA_0300MS;
			break;
	}
	
	uint8_t dat1[] = {0x50, 0x52, 0x45, 0x4C, 0x45, 0x41, 0x53, 0x45, 0x00};
    i2c_write_reg8(D6T_ADDR, dat1, sizeof(dat1));
    uint8_t dat2[] = {0x42, para, 0x00};
    i2c_write_reg8(D6T_ADDR, dat2, sizeof(dat2));
    uint8_t dat3[] = {0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    i2c_write_reg8(D6T_ADDR, dat3, sizeof(dat3));
    delay(2*samplingTime);

}

/** <!-- main - Thermal sensor {{{1 -->
 * 1. read sensor.
 * 2. output results, format is: [degC]
 */
int main() {
    int i, j;
	initialSetting();
	while(1){
		memset(rbuf, 0, N_READ);
		//for (i = 0; i < 10; i++) {
			uint32_t ret = i2c_read_reg8(D6T_ADDR, D6T_CMD, rbuf, N_READ);
			//if (ret == 0) {
			//    break;
			//} else if (ret == 23) {  // write error
			//    delay(60);
			//} else if (ret == 24) {  // read error
			//    delay(3000);
			//}
		//}

		if (D6T_checkPEC(rbuf, N_READ - 1)) {
			//return 2;
		}

		// 1st data is PTAT measurement (: Proportional To Absolute Temperature)
		int16_t itemp = conv8us_s16_le(rbuf, 0);
		printf("PTAT: %4.1f [degC], Temperature: ", itemp / 10.0);
	
		// loop temperature pixels of each thrmopiles measurements
		for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
			itemp = conv8us_s16_le(rbuf, j);
			pix_data[i] = itemp; //add
			printf("%4.1f", itemp / 10.0);  // print PTAT & Temperature
			if ((i % N_ROW) == N_ROW - 1) {
				printf(", ");  // wrap text at ROW end.
			} else {
				printf(", ");   // print delimiter
			}
		}
		judge_seatOccupancy(); //add
		printf("[degC], Occupancy: %d\n", resultOccupancy);  //add
		delay(samplingTime);  //add
		//return 0;
	}
}
// vi: ft=c:fdm=marker:et:sw=4:tw=80
