/*
 * Rafael Micro R820T/R828D driver
 *
 * Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
 *
 * This driver is a heavily modified version of the driver found in the
 * Linux kernel:
 * http://git.linuxtv.org/linux-2.6.git/history/HEAD:/drivers/media/tuners/r820t.c
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>


#include "rtlsdr_i2c.h"
#include "tuner_r848.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MHZ(x)		((x)*1000*1000)
#define KHZ(x)		((x)*1000)

uint32_t R848_LNA_HIGH_MID[R848_TF_SIZE] = { 644000, 644000, 644000, 644000, 644000, 500000, 500000, 500000, 500000};
uint32_t R848_LNA_MID_LOW[R848_TF_SIZE] = { 388000, 388000, 348000, 348000, 348000, 300000, 300000, 300000, 300000};
uint32_t R848_LNA_LOW_LOWEST[R848_TF_SIZE] = {164000, 164000, 148000, 124000, 124000, 156000, 156000, 108000, 108000};

uint8_t R848_iniArray_hybrid[R848_REG_NUM] = {
                                                0x00, 0x00, 0x40, 0x44, 0x17, 0x00, 0x06, 0xF0, 0x00, 0x41,
                                        //  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F  0x10  0x11
                                                0x7B, 0x0B, 0x70, 0x06, 0x6E, 0x20, 0x70, 0x87, 0x96, 0x00,
                                        //  0x12  0x13  0x14  0x15  0x16  0x17  0x18  0x19  0x1A  0x1B
                                                0x10, 0x00, 0x80, 0xA5, 0xB7, 0x00, 0x40, 0xCB, 0x95, 0xF0,
                                        //  0x1C  0x1D  0x1E  0x1F  0x20  0x21  0x22  0x23  0x24  0x25
                                                0x24, 0x00, 0xFD, 0x8B, 0x17, 0x13, 0x01, 0x07, 0x01, 0x3F};
                                        //  0x26  0x27  0x28  0x29  0x2A  0x2B  0x2C  0x2D  0x2E  0x2F



uint8_t R848_iniArray_dvbs[R848_REG_NUM] = {
                                                0x80, 0x05, 0x40, 0x40, 0x1F, 0x1F, 0x07, 0xFF, 0x00, 0x40,
                                        //  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F  0x10  0x11
                                                0xF0, 0x0F, 0x4D, 0x06, 0x6F, 0x20, 0x28, 0x83, 0x96, 0x00,  //0x16[1] pulse_flag HPF : Bypass ;  0x19[1:0] Deglich SW Cur : highest
                                        //  0x12  0x13  0x14  0x15  0x16  0x17  0x18  0x19  0x1A  0x1B
                                                0x1C, 0x99, 0xC1, 0x83, 0xB7, 0x00, 0x4F, 0xCB, 0x95, 0xFD,
                                        //  0x1C  0x1D  0x1E  0x1F  0x20  0x21  0x22  0x23  0x24  0x25
                                                0xA4, 0x01, 0x24, 0x0B, 0x4F, 0x05, 0x01, 0x47, 0x3F, 0x3F};
                                        //  0x26  0x27  0x28  0x29  0x2A  0x2B  0x2C  0x2D  0x2E  0x2F


R848_SectType R848_IMR_Data[5] = {
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0}
                                                };//Please keep this array data for standby mode.



/*
 * Static constants
 */

/* Those initial values start from R848_REG_SHADOW_START */
static const uint8_t r848_init_array[R848_NUM_REGS] = {
};

/* Tuner frequency ranges */
static const struct r848_freq_range freq_ranges[] = {
};

static int r848_xtal_capacitor[][2] = {
};

/*
 * I2C read/write code and shadow registers logic
 */
static void r848_shadow_store(struct r848_priv *priv, uint8_t reg, const uint8_t *val,
			 int len)
{
	int i;
	int r = reg - R848_REG_SHADOW_START;

	if (r < 0) {
		len += r;
		r = 0;
	}
	if (len <= 0)
		return;
	if (len > R848_NUM_REGS - r)
		len = R848_NUM_REGS - r;

	memcpy(&priv->regs[r], val, len);
}

static int r848_write(struct r848_priv *priv, uint8_t reg, const uint8_t *val,
		       unsigned int len)
{
	int rc, size, pos = 0;
	/* Store the shadow registers */
	r848_shadow_store(priv, reg, val, len);

	
	do {
		if (len > priv->cfg->max_i2c_msg_len - 1)
			size = priv->cfg->max_i2c_msg_len - 1;
		else
			size = len;
		/* Fill I2C buffer */
		priv->buf[0] = reg;
		memcpy(&priv->buf[1], &val[pos+5], size);
		//printf("Writing value %x to register %x of size %x with %d left\n",priv->buf[1], pos, size+1, len);
		rc = rtlsdr_i2c_write_fn(priv->rtl_dev, priv->cfg->i2c_addr,
					 priv->buf, size + 1);

		if (rc != size + 1) {
			fprintf(stderr, "%s: i2c wr failed=%d reg=%02x len=%d\n",
				   __FUNCTION__, rc, reg, size);
			if (rc < 0)
				return rc;
			return -1;
		}
		reg += size;
		len -= size;
		pos += size;
	} while (len > 0);

	return 0;
}

static int r848_write_reg(struct r848_priv *priv, uint8_t reg, uint8_t val)
{
	return r848_write(priv, reg, &val, 1);
}

static int r848_read_cache_reg(struct r848_priv *priv, int reg)
{
	reg -= R848_REG_SHADOW_START;

	if (reg >= 0 && reg < R848_NUM_REGS)
		return priv->regs[reg];
	else
		return -1;
}

static int r848_write_reg_mask(struct r848_priv *priv, uint8_t reg, uint8_t val,
				uint8_t bit_mask)
{
	int rc = r848_read_cache_reg(priv, reg);

	if (rc < 0)
		return rc;

	val = (rc & ~bit_mask) | (val & bit_mask);

	return r848_write(priv, reg, &val, 1);
}

static uint8_t r848_bitrev(uint8_t byte)
{
	const uint8_t lut[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
				  0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	return (lut[byte & 0xf] << 4) | lut[byte >> 4];
}

static int r848_read(struct r848_priv *priv, uint8_t reg, uint8_t *val, int len)
{
	int rc, i;
	uint8_t *p = &priv->buf[1];

	priv->buf[0] = reg;

	rc = rtlsdr_i2c_write_fn(priv->rtl_dev, priv->cfg->i2c_addr, priv->buf, 1);
	if (rc < 1)
		return rc;

	rc = rtlsdr_i2c_read_fn(priv->rtl_dev, priv->cfg->i2c_addr, p, len);

	if (rc != len) {
		fprintf(stderr, "%s: i2c rd failed=%d reg=%02x len=%d\n",
			   __FUNCTION__, rc, reg, len);
		if (rc < 0)
			return rc;
		return -1;
	}

	/* Copy data to the output buffer */
	for (i = 0; i < len; i++)
		val[i] = r848_bitrev(p[i]);

	return 0;
}

/*
 * r848 tuning logic
 */

static int r848_set_tv_standard(struct r848_priv *priv, unsigned bw, R848_Standard_Type type, uint32_t delsys)

{
return 1;
}

static int r848_read_gain(struct r848_priv *priv)
{
return 1;
}

int r848_set_gain(struct r848_priv *priv, int set_manual_gain, int gain)
{
	int rc;
	uint8_t buf[5];

	uint8_t MixerGain = 0;
	uint8_t RfGain = 0;
	uint8_t LnaGain = 0;
	I2C_TYPE  R848_I2C;
	I2C_LEN_TYPE R848_I2C_Len;


	printf("Setting gain to %d with autogain %d\n", gain, set_manual_gain);

	if(set_manual_gain)
	{
		//LNA auto off
	     priv->cfg->R848_Array[5] = priv->cfg->R848_Array[5] | 0x80;   // 848:13[7:0]   
	rc = r848_write_reg(priv, 0x0D, priv->cfg->R848_Array[5]);

		 //Mixer buffer off
	     priv->cfg->R848_Array[26] = priv->cfg->R848_Array[26] | 0x10;  // 848:34[7:0]   
        rc = r848_write_reg(priv, 0x22, priv->cfg->R848_Array[26]);

		 //Mixer auto off
	     priv->cfg->R848_Array[7] = priv->cfg->R848_Array[7] & 0xEF;  //848:15[6:0]
	rc = r848_write_reg(priv, 0x0F, priv->cfg->R848_Array[7]);

       		rc = r848_read(priv , 0x00 , buf , 5);

		//MixerGain = (((buf[1] & 0x40) >> 6)<<3)+((buf[3] & 0xE0)>>5);   //?
		MixerGain = (buf[3] & 0x0F); //mixer // 848:3[4:0]
		RfGain = ((buf[3] & 0xF0) >> 4);   //rf		 // 848:3[4:0] 
		LnaGain = buf[4] & 0x1F;  //lna    // 848:4[4:0]  

		//set LNA gain
	     priv->cfg->R848_Array[5] = (priv->cfg->R848_Array[5] & 0xE0) | LnaGain;  // 848:13[7:0]  
        rc = r848_write_reg(priv, 0x0D, priv->cfg->R848_Array[5]);


		 //set Mixer Buffer gain
	     priv->cfg->R848_Array[26] = (priv->cfg->R848_Array[26] & 0xF0) | RfGain;  //848:34[7:0] 
        rc = r848_write_reg(priv, 0x22, priv->cfg->R848_Array[26]);

		 //set Mixer gain
	     priv->cfg->R848_Array[7] = (priv->cfg->R848_Array[7] & 0xF0) | MixerGain; // 848:15[6:0]  
        rc = r848_write_reg(priv, 0x0F, priv->cfg->R848_Array[7]);

	}
	else
	{
	    //LNA auto on
	     priv->cfg->R848_Array[5] = priv->cfg->R848_Array[5] & 0x7F;  // 848:13[7:0]  
        rc = r848_write_reg(priv, 0x0D, priv->cfg->R848_Array[5]);

		 //Mixer buffer on
	     priv->cfg->R848_Array[26] = priv->cfg->R848_Array[26] & 0xEF;	// 848:34[7:0]  
        rc = r848_write_reg(priv, 0x22, priv->cfg->R848_Array[26]);

		 //Mixer auto on
	     priv->cfg->R848_Array[7] = priv->cfg->R848_Array[7] | 0x10;	// 848:15[6:0]  
        rc = r848_write_reg(priv, 0x0F, priv->cfg->R848_Array[7]);

	}

return 1;
}

int r848_set_bandwidth(struct r848_priv *priv, int bw, uint32_t rate)
{

	int ret = 0, i;
	uint8_t tuner_lock;
	int satellite_system = 1;
	uint32_t frequency = priv->int_freq;

	R848_Set_Info R848_INFO;

	/* failsafe */
	R848_INFO.R848_Standard = R848_DVB_T2_8M_IF_5M;


	if (satellite_system) {
		R848_INFO.RF_KHz = frequency;
		R848_INFO.R848_Standard = R848_DVB_S;
		R848_INFO.DVBS_BW = (rate/200*135+2000000)/1000*2;//unit KHz
		R848_INFO.R848_DVBS_OutputSignal_Mode = DIFFERENTIALOUT;
		R848_INFO.R848_DVBS_AGC_Mode = AGC_NEGATIVE;

		/* set pll data */
		if(R848_DVBS_Setting(priv,R848_INFO) != RT_Success)
			return RT_Fail;
	} else {		
		R848_INFO.RF_KHz = frequency / 1000;
		if(bw <= 1700000) {
			R848_INFO.R848_Standard = R848_DVB_T2_1_7M_IF_5M;
		} else if (bw <= 6000000) {
			R848_INFO.R848_Standard = R848_DVB_T2_6M_IF_5M;
		} else if (bw <= 7000000) {
			R848_INFO.R848_Standard = R848_DVB_T2_7M_IF_5M;
		} else if (bw == 8000000) {
			R848_INFO.R848_Standard = R848_DVB_T2_8M_IF_5M;
		}

		/* set pll data */
		//if(r848_set_standard(priv, R848_INFO.R848_Standard) != RT_Success) {
		//	return RT_Fail;
		//}
		//if(r848_set_freq(priv, frequency) != RT_Success) {
		//	return RT_Fail;
		//}
		
	}



	//if (ret) {
	//	printk("[r848_lock_n_wait] Tuner lock function Failed!\n");
	//	goto exit;
	//}
	//for (i = 0; i < 5; i++) {
	//	ret = r848_get_lock_status(priv, &tuner_lock);
	//	if(tuner_lock) {
	//		printf("Tuner Locked.\n");
	//		break;
	//	} else {
	//		printf("Tuner not Locked!\n");
	//	}
	//	usleep(20);
	//}

exit:
	return ret;



}


/*
 * r848 standby logic
 */

int r848_standby(struct r848_priv *priv)
{
return 1;
}

/*
 * r848 device init logic
 */

static int r848_xtal_check(struct r848_priv *priv)
{
	return 1;
}

int r848_init(struct r848_priv *priv)
{
	int rc;

	/* Initialize registers */
	rc = r848_write(priv, 0x08, R848_iniArray_dvbs, sizeof(R848_iniArray_dvbs));
//	rc = r848_print_registers(priv);

	printf("init tuner\n");
	if (priv->inited == 1)
		return 0;
	priv->inited = 1;
	printf("init tuner first time\n");

	rc = r848_tf_check(priv);

	//start IMR calibration

	rc = r848_cal_prepare(priv,R848_IMR_CAL);

	rc = r848_imr(priv,3, 0);

	rc = r848_imr(priv,0, 1);

	rc = r848_imr(priv,1, 1);

	rc = r848_imr(priv,2, 1);

	rc = r848_imr(priv,4, 0);

	//do Xtal check
        //write initial reg
        rc = r848_write(priv, 0x08, R848_iniArray_dvbs, sizeof(R848_iniArray_dvbs));

	priv->cfg->R848_Xtal_Pwr = XTAL_SMALL_LOWEST;
	priv->cfg->R848_Xtal_Pwr_tmp = XTAL_SMALL_LOWEST;

//	for (i=0; i<3; i++) {
//		if(R848_Xtal_Check(priv) != RT_Success)
//			return RT_Fail;

//		if(priv->cfg->R848_Xtal_Pwr_tmp > priv->cfg->R848_Xtal_Pwr)
//			priv->cfg->R848_Xtal_Pwr = priv->cfg->R848_Xtal_Pwr_tmp;
//	}

	//write initial reg
	rc = r848_write(priv, 0x08, R848_iniArray_dvbs, sizeof(R848_iniArray_dvbs));
	
	priv->cfg->R848_pre_standard = R848_STD_SIZE;


	printf("init tuner done\n");

	return rc;
}

static int r848_get_lock_status(struct r848_priv *priv, uint8_t *lock)
{
	int rc;
	uint8_t buf[3];

	rc = r848_read(priv , 0x00 , buf , 3);
	
	if ((buf[2] & 0x40) == 0x00)
		*lock = 0;
	else
	  	*lock = 1;

	return rc;
}

int r848_print_registers(struct r848_priv *priv)
{
	int i,rc;
	uint8_t val[47];

	for (i=0; i <47 ; i++)
		val[i] = 0x00;

	r848_read(priv , 0x0 , &val[0] ,16);
//	r848_read(priv , 0x15 , &val[15] , 16);
	
	for (i=0; i <47 ; i++)
	printf (" %02x ",val[i]);
	printf ("\n");
	return 1;
}

//-----------------------------------------------------------------------------------/ 
// Purpose: read multiple IMC results for stability
// input: IMR_Reg: IMC result address
//        IMR_Result_Data: result 
// output: TRUE or FALSE
//-----------------------------------------------------------------------------------/
static int R848_Muti_Read( struct r848_priv *priv,uint8_t* IMR_Result_Data)
{
	uint8_t ReadCunt     = 0;
	uint16_t ReadAmount  = 0;
	uint8_t ReadMax = 0;
	uint8_t ReadMin = 255;
	uint8_t ReadData = 0;
	I2C_LEN_TYPE R848_I2C_Len;

	uint8_t buf[2];
	int ret;


    //msleep(3);//3

	for(ReadCunt = 0; ReadCunt < 3; ReadCunt ++) {
#if 0
		R848_I2C_Len.RegAddr = 0x00;
		R848_I2C_Len.Len = 2;              // read 0x01
		if(I2C_Read_Len(priv,&R848_I2C_Len) != RT_Success)
		{
			I2C_Read_Len(priv,&R848_I2C_Len);
		}
#endif
		//ret = r848_rdm(priv, 0x00, buf, 2);
		//ReadData = (buf[1] & 0x3f);
		
		ReadAmount = ReadAmount + (uint16_t)ReadData;
		
		if(ReadData < ReadMin)
			ReadMin = ReadData;
		
        if(ReadData > ReadMax)
			ReadMax = ReadData;
	}
	*IMR_Result_Data = (uint8_t) (ReadAmount - (uint16_t)ReadMax - (uint16_t)ReadMin);


	return RT_Success;
}

R848_Freq_Info_Type R848_Freq_Sel(uint32_t LO_freq, uint32_t RF_freq,R848_Standard_Type R848_Standard)
{
	R848_Freq_Info_Type R848_Freq_Info;

	//----- RF dependent parameter --------????
	//LNA band  R13[6:5]
	if(RF_freq<R848_LNA_LOW_LOWEST[R848_TF_BEAD])  //<164
		 R848_Freq_Info.LNA_BAND = 0x60;   // ultra low			// R848:R13[6:5]
	else if((RF_freq>=R848_LNA_LOW_LOWEST[R848_TF_BEAD]) && (RF_freq<
R848_LNA_MID_LOW[R848_TF_BEAD]))  //164~388
		 R848_Freq_Info.LNA_BAND = 0x40;   //low					// R848:R13[6:5]
	else if((RF_freq>=R848_LNA_MID_LOW[R848_TF_BEAD]) && (RF_freq<
R848_LNA_HIGH_MID[R848_TF_BEAD]))  //388~612
		 R848_Freq_Info.LNA_BAND = 0x20;   // mid					// R848:R13[6:5]
	else     // >612
		 R848_Freq_Info.LNA_BAND = 0x00;   // high				// R848:R13[6:5]
	
	//----- LO dependent parameter --------
	//IMR point 
	if(LO_freq<133000)  
         R848_Freq_Info.IMR_MEM = 0;   
	else if((LO_freq>=133000) && (LO_freq<221000))  
         R848_Freq_Info.IMR_MEM = 1;   
	else if((LO_freq>=221000) && (LO_freq<450000))  
		 R848_Freq_Info.IMR_MEM = 2;  
	else if((LO_freq>=450000) && (LO_freq<775000))  
		 R848_Freq_Info.IMR_MEM = 3; 
	else 
		 R848_Freq_Info.IMR_MEM = 4; 

	//RF polyfilter band   R33[7:6]
	if(LO_freq<133000)  
         R848_Freq_Info.RF_POLY = 0x80;   //low	
	else if((LO_freq>=133000) && (LO_freq<221000))  
         R848_Freq_Info.RF_POLY = 0x40;   // mid
	else if((LO_freq>=221000) && (LO_freq<775000))  
		 R848_Freq_Info.RF_POLY = 0x00;   // highest
	else
		 R848_Freq_Info.RF_POLY = 0xC0;   // ultra high

	
	//LPF Cap, Notch
	switch(R848_Standard)
	{
		case R848_DVB_C_8M:
		case R848_DVB_C_6M:
		case R848_J83B:
        case R848_DVB_C_8M_IF_5M:
		case R848_DVB_C_6M_IF_5M:
		case R848_J83B_IF_5M:
			if( LO_freq<77000)  
			{
				R848_Freq_Info.LPF_CAP = 16;
				R848_Freq_Info.LPF_NOTCH = 10;
			}
			else if((LO_freq>=77000) && (LO_freq<85000))
			{
				R848_Freq_Info.LPF_CAP = 16;
				R848_Freq_Info.LPF_NOTCH = 4;
			}
			else if((LO_freq>=85000) && (LO_freq<115000))
			{
				R848_Freq_Info.LPF_CAP = 13;
				R848_Freq_Info.LPF_NOTCH = 3;
			}
			else if((LO_freq>=115000) && (LO_freq<125000))
			{
				R848_Freq_Info.LPF_CAP = 11;
				R848_Freq_Info.LPF_NOTCH = 1;
			}
			else if((LO_freq>=125000) && (LO_freq<141000))
			{
				R848_Freq_Info.LPF_CAP = 9;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=141000) && (LO_freq<157000))
			{
				R848_Freq_Info.LPF_CAP = 8;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=157000) && (LO_freq<181000))
			{
				R848_Freq_Info.LPF_CAP = 6;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=181000) && (LO_freq<205000))
			{
				R848_Freq_Info.LPF_CAP = 3;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else //LO>=201M
			{
				R848_Freq_Info.LPF_CAP = 0;
				R848_Freq_Info.LPF_NOTCH = 0;
			}

			break;

		default:
			if(LO_freq<73000)  
			{
				R848_Freq_Info.LPF_CAP = 8;
				R848_Freq_Info.LPF_NOTCH = 10;
			}
			else if((LO_freq>=73000) && (LO_freq<81000))
			{
				R848_Freq_Info.LPF_CAP = 8;
				R848_Freq_Info.LPF_NOTCH = 4;
			}
			else if((LO_freq>=81000) && (LO_freq<89000))
			{
				R848_Freq_Info.LPF_CAP = 8;
				R848_Freq_Info.LPF_NOTCH = 3;
			}
			else if((LO_freq>=89000) && (LO_freq<121000))
			{
				R848_Freq_Info.LPF_CAP = 6;
				R848_Freq_Info.LPF_NOTCH = 1;
			}
			else if((LO_freq>=121000) && (LO_freq<145000))
			{
				R848_Freq_Info.LPF_CAP = 4;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=145000) && (LO_freq<153000))
			{
				R848_Freq_Info.LPF_CAP = 3;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=153000) && (LO_freq<177000))
			{
				R848_Freq_Info.LPF_CAP = 2;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=177000) && (LO_freq<201000))
			{
				R848_Freq_Info.LPF_CAP = 1;
				R848_Freq_Info.LPF_NOTCH = 0;
			}
			else //LO>=201M
			{
				R848_Freq_Info.LPF_CAP = 0;
				R848_Freq_Info.LPF_NOTCH = 0;
			}

			break;

	}//end switch(standard)

	return R848_Freq_Info;

}

int r848_cal_prepare(struct r848_priv *priv, uint8_t u1CalFlag)
{
	int rc = 0;
	R848_Cal_Info_Type  Cal_Info;
	uint8_t R848_IMR_Cal_Type = R848_IMR_CAL;

	switch(u1CalFlag)
	{
	    case R848_IMR_CAL:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB		 R848:R38[3]
				//Cal_Info.RFBUF_OUT = 0x60;            //from RF_Buf ON, RF_Buf pwr off		 // R848:R12[5]
				//from RF_Buf ON, RF_Buf pwr off
				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF,RF_Buf pwr off  //  R848:R8[7]
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x00;					// TF cal OFF, -6dB	OFF   // R848:R14[6:5]
				Cal_Info.MIXER_AMP_GAIN = 0x08;			//manual +8				  // R848:R15[4:0]
				Cal_Info.MIXER_BUFFER_GAIN = 0x10;		//manual min(0)			  // R848:R34[4:0]
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max		//  R848:R13[7:0]
				//Cal_Info.LNA_GAIN = 0x80;
				R848_IMR_Cal_Type = R848_IMR_CAL;
			break;
		case R848_IMR_LNA_CAL:						    
				Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x00;              //from RF_Buf ON, RF_Buf pwr on

				Cal_Info.RFBUF_OUT = 0x00;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x00;				//RF_BUF_pwr OFF

				Cal_Info.LNA_POWER = 0x80;             // LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x60;				   // TF cal ON, -6dB ON
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max
				//Cal_Info.LNA_GAIN = 0x80;
				R848_IMR_Cal_Type = R848_IMR_LNA_CAL;
			break;
        case R848_TF_CAL: //TBD
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x60;               //from RF_Buf ON, RF_Buf pwr off

				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF

				Cal_Info.RFBUF_OUT = 0x20;
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x00;					//TF cal OFF, -6dB OFF	
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x9F;                  //manual: max
			break;
        case R848_TF_LNA_CAL:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB				
				//Cal_Info.RFBUF_OUT = 0x00;              //from RF_Buf ON, RF_Buf pwr on	

				Cal_Info.RFBUF_OUT = 0x00;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x00;				//RF_BUF_pwr OFF

				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x60;					// TF cal ON, -6dB ON	
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x80;                  //manual: min
			break;
		case R848_LPF_CAL: 
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB						//  R848:R38[3]
				//Cal_Info.RFBUF_OUT = 0x60;               //from RF_Buf ON, RF_Buf pwr off
				//Cal_Info.RFBUF_OUT = 0x20;				//RF_Buf pwr off			//  R848:R12[5]
				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF

				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF, TF cal OFF, -6dB OFF	
				//Cal_Info.LNA_POWER = 0x00;              //LNA need on			   //  R848:R8[7]
				Cal_Info.TF_CAL = 0x00;					// TF cal OFF, -6dB OFF		// R848:R14[6:5]
				Cal_Info.MIXER_AMP_GAIN = 0x08;    //manual +8						// R848:R15[4:0]
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)					// R848:R34[4:0]	
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max				// R848:R13[7:0]
				R848_IMR_Cal_Type = R848_LPF_CAL;
			break;
		case R848_LPF_LNA_CAL:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x00;               //from RF_Buf ON, RF_Buf pwr on
				Cal_Info.RFBUF_OUT = 0x00;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x00;				//RF_BUF_pwr OFF
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;              //LNA need on
				Cal_Info.TF_CAL = 0x20;					// TF cal ON, -6dB OFF	
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x80;                  //manual: min
			break;
		default:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x60;               //from RF_Buf ON, RF_Buf pwr off
				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;              //LNA need on
				Cal_Info.TF_CAL = 0x00;					//TF cal OFF, -6dB OFF
				Cal_Info.MIXER_AMP_GAIN = 0x08;    //manual +8
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max
	 }


	//Ring From RF_Buf Output & RF_Buf Power
	//R848_I2C.RegAddr = 0x0C;
	priv->cfg->R848_Array[4] = (priv->cfg->R848_Array[4] & 0xDF) | Cal_Info.RFBUF_OUT;   //  R848:R12[5]  12-8=4  12(0x0C) is addr ; [4] is data
	rc = r848_write_reg(priv, 0x0c, priv->cfg->R848_Array[4]);


	  //RF_Buf Power
	  //R848_I2C.RegAddr = 0x09;
      priv->cfg->R848_Array[1] = (priv->cfg->R848_Array[1] & 0xFB) | Cal_Info.RFBUF_POWER;
	rc = r848_write_reg(priv, 0x09, priv->cfg->R848_Array[1]);
  
	  //TF cal (LNA power ON/OFF , TF cal ON/OFF, TF_-6dB ON/OFF)
	  //R848_I2C.RegAddr = 0x06;
      //priv->cfg->R848_Array[6] = (priv->cfg->R848_Array[6] & 0x1F) | Cal_Info.LNA_POWER;
      //R848_I2C.Data = priv->cfg->R848_Array[6];
	//rc = r848_write_reg(priv, 0x06, priv->cfg->R848_Array[6]);

	  //(LNA power ON/OFF )
	  //R848_I2C.RegAddr = 0x08;
      priv->cfg->R848_Array[0] = (priv->cfg->R848_Array[0] & 0x7F) | Cal_Info.LNA_POWER;	 //  R848:R8[7]  8-8=0  8(0x08) is addr ; [0] is data
	  //priv->cfg->R848_Array[0] = (priv->cfg->R848_Array[0] & 0x80) 	 // R848:R8[7]  8-8=0  14(0x08) is addr ; [0] is data
	rc = r848_write_reg(priv, 0x08, priv->cfg->R848_Array[0]);


	  //TF cal (TF cal ON/OFF, TF_-6dB ON/OFF)
	//  R848_I2C.RegAddr = 0x0E;
      priv->cfg->R848_Array[6] = (priv->cfg->R848_Array[6] & 0x9F) | Cal_Info.TF_CAL;	 // R848:R14[6:5]  14-8=6  14(0x0E) is addr ; [6] is data
	rc = r848_write_reg(priv, 0x0e, priv->cfg->R848_Array[6]);


	  //LNA gain
	//  R848_I2C.RegAddr = 0x0D;
	  priv->cfg->R848_Array[5] = (priv->cfg->R848_Array[5] & 0x60) | Cal_Info.LNA_GAIN; // R848:R13[7:0]  13-8=5  13(0x0D) is addr ; [5] is data
	rc = r848_write_reg(priv, 0x0d, priv->cfg->R848_Array[5]);


	  //Mixer Amp Gain
	  //R848_I2C.RegAddr = 0x0F;
	  priv->cfg->R848_Array[7] = (priv->cfg->R848_Array[7] & 0xE0) | Cal_Info.MIXER_AMP_GAIN; // R848:R15[4:0]  15-8=7  15(0x0F) is addr ; [7] is data
	rc = r848_write_reg(priv, 0x0f, priv->cfg->R848_Array[7]);


	  //Mixer Buffer Gain
	  //R848_I2C.RegAddr = 0x22;								// R848:R34[4:0]  34-8=26  34(0x22) is addr ; [26] is data
	  priv->cfg->R848_Array[26] = (priv->cfg->R848_Array[26] & 0xE0) | Cal_Info.MIXER_BUFFER_GAIN;  
	rc = r848_write_reg(priv, 0x22, priv->cfg->R848_Array[26]);

	  // Set filter +0/6dB; NA det=OFF 
      //R848_I2C.RegAddr  = 0x26;								// R848:R38[3]  38-8=30  38(0x26) is addr ; [30] is data
	  priv->cfg->R848_Array[30] = (priv->cfg->R848_Array[30] & 0xF7) | Cal_Info.FILTER_6DB | 0x80;
	rc = r848_write_reg(priv, 0x26, priv->cfg->R848_Array[30]);

	  //Set NA det 710 = OFF
	//  R848_I2C.RegAddr  = 0x28;								// R848:R40[3]  40-8=32  40(0x28) is addr ; [32] is data
	  priv->cfg->R848_Array[32] = (priv->cfg->R848_Array[32] | 0x08);
	rc = r848_write_reg(priv, 0x28, priv->cfg->R848_Array[32]);


	 //---- General calibration setting ----//	 
	 //IMR IQ cap=0
//	 R848_I2C.RegAddr = 0x0B;		//R848:R11[1:0]  11-8=3  11(0x0B) is addr ; [3] is data
     priv->cfg->R848_Array[3] = (priv->cfg->R848_Array[3] & 0xFC);
	rc = r848_write_reg(priv, 0x0b, priv->cfg->R848_Array[3]);

	 // Set RF_Flag ON(%)
//	 R848_I2C.RegAddr = 0x16;		//R848:R22[0]  22-8=14  22(0x16) is addr ; [14] is data
     priv->cfg->R848_Array[14] = priv->cfg->R848_Array[14] | 0x01;  //force far-end mode
//     R848_I2C.Data = priv->cfg->R848_Array[14];
	rc = r848_write_reg(priv, 0x16, priv->cfg->R848_Array[14]);

	 //RingPLL power ON
//     R848_I2C.RegAddr = 0x12;	  //R848:R18[4]  18-8=10  18(0x12) is addr ; [10] is data
     priv->cfg->R848_Array[10] = (priv->cfg->R848_Array[10] & 0xEF);
//     R848_I2C.Data = priv->cfg->R848_Array[10];
	rc = r848_write_reg(priv, 0x12, priv->cfg->R848_Array[10]);

	 //LPF filter code = 15
//     R848_I2C.RegAddr = 0x12;	//R848:R18[3:0]  18-8=10  18(0x12) is addr ; [10] is data
     priv->cfg->R848_Array[10] = (priv->cfg->R848_Array[10] & 0xF0) | 0x0F;
//     R848_I2C.Data = priv->cfg->R848_Array[10];
	rc = r848_write_reg(priv, 0x12, priv->cfg->R848_Array[10]);
	 
     //HPF corner=narrowest; LPF coarse=6M; 1.7M disable
//     R848_I2C.RegAddr = 0x13;	//R848:R19[7:0]  19-8=11  19(0x13) is addr ; [11] is data
     priv->cfg->R848_Array[11] = (priv->cfg->R848_Array[11] & 0x00) | 0x60;
//     R848_I2C.Data = priv->cfg->R848_Array[11];
	rc = r848_write_reg(priv, 0x13, priv->cfg->R848_Array[11]);

     //ADC/VGA PWR on; Vga code mode(b4=1), Gain = 26.5dB; Large Code mode Gain(b5=1)
	 //ADC PWR on (b7=0)
//	 R848_I2C.RegAddr = 0x0F;	//R848:R15[7]  15-8=7  15(0x0F) is addr ; [7] is data
     priv->cfg->R848_Array[7] = (priv->cfg->R848_Array[7] & 0x7F);
//     R848_I2C.Data = priv->cfg->R848_Array[7];
	rc = r848_write_reg(priv, 0x0f, priv->cfg->R848_Array[7]);

	 //VGA PWR on (b0=0)
	 //R848_I2C.RegAddr = 0x09;	// R848:R9[0]  9-8=1  9(0x09) is addr ; [1] is data
     //priv->cfg->R848_Array[1] = (priv->cfg->R848_Array[1] & 0xFE);  
     //R848_I2C.Data = priv->cfg->R848_Array[1];

	 //VGA PWR on (b0=0)  MT2
//	 R848_I2C.RegAddr = 0x12;	//R848:R18[7]  9-8=1  9(0x09) is addr ; [1] is data
     priv->cfg->R848_Array[10] = (priv->cfg->R848_Array[10] & 0x7F);  
	rc = r848_write_reg(priv, 0x12, priv->cfg->R848_Array[10]);

	 //Large Code mode Gain(b5=1)
//	 R848_I2C.RegAddr = 0x0B;	//R848:R11[3]  11-8=3  11(0x0B) is addr ; [3] is data
     priv->cfg->R848_Array[3] = (priv->cfg->R848_Array[3] & 0xF7) | 0x08;  
//     R848_I2C.Data = priv->cfg->R848_Array[3];
	rc = r848_write_reg(priv, 0x0b, priv->cfg->R848_Array[3]);

	 //Vga code mode(b4=1)
//	 R848_I2C.RegAddr = 0x09;	//R848:R9[1]  9-8=1  9(0x09) is addr ; [1] is data
     priv->cfg->R848_Array[1] = (priv->cfg->R848_Array[1] & 0xFD) | 0x02;  
	rc = r848_write_reg(priv, 0x09, priv->cfg->R848_Array[1]);

	 //Gain = 26.5dB
//     R848_I2C.RegAddr = 0x14;	//R848:R20[3:0]  20-8=12  20(0x14) is addr ; [12] is data
     priv->cfg->R848_Array[12] = (priv->cfg->R848_Array[12] & 0xF0) | 0x0B;  
	rc = r848_write_reg(priv, 0x14, priv->cfg->R848_Array[12]);


	//LNA, RF, Nrb dector pw on; det2 cap=IF_det 
	//R848_I2C.RegAddr = 0x25;	
	//R848:R37[3:0]  37-8=29  37(0x25) is addr ; [29] is data
     	priv->cfg->R848_Array[29] = (priv->cfg->R848_Array[29] & 0xF0) | 0x02;
	rc = r848_write_reg(priv, 0x25, priv->cfg->R848_Array[29]);

      return rc;

}


int r848_tf_check(struct r848_priv *priv)
{
	uint32_t   RingVCO = 0;
	uint32_t   RingFreq = 72000;
	uint32_t   RingRef = R848_XTAL;
	uint8_t     divnum_ring = 0;
	 uint8_t   VGA_Count = 0;
	 uint8_t   VGA_Read = 0;
	uint8_t	   rc;

	if(R848_XTAL==16000000)  //16M
	{
         divnum_ring = 11;
	}
	else  //24M
	{
		 divnum_ring = 2;
	}
	RingVCO = (16+divnum_ring)* 8 * RingRef;
	RingFreq = RingVCO/48;


	rc = r848_cal_prepare(priv,R848_TF_LNA_CAL);
	
	priv->cfg->R848_Array[31] = (priv->cfg->R848_Array[31] & 0x03) | 0x80 | (divnum_ring<<2);  //pre div=6 & div_num
	r848_write_reg(priv, 0x27, priv->cfg->R848_Array[31]);


	 //Ring PLL PW on
	priv->cfg->R848_Array[18] = (priv->cfg->R848_Array[18] & 0xEF); 
	r848_write_reg(priv, 0x12, priv->cfg->R848_Array[18]);


	 //NAT Det Sour : Mixer buf out
	priv->cfg->R848_Array[37] = (priv->cfg->R848_Array[37] & 0x7F); 
	r848_write_reg(priv, 0x24, priv->cfg->R848_Array[37]);


	 //R848 R33[7:0]   

	 priv->cfg->R848_Array[25] = (priv->cfg->R848_Array[25] & 0x00) | 0x8B;  //out div=8, RF poly=low band, power=min_lp
	 if(RingVCO>=3200000)
	    priv->cfg->R848_Array[25] = (priv->cfg->R848_Array[25] & 0xDF);      //vco_band=high, R25[5]=0
	 else
        priv->cfg->R848_Array[25] = (priv->cfg->R848_Array[25] | 0x20);      //vco_band=low, R25[5]=1
        r848_write_reg(priv, 0x21, priv->cfg->R848_Array[25]);

     //Must do before PLL()
	if(R848_MUX(priv,RingFreq + 5000, RingFreq, R848_STD_SIZE) != RT_Success)     //FilCal MUX (LO_Freq, RF_Freq)
	     return RT_Fail;

	 //Set PLL
	 if(r848_set_freq(priv,(RingFreq + 5000)) != RT_Success)   //FilCal PLL
	       return RT_Fail;


	//Set LNA TF=(1,15),for VGA training   // R848 R8[6:0]
     priv->cfg->R848_Array[0] = (priv->cfg->R848_Array[0] & 0x80) | 0x1F;  	
        r848_write_reg(priv, 0x08, priv->cfg->R848_Array[0]);

	//Qctrl=off  // R848 R14[5] 
     priv->cfg->R848_Array[6] = (priv->cfg->R848_Array[6] & 0xDF);  	
        r848_write_reg(priv, 0x0e, priv->cfg->R848_Array[6]);

	// FilterComp OFF  // R848 R14[2]  
     priv->cfg->R848_Array[6] = (priv->cfg->R848_Array[6] & 0xFB);  	
        r848_write_reg(priv, 0x0e, priv->cfg->R848_Array[6]);

	// Byp-LPF: Bypass  R848 R12[6]  12-8=4  12(0x0C) is addr ; [4] is data
     priv->cfg->R848_Array[4] = priv->cfg->R848_Array[4] & 0xBF;  	
        r848_write_reg(priv, 0x0c, priv->cfg->R848_Array[4]);

	 //Adc=on; Vga code mode, Gain = -12dB 
	 //R848 R20[3:0]         set 0
	 //R848 R9[1]            set 1
	 //R848 R11[3]           set 0
	 //R848 R18[7]           set 0
	 //R848 R15[7]           set 0
	 
	 // VGA Gain = -12dB 
     priv->cfg->R848_Array[12] = (priv->cfg->R848_Array[12] & 0xF0);
        r848_write_reg(priv, 0x14, priv->cfg->R848_Array[12]);

	// Vga code mode
     priv->cfg->R848_Array[1] = (priv->cfg->R848_Array[1] | 0x02);
        r848_write_reg(priv, 0x09, priv->cfg->R848_Array[1]);

	// VGA 6dB
     priv->cfg->R848_Array[3] = (priv->cfg->R848_Array[3] & 0xF7);
        r848_write_reg(priv, 0x08, priv->cfg->R848_Array[3]);

	// VGA PW ON
     priv->cfg->R848_Array[10] = (priv->cfg->R848_Array[10] & 0x7F);
        r848_write_reg(priv, 0x12, priv->cfg->R848_Array[10]);

	 // Adc on
     priv->cfg->R848_Array[7] = (priv->cfg->R848_Array[7] & 0x7F);
        r848_write_reg(priv, 0x0f, priv->cfg->R848_Array[7]);



	 //------- increase VGA power to let ADC read value significant ---------//


	 for(VGA_Count=0; VGA_Count < 16; VGA_Count ++)
	 {
        	r848_write_reg(priv, 0x14, (priv->cfg->R848_Array[12] & 0xF0) + VGA_Count); 

		usleep(10); //
		
		if(R848_Muti_Read(priv,&VGA_Read) != RT_Success)
			return RT_Fail;

		if(VGA_Read > 40)
			break;
	 }

	 //Set LNA TF=(0,0)
     priv->cfg->R848_Array[0] =(priv->cfg->R848_Array[0] & 0x80);  	
        r848_write_reg(priv, 0x08, priv->cfg->R848_Array[0]);

	 usleep(10); //

	 if(R848_Muti_Read(priv,&VGA_Read) != RT_Success)
		  return RT_Fail;

	 if(VGA_Read > (36))
        priv->cfg->R848_DetectTfType = R848_UL_USING_BEAD;
	 else
	    priv->cfg->R848_DetectTfType = R848_UL_USING_270NH;

	return RT_Success;
}

int r848_imr(struct r848_priv *priv, uint8_t IMR_MEM, uint8_t IM_Flag)
{
int rc =0;
return rc;
}

R848_ErrCode R848_Xtal_Check( struct r848_priv *priv)
{


    return RT_Success;
}	

int r848_set_freq(struct r848_priv *priv,uint32_t LO_Freq)
{
	int ret;
	uint8_t buf[3];

	uint8_t  MixDiv = 2;
	uint8_t  DivBuf = 0;
	uint8_t  Ni = 0;
	uint8_t  Si = 0;
	uint8_t  DivNum = 0;
	uint16_t  Nint = 0;
	uint32_t VCO_Min = 2410000; 
	uint32_t VCO_Max = VCO_Min*2;
	uint32_t VCO_Freq = 0;
	uint32_t PLL_Ref	= R848_XTAL;		
	uint32_t VCO_Fra	= 0;		
	uint16_t Nsdm = 2;
	uint16_t SDM = 0;
	uint16_t SDM16to9 = 0;
	uint16_t SDM8to1 = 0;
	uint8_t   CP_CUR = 0x00;
	uint8_t   CP_OFFSET = 0x00;
	uint8_t   SDM_RES = 0x00;
	uint8_t   XTAL_POW1 = 0x00;
	uint8_t   XTAL_POW0 = 0x00;
	uint8_t   XTAL_GM = 0x00;
	uint16_t  u2XalDivJudge;
	uint8_t   u1XtalDivRemain;
	uint8_t   VCO_current_trial = 0;

	uint8_t   u1RfFlag = 0;
	uint8_t   u1PulseFlag = 0;
	uint8_t   u1SPulseFlag=0;

	R848_Standard_Type R848_Standard = priv->type;

	uint8_t	R848_XtalDiv = XTAL_DIV2;

	
	I2C_TYPE  R848_I2C;
	I2C_LEN_TYPE R848_I2C_Len;

	//TF, NA fix
	u1RfFlag = (priv->cfg->R848_Array[14] & 0x01);         //R22[0]
	u1PulseFlag = (priv->cfg->R848_Array[30] & 0x80);   //R38[7]
	u1SPulseFlag= (priv->cfg->R848_Array[32] & 0x08);   //R40[3]


	priv->cfg->R848_Array[14] = priv->cfg->R848_Array[14] | 0x01;		// TF force sharp mode
	r848_write_reg(priv, 0x16, priv->cfg->R848_Array[14]);

	priv->cfg->R848_Array[30] = priv->cfg->R848_Array[30] | 0x80;		// NA det off
	r848_write_reg(priv, 0x26, priv->cfg->R848_Array[30]);

	//Set NA det 710 = OFF
	priv->cfg->R848_Array[32] = (priv->cfg->R848_Array[32] | 0x08);
	r848_write_reg(priv, 0x28, priv->cfg->R848_Array[32]);

	//DTV
	CP_CUR = 0x00;     //0.7m, R25[6:4]=000
	CP_OFFSET = 0x00;  //0u,     [2]=0
	
	//CP current  R25[6:4]=000
	priv->cfg->R848_Array[17]    = (priv->cfg->R848_Array[17] & 0x8F)  | CP_CUR ;
	r848_write_reg(priv, 0x19, priv->cfg->R848_Array[17]);


	//Div Cuurent   R20[7:6]=2'b01(150uA)	
	priv->cfg->R848_Array[12]    = (priv->cfg->R848_Array[12] & 0x3F)  | 0x40;  
	r848_write_reg(priv, 0x14, priv->cfg->R848_Array[12]);


	//CPI*2 R28[7]=1
	if((R848_Standard!=R848_DVB_S) && (LO_Freq >= 865000))
	{
		priv->cfg->R848_Array[20] = (priv->cfg->R848_Array[20] & 0x7F) | 0x80;
		r848_write_reg(priv, 0x1c, priv->cfg->R848_Array[20]);
	}
	else
	{
		priv->cfg->R848_Array[20] = (priv->cfg->R848_Array[20] & 0x7F);
		r848_write_reg(priv, 0x1c, priv->cfg->R848_Array[20]);
	}

	//  R848:R26[7:5]  VCO_current= 2
	priv->cfg->R848_Array[18]    = (priv->cfg->R848_Array[18] & 0x1F) | 0x40; 
	r848_write_reg(priv, 0x1a, priv->cfg->R848_Array[18]);


	//CP Offset R21[7] 
	priv->cfg->R848_Array[13]    = (priv->cfg->R848_Array[13] & 0x7F) | CP_OFFSET; 
	r848_write_reg(priv, 0x15, priv->cfg->R848_Array[13]);

	//if(R848_Initial_done_flag==TRUE)
	{
		//set XTAL Power
		if((priv->cfg->R848_Xtal_Pwr<XTAL_SMALL_HIGH) && (LO_Freq > (64000+8500)))
		{			
				XTAL_POW1 = 0x00;        //high,       R16[4]=0  //R848:R23[7] 
				XTAL_POW0 = 0x20;        //high,       R15[6:5]=01  //R848:R23[6:5] 
				XTAL_GM = 0x00;            //SMALL(0),   R15[4:3]=00		
		}
		else
		{
			if(priv->cfg->R848_Xtal_Pwr <= XTAL_SMALL_HIGHEST)
			{
				XTAL_POW1 = 0x00;        //high,       R16[4]=0	// R848:R23[7]    
				XTAL_POW0 = ((uint8_t)(XTAL_SMALL_HIGHEST-priv->cfg->R848_Xtal_Pwr)<<5);	//R848:R23[6:5]      
				XTAL_GM = 0x00;            //SMALL(0),   R27[0]=0
			}
			else if(priv->cfg->R848_Xtal_Pwr == XTAL_LARGE_HIGHEST)
			{
				XTAL_POW1 = 0x00;        //high,      	// R848:R23[7]  
				XTAL_POW0 = 0x00;        //highest,  	// R848:R23[6:5] 
				XTAL_GM = 0x01;            //LARGE(1),         R27[0]=1
			}
			else
			{
				XTAL_POW1 = 0x00;        //high,     // R848:R23[7]    
				XTAL_POW0 = 0x00;        //highest,  // R848:R23[6:5]
				XTAL_GM = 0x01;            //LARGE(1),         R27[0]=1
			}
		}
	}
//	else
//	{
//		XTAL_POW1 = 0x00;        //high,      	// R848:R23[7]  
//		XTAL_POW0 = 0x00;        //highest,  	// R848:R23[6:5] 
//		XTAL_GM = 0x01;          //LARGE(1),         R27[0]=1
//	}

	//Xtal_Gm=SMALL(0) R27[0]
	priv->cfg->R848_Array[19] = (priv->cfg->R848_Array[19] & 0xFE) | XTAL_GM;
	r848_write_reg(priv, 0x1b, priv->cfg->R848_Array[19]);

	priv->cfg->R848_Array[15]    = (priv->cfg->R848_Array[15] & 0x9F) | XTAL_POW0; 
	r848_write_reg(priv, 0x17, priv->cfg->R848_Array[15]);

	priv->cfg->R848_Array[15]    = (priv->cfg->R848_Array[15] & 0x7F) | XTAL_POW1; 
	r848_write_reg(priv, 0x17, priv->cfg->R848_Array[15]);

	//IQ gen ON 
	priv->cfg->R848_Array[31]    = (priv->cfg->R848_Array[31] & 0xFD) | 0x00; //[0]=0'b0
	r848_write_reg(priv, 0x27, priv->cfg->R848_Array[31]);

	// current:Dmin, Bmin
	priv->cfg->R848_Array[27]    = (priv->cfg->R848_Array[27] & 0xCF) | 0x00;
	r848_write_reg(priv, 0x23, priv->cfg->R848_Array[27]);

	//set pll autotune = 128kHz (fast)  R23[4:3]=2'b00   
	priv->cfg->R848_Array[15]    = priv->cfg->R848_Array[15] & 0xE7;
	r848_write_reg(priv, 0x17, priv->cfg->R848_Array[15]);

	//Divider
	while(MixDiv <= 64)
	{
		if(((LO_Freq * MixDiv) >= VCO_Min) && ((LO_Freq * MixDiv) < VCO_Max))
		{
			DivBuf = MixDiv;
			while(DivBuf > 2)
			{
				DivBuf = DivBuf >> 1;
				DivNum ++;
			}			
			break;
		}
		MixDiv = MixDiv << 1;
	}

	SDM_RES = 0x00;    //short, R27[4:3]=00
	priv->cfg->R848_Array[19]    = (priv->cfg->R848_Array[19] & 0xE7) | SDM_RES; 
	r848_write_reg(priv, 0x1b, priv->cfg->R848_Array[19]);

	//Xtal Div
	if(R848_Standard == R848_STD_SIZE) //for cal
	{
		    R848_XtalDiv = XTAL_DIV1;
	        priv->cfg->R848_Array[16] = priv->cfg->R848_Array[16] & 0xFB; //b2=0  // R848:R24[2]   
	        PLL_Ref = R848_XTAL;
	}
	else	//DTV_Standard
	{
		    u2XalDivJudge = (uint16_t) (LO_Freq/1000/8);
			u1XtalDivRemain = (uint8_t) (u2XalDivJudge % 2);
		   if(u1XtalDivRemain==1) //odd
		   {
				R848_XtalDiv = XTAL_DIV1;
				priv->cfg->R848_Array[16] = priv->cfg->R848_Array[16] & 0xFB; //R24[2]=0  
				PLL_Ref = R848_XTAL;
			}
			else  // div2, note that agc clk also div2
			{

			   R848_XtalDiv = XTAL_DIV2;
				priv->cfg->R848_Array[16] |= 0x04;   	//R24[2]=1
				PLL_Ref = R848_XTAL / 2;
				
			}	
	}

	r848_write_reg(priv, 0x18, priv->cfg->R848_Array[16]);



	//Divider num
	priv->cfg->R848_Array[16] &= 0x1F;
	priv->cfg->R848_Array[16] |= (DivNum << 5);
	r848_write_reg(priv, 0x18, priv->cfg->R848_Array[16]);

	VCO_Freq = LO_Freq * MixDiv;
	Nint     = (uint16_t) (VCO_Freq / 2 / PLL_Ref);
	VCO_Fra  = (uint16_t) (VCO_Freq - 2 * PLL_Ref * Nint);

	//Boundary spur prevention
	if (VCO_Fra < PLL_Ref/64)           //2*PLL_Ref/128
		VCO_Fra = 0;
	else if (VCO_Fra > PLL_Ref*127/64)  //2*PLL_Ref*127/128
	{
		VCO_Fra = 0;
		Nint ++;
	}
	else if((VCO_Fra > PLL_Ref*127/128) && (VCO_Fra < PLL_Ref)) //> 2*PLL_Ref*127/256,  < 2*PLL_Ref*128/256
		VCO_Fra = PLL_Ref*127/128;      // VCO_Fra = 2*PLL_Ref*127/256
	else if((VCO_Fra > PLL_Ref) && (VCO_Fra < PLL_Ref*129/128)) //> 2*PLL_Ref*128/256,  < 2*PLL_Ref*129/256
		VCO_Fra = PLL_Ref*129/128;      // VCO_Fra = 2*PLL_Ref*129/256
	else
		VCO_Fra = VCO_Fra;

	//Ni:R848:R28[6:0]   Si:R848:R20[5:4]
	Ni = (Nint - 13) / 4;
	Si = Nint - 4 *Ni - 13;
	//Si
	priv->cfg->R848_Array[12] = (priv->cfg->R848_Array[12] & 0xCF) | ((Si << 4));
	r848_write_reg(priv, 0x14, priv->cfg->R848_Array[12]);
	//Ni
	priv->cfg->R848_Array[20] = (priv->cfg->R848_Array[20] & 0x80) | (Ni);
	r848_write_reg(priv, 0x1c, priv->cfg->R848_Array[20]);

         	
	//pw_sdm		// R848:R27[7]  
	priv->cfg->R848_Array[19] &= 0x7F;
	if(VCO_Fra == 0)
		priv->cfg->R848_Array[19] |= 0x80;
	r848_write_reg(priv, 0x1b, priv->cfg->R848_Array[19]);

	//SDM calculator
	while(VCO_Fra > 1)
	{			
		if (VCO_Fra > (2*PLL_Ref / Nsdm))
		{		
			SDM = SDM + 32768 / (Nsdm/2);
			VCO_Fra = VCO_Fra - 2*PLL_Ref / Nsdm;
			if (Nsdm >= 0x8000)
				break;
		}
		Nsdm = Nsdm << 1;
	}

	SDM16to9 = SDM >> 8;
	SDM8to1 =  SDM - (SDM16to9 << 8);

	// R848:R30[7:0]  
	priv->cfg->R848_Array[22]    = (uint8_t) SDM16to9;
	r848_write_reg(priv, 0x1e, priv->cfg->R848_Array[22]);
	//R848:R29[7:0] 
	priv->cfg->R848_Array[21]    = (uint8_t) SDM8to1;
	r848_write_reg(priv, 0x1d, priv->cfg->R848_Array[21]);

	//if(R848_Standard <= R848_SECAM_L1_INV)
	if(R848_XtalDiv == XTAL_DIV2)
	    usleep(20);
	else
	    usleep(10);

	for(VCO_current_trial=0; VCO_current_trial<3; VCO_current_trial++)
	{
		//check PLL lock status 
//		R848_I2C_Len.RegAddr = 0x00;
//		R848_I2C_Len.Len = 3;
//		if(I2C_Read_Len(priv,&R848_I2C_Len) != RT_Success)
//		{	
//			I2C_Read_Len(priv,&R848_I2C_Len);
//		}
                ret = r848_read(priv , 0x00 , buf , 3);

		// R848:R26[7:5] 
		if( (buf[2] & 0x40) == 0x00 ) 
		{
			//Set VCO current = 011 (3)
			//priv->cfg->R848_Array[18]    = (priv->cfg->R848_Array[18] & 0x1F) | 0x60;  //increase VCO current
			priv->cfg->R848_Array[18]    = (priv->cfg->R848_Array[18] & 0x1F) | ((2-VCO_current_trial) << 5);  //increase VCO current
			r848_write_reg(priv, 0x1a, priv->cfg->R848_Array[18]);
		}
	}

	if(VCO_current_trial==2)
	{
		//check PLL lock status 
//		R848_I2C_Len.RegAddr = 0x00;
//		R848_I2C_Len.Len = 3;
//		if(I2C_Read_Len(priv,&R848_I2C_Len) != RT_Success)
//			return RT_Fail;
                ret = r848_read(priv , 0x00 , buf , 3);

		if( (buf[2] & 0x40) == 0x00) 
		{
			if(priv->cfg->R848_Xtal_Pwr <= XTAL_SMALL_HIGHEST)
				XTAL_GM = 0x01;            //LARGE(1),         R15[4:3]=11

			priv->cfg->R848_Array[19] = (priv->cfg->R848_Array[19] & 0xFE) | XTAL_GM;
			r848_write_reg(priv, 0x1b, priv->cfg->R848_Array[19]);
		}
	}

	//set pll autotune = 8kHz (slow)
	priv->cfg->R848_Array[15] = (priv->cfg->R848_Array[15] & 0xE7) | 0x10;
	r848_write_reg(priv, 0x17, priv->cfg->R848_Array[15]);
		

	//restore TF, NA det setting
	priv->cfg->R848_Array[14] = (priv->cfg->R848_Array[14] & 0xFE) | u1RfFlag;     
	r848_write_reg(priv, 0x16, priv->cfg->R848_Array[14]);

	priv->cfg->R848_Array[30] = (priv->cfg->R848_Array[30] & 0x7F) | u1PulseFlag;  
	r848_write_reg(priv, 0x26, priv->cfg->R848_Array[30]);

	//Set NA det 710 = OFF
	priv->cfg->R848_Array[32] = (priv->cfg->R848_Array[32] & 0xF7) | u1SPulseFlag;
	r848_write_reg(priv, 0x28, priv->cfg->R848_Array[32]);

	return RT_Success;

}


R848_ErrCode R848_MUX( struct r848_priv *priv,uint32_t LO_KHz, uint32_t RF_KHz, R848_Standard_Type R848_Standard)
{	
	int ret;

	R848_Freq_Info_Type Freq_Info1;
	uint8_t Reg08_IMR_Gain   = 0;
	uint8_t Reg09_IMR_Phase  = 0;
	uint8_t Reg03_IMR_Iqcap  = 0;
	I2C_TYPE  R848_I2C;
	Freq_Info1 = R848_Freq_Sel(LO_KHz, RF_KHz, R848_Standard);


	// LNA band (depend on RF_KHz)
//	R848_I2C.RegAddr = 0x0D;	// R13[6:5] 
	priv->cfg->R848_Array[5] = (priv->cfg->R848_Array[5] & 0x9F) | Freq_Info1.LNA_BAND;
	r848_write_reg(priv, 0x0d, priv->cfg->R848_Array[5]);
	

// RF Polyfilter
//	R848_I2C.RegAddr = 0x21;	// R33[7:6]  
	priv->cfg->R848_Array[25] = (priv->cfg->R848_Array[25] & 0x3F) | Freq_Info1.RF_POLY;
	r848_write_reg(priv, 0x21, priv->cfg->R848_Array[25]);

	// LNA Cap
//	R848_I2C.RegAddr = 0x09;	// R9[7:3]  
	priv->cfg->R848_Array[1] = (priv->cfg->R848_Array[1] & 0x07) | (Freq_Info1.LPF_CAP<<3);	
	r848_write_reg(priv, 0x09, priv->cfg->R848_Array[1]);

	// LNA Notch
//	R848_I2C.RegAddr = 0x0A;	// R10[4:0] 
	priv->cfg->R848_Array[2] = (priv->cfg->R848_Array[2] & 0xE0) | (Freq_Info1.LPF_NOTCH);	
	r848_write_reg(priv, 0x0a, priv->cfg->R848_Array[2]);

	//Set_IMR

	Reg08_IMR_Gain = R848_IMR_Data[Freq_Info1.IMR_MEM].Gain_X & 0x3F;
	Reg09_IMR_Phase = R848_IMR_Data[Freq_Info1.IMR_MEM].Phase_Y & 0x3F;
	Reg03_IMR_Iqcap = R848_IMR_Data[Freq_Info1.IMR_MEM].Iqcap & 0x03;



//	R848_I2C.RegAddr = 0x10; // R16[5:0]            
	priv->cfg->R848_Array[8] = (priv->cfg->R848_Array[8] & 0xC0) | Reg08_IMR_Gain;
	r848_write_reg(priv, 0x10, priv->cfg->R848_Array[8]);

//	R848_I2C.RegAddr = 0x11; // R17[5:0]  
	priv->cfg->R848_Array[9] = (priv->cfg->R848_Array[9] & 0xC0) | Reg09_IMR_Phase;
	r848_write_reg(priv, 0x11, priv->cfg->R848_Array[9]);

//	R848_I2C.RegAddr = 0x0B; // R11[1:0]  
	priv->cfg->R848_Array[3] = (priv->cfg->R848_Array[3] & 0xFC) | Reg03_IMR_Iqcap;
	r848_write_reg(priv, 0x0b, priv->cfg->R848_Array[3]);

	return RT_Success;
}




//-----------------------------------------------------------------------------------/ 
// Purpose: compare IMC result aray [0][1][2], find min value and store to CorArry[0]
// input: CorArry: three IMR data array
// output: TRUE or FALSE
//-----------------------------------------------------------------------------------/
R848_ErrCode R848_CompreCor( struct r848_priv *priv,R848_SectType* CorArry)
{
	return RT_Success;
}



R848_ErrCode R848_IQ_Tree( struct r848_priv *priv,uint8_t FixPot, uint8_t FlucPot, uint8_t PotReg, R848_SectType* CompareTree)
{
	return RT_Success;
}


R848_ErrCode R848_Section( struct r848_priv *priv,R848_SectType* IQ_Pont)
{
	return RT_Success;
}

R848_ErrCode R848_IMR_Cross( struct r848_priv *priv,R848_SectType* IQ_Pont, uint8_t* X_Direct)
{
	int ret;
	return RT_Success;
}

//-------------------------------------------------------------------------------------//
// Purpose: if (Gain<9 or Phase<9), Gain+1 or Phase+1 and compare with min value
//          new < min => update to min and continue
//          new > min => Exit
// input: StepArry: three IMR data array
//        Pace: gain or phase register
// output: TRUE or FALSE 
//-------------------------------------------------------------------------------------//
R848_ErrCode R848_CompreStep( struct r848_priv *priv,R848_SectType* StepArry, uint8_t Pace)
{
	return RT_Success;
}

R848_ErrCode R848_IMR_Iqcap( struct r848_priv *priv,R848_SectType* IQ_Point)   
{
    return RT_Success;
}


R848_ErrCode R848_IQ( struct r848_priv *priv,R848_SectType* IQ_Pont)
{
	int ret;
	R848_SectType Compare_IQ[3];
	uint8_t   VGA_Count = 0;
	return RT_Success;
}

//----------------------------------------------------------------------------------------//
// purpose: search surrounding points from previous point 
//          try (x-1), (x), (x+1) columns, and find min IMR result point
// input: IQ_Pont: previous point data(IMR Gain, Phase, ADC Result, RefRreq)
//                 will be updated to final best point                 
// output: TRUE or FALSE
//----------------------------------------------------------------------------------------//
R848_ErrCode R848_F_IMR( struct r848_priv *priv,R848_SectType* IQ_Pont)
{
	return RT_Success;
}


R848_ErrCode R848_SetTF( struct r848_priv *priv,uint32_t u4FreqKHz, uint8_t u1TfType)
{
    return RT_Success;
}

R848_ErrCode R848_IMR( struct r848_priv *priv,uint8_t IMR_MEM, bool IM_Flag)
{
	return RT_Success;
}




uint8_t  R848_Filt_Cal_ADC( struct r848_priv *priv,uint32_t IF_Freq, uint8_t R848_BW, uint8_t FilCal_Gap)
{
	  return 1;

}


R848_ErrCode R848_DVBS_Setting( struct r848_priv *priv,R848_Set_Info R848_INFO)
{
	 uint32_t	LO_KHz;
	 uint8_t fine_tune,Coarse_tune;
	// uint8_t test_coar=0x0D;
	 uint32_t Coarse;
	I2C_TYPE  R848_I2C;

//	if(R848_INFO.R848_Standard != R848_pre_standard)
	{
		r848_write(priv, 0x08, R848_iniArray_dvbs, sizeof(R848_iniArray_dvbs));
	}
    priv->cfg->R848_pre_standard = R848_INFO.R848_Standard;


	 LO_KHz = R848_INFO.RF_KHz;

	if(r848_set_freq(priv,LO_KHz)!= RT_Success)
	{
		return RT_Fail;
	}
	
	//VTH/VTL
	if((R848_INFO.RF_KHz >= 1200000)&&(R848_INFO.RF_KHz <= 1750000))
	{
		priv->cfg->R848_Array[23]=(priv->cfg->R848_Array[23] & 0x00) | 0x93;			//R848:R31[7:0]    1.24/0.64
	}
	else
	{	
		priv->cfg->R848_Array[23]=(priv->cfg->R848_Array[23] & 0x00) | 0x83;			//R848:R31[7:0]   1.14/0.64
	}
	R848_I2C.Data = priv->cfg->R848_Array[23];
        r848_write_reg(priv, 0x1F, priv->cfg->R848_Array[23]);



	if(R848_INFO.RF_KHz >= 2000000) 
	{
		priv->cfg->R848_Array[38]=(priv->cfg->R848_Array[38] & 0xCF) | 0x20;			//R848:R46[4:5]
		r848_write_reg(priv, 0x2e, priv->cfg->R848_Array[38]);
	}


	if((R848_INFO.RF_KHz >= 1600000) && (R848_INFO.RF_KHz <= 1950000))
	{
		priv->cfg->R848_Array[35] |= 0x20; //LNA Mode with att   //R710 R2[6]    R848:R43[5]   43-8=35   43(0x2B) is addr ; [35] is data
		//priv->cfg->R848_Array[36] |= 0x04; //Mixer Buf -3dB		  //R710 R8[7]    R848:R44[2]   44-8=36   44(0x2C) is addr ; [36] is data
	}
	else
	{
		priv->cfg->R848_Array[35] &= 0xDF; //LNA Mode no att
		//priv->cfg->R848_Array[36] &= 0xFB; //Mixer Buf off
	}

        r848_write_reg(priv, 0x2B, priv->cfg->R848_Array[35]);
        r848_write_reg(priv, 0x2C, priv->cfg->R848_Array[36]);




	//Output Signal Mode    (  O is diff  ; 1 is single  )
	if(R848_INFO.R848_DVBS_OutputSignal_Mode != SINGLEOUT)
	{
		priv->cfg->R848_Array[35] &=0x7F;
	}
	else
	{
		priv->cfg->R848_Array[35] |=0x80;  //R710 R11[4]    R848:R43[7]   43-8=35   43(0x2B) is addr ; [35] is data
	}

        r848_write_reg(priv, 0x2B, priv->cfg->R848_Array[35]);



	//AGC Type  //R13[4] Negative=0 ; Positive=1;
	if(R848_INFO.R848_DVBS_AGC_Mode != AGC_POSITIVE)
	{
		priv->cfg->R848_Array[37] &= 0xF7;
	}
	else
	{
		priv->cfg->R848_Array[37] |= 0x08;  //R710 R13[4]    R848:R45[3]   45-8=37   45(0x2D) is addr ; [37] is data
	}
        r848_write_reg(priv, 0x2D, priv->cfg->R848_Array[37]);



	if (R848_INFO.DVBS_BW >67400)
	{
		fine_tune=1;
		Coarse =(( R848_INFO.DVBS_BW -67400) /1600)+31;
		if((( R848_INFO.DVBS_BW -67400) % 1600) > 0)
		Coarse+=1;
	}

	else if((R848_INFO.DVBS_BW >62360)&&(R848_INFO.DVBS_BW<=67400))
	{
		Coarse=31;
		fine_tune=1;		
	}
	else if((R848_INFO.DVBS_BW >38000)&&(R848_INFO.DVBS_BW<=62360))
	{
		fine_tune=1;	
		Coarse =(( R848_INFO.DVBS_BW -38000) /1740)+16;
		if((( R848_INFO.DVBS_BW -38000) % 1740) > 0)
		Coarse+=1;
				
	}
	else if(R848_INFO.DVBS_BW<=5000)
	{	
		Coarse=0;
		fine_tune=0;
	}
	else if((R848_INFO.DVBS_BW>5000) && (R848_INFO.DVBS_BW<=8000))
	{
		Coarse=0;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>8000) && (R848_INFO.DVBS_BW<=10000))
	{
		Coarse=1;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>10000) && (R848_INFO.DVBS_BW<=12000))
	{
		Coarse=2;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>12000) && (R848_INFO.DVBS_BW<=14200))
	{
		Coarse=3;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>14200) && (R848_INFO.DVBS_BW<=16000))
	{
		Coarse=4;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>16000) && (R848_INFO.DVBS_BW<=17800))
	{
		Coarse=5;
		fine_tune=0;
	}
	else if((R848_INFO.DVBS_BW>17800) && (R848_INFO.DVBS_BW<=18600))
	{
		Coarse=5;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>18600) && (R848_INFO.DVBS_BW<=20200))
	{
		Coarse=6;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>20200) && (R848_INFO.DVBS_BW<=22400))
	{
		Coarse=7;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>22400) && (R848_INFO.DVBS_BW<=24600))
	{
		Coarse=9;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>24600) && (R848_INFO.DVBS_BW<=25400))
	{
		Coarse=10;
		fine_tune=0;
	}
	else if((R848_INFO.DVBS_BW>25400) && (R848_INFO.DVBS_BW<=26000))
	{
		Coarse=10;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>26000) && (R848_INFO.DVBS_BW<=27200))
	{
		Coarse=11;
		fine_tune=0;
	}
	else if((R848_INFO.DVBS_BW>27200) && (R848_INFO.DVBS_BW<=27800))
	{
		Coarse=11;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>27800) && (R848_INFO.DVBS_BW<=30200))
	{
		Coarse=12;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>30200) && (R848_INFO.DVBS_BW<=32600))
	{
		Coarse=13;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>32600) && (R848_INFO.DVBS_BW<=33800))
	{
		Coarse=14;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>33800) && (R848_INFO.DVBS_BW<=36800))
	{
		Coarse=15;
		fine_tune=1;
	}
	else if((R848_INFO.DVBS_BW>36800) && (R848_INFO.DVBS_BW<=38000))
	{
		Coarse=16;
		fine_tune=1;
	}

	Coarse_tune = (unsigned char) Coarse;//coras filter value

	//fine tune and coras filter write		
	priv->cfg->R848_Array[39] &= 0x00;		//47-8=39
	priv->cfg->R848_Array[39] = ((priv->cfg->R848_Array[39] | ( fine_tune<< 6 ) ) | ( Coarse_tune));
        r848_write_reg(priv, 0x2F, priv->cfg->R848_Array[39]);

	//Set GPO Low
	priv->cfg->R848_Array[15] = (priv->cfg->R848_Array[15] & 0xFE);
        r848_write_reg(priv, 0x17, priv->cfg->R848_Array[15]);


	return RT_Success;

}




R848_Sys_Info_Type R848_Sys_Sel( struct r848_priv *priv,R848_Standard_Type R848_Standard)
	{
	
		R848_Sys_Info_Type R848_Sys_Info;
	
		return R848_Sys_Info;
	}

static int r848_sleep(struct r848_priv *priv)
{
	return 1;
}

