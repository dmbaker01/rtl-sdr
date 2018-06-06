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

#ifndef R848_H
#define R848_H

#define R848_I2C_ADDR		0xF4
#define R848_XTAL		16000000

#define R848_CHECK_ADDR		0x00
#define R848_CHECK_VAL		0x69 //This value is bit reveresed

#define R848_IF_FREQ		3570000

#define REG_SHADOW_START	5
#define R848_NUM_REGS		40
#define R848_REG_NUM		40
#define NUM_IMR			5
#define IMR_TRIAL		9

#define VER_NUM			49

#define R848_TF_HIGH_NUM	8  
#define R848_TF_MID_NUM		8
#define R848_TF_LOW_NUM		8
#define R848_TF_LOWEST_NUM	8
#define R848_RING_POWER_FREQ_LOW	115000
#define R848_RING_POWER_FREQ_HIGH	450000
#define R848_IMR_IF		5300         
#define R848_IMR_TRIAL		9

enum r848_chip {
        CHIP_R848
};

typedef enum _R848_Standard_Type  //Don't remove standand list!!
{
        R848_DVB_T_6M = 0,
        R848_DVB_T_7M,
        R848_DVB_T_8M,
        R848_DVB_T2_6M,                 //IF=4.57M
        R848_DVB_T2_7M,                 //IF=4.57M
        R848_DVB_T2_8M,                 //IF=4.57M
        R848_DVB_T2_1_7M,
        R848_DVB_T2_10M,
        R848_DVB_C_8M,
        R848_DVB_C_6M,
        R848_J83B,
        R848_ISDB_T,             //IF=4.063M
        R848_ISDB_T_4570,                //IF=4.57M
        R848_DTMB_4570,                  //IF=4.57M
        R848_DTMB_6000,                  //IF=6.00M
        R848_DTMB_6M_BW_IF_5M,   //IF=5.0M, BW=6M
        R848_DTMB_6M_BW_IF_4500, //IF=4.5M, BW=6M
        R848_ATSC,
        R848_DVB_S,
        R848_DVB_T_6M_IF_5M,
        R848_DVB_T_7M_IF_5M,
        R848_DVB_T_8M_IF_5M,
        R848_DVB_T2_6M_IF_5M,
        R848_DVB_T2_7M_IF_5M,
        R848_DVB_T2_8M_IF_5M,
        R848_DVB_T2_1_7M_IF_5M,
        R848_DVB_C_8M_IF_5M,
        R848_DVB_C_6M_IF_5M,
        R848_J83B_IF_5M,
        R848_ISDB_T_IF_5M,
        R848_DTMB_IF_5M,
        R848_ATSC_IF_5M,
        R848_FM,
        R848_STD_SIZE,

}R848_Standard_Type;

enum r848_xtal_cap_value {
	XTAL_CAP
	//XTAL_LOW_CAP_30P = 0,
	//XTAL_LOW_CAP_20P,
	//XTAL_LOW_CAP_10P,
	//XTAL_LOW_CAP_0P,
	//XTAL_HIGH_CAP_0P
};

typedef struct _R848_I2C_LEN_TYPE
{
	uint8_t RegAddr;
	uint8_t Data[50];
	uint8_t Len;
}I2C_LEN_TYPE;

typedef struct _R848_I2C_TYPE
{
	uint8_t RegAddr;
	uint8_t Data;
}I2C_TYPE;

typedef struct _R848_Sys_Info_Type
{
        uint16_t                   IF_KHz;
        uint16_t                   FILT_CAL_IF;
        uint8_t          BW;
        uint8_t            V17M;
        uint8_t            HPF_COR;
        uint8_t          FILT_EXT_ENA;
        uint8_t          FILT_EXT_WIDEST;
        uint8_t          FILT_EXT_POINT;
//      uint8_t          AGC_CLK;
        uint8_t            FILT_COMP;
        uint8_t            FILT_CUR;
        uint8_t            FILT_3DB;
        uint8_t            SWBUF_CUR;
        uint8_t          TF_CUR;
        uint8_t            INDUC_BIAS;
        uint8_t          SWCAP_CLK;
        uint8_t            NA_PWR_DET;
}R848_Sys_Info_Type;

struct r848_config {
	uint8_t i2c_addr;
	uint32_t xtal;
	unsigned int max_i2c_msg_len;
	int use_predetect;
	enum r848_chip rafael_chip;

	/* tuner i2c address */
	uint8_t i2c_address;

	// tuner
	uint8_t R848_DetectTfType ;
	unsigned char R848_pre_standard;
	uint8_t R848_Array[40];
	uint8_t R848_Xtal_Pwr ;
	uint8_t R848_Xtal_Pwr_tmp ;

  /* dvbc/t */
	uint8_t R848_SetTfType;
	R848_Sys_Info_Type Sys_Info1;
  /* DVBT */



};

struct r848_priv {
	struct r848_config		*cfg;

	uint8_t				regs[R848_NUM_REGS];
	uint8_t				buf[R848_NUM_REGS + 1];
	enum r848_xtal_cap_value	xtal_cap_sel;
	uint16_t			pll;	/* kHz */
	uint32_t			int_freq;
	uint8_t				fil_cal_code;
	uint8_t				input;
	int				has_lock;
	int				init_done;
	int				inited;
	int				i2c;

	/* Store current mode */
	uint32_t			delsys;
	R848_Standard_Type		type;

	uint32_t			bw;	/* in MHz */

	void *rtl_dev;
};

struct r848_priv_2 {
	//struct r848_sect_type		imr_data[NUM_IMR];
	uint8_t 			delsys;

	//tune settings /
	uint8_t				standard;
	uint8_t				output_mode;
	uint8_t				agc_mode;

	//struct filter_cal		fc[R848_STD_SIZE];



	//R848_Sys_Info_Type Sys_Info1;
};

struct r848_freq_range {
	uint32_t	freq;
	uint8_t		open_d;
	uint8_t		rf_mux_ploy;
	uint8_t		tf_c;
	uint8_t		xtal_cap20p;
	uint8_t		xtal_cap10p;
	uint8_t		xtal_cap0p;
};


typedef struct _R848_Set_Info
{
	uint32_t                       RF_KHz;
	uint32_t						 DVBS_BW;
	uint8_t           R848_Standard;
	int  R848_DVBS_OutputSignal_Mode;
	int           R848_DVBS_AGC_Mode; 
} R848_Set_Info;

struct filter_cal {
	uint8_t flag;
	uint8_t bw;
	uint8_t code;
};

typedef struct _R848_SectType
{
	uint8_t   Phase_Y;
	uint8_t   Gain_X;
	uint8_t   Iqcap;
	uint8_t   Value;
}R848_SectType;

/*
typedef struct _R848_Sys_Info_Type
{
	uint16_t		   IF_KHz;
	uint16_t		   FILT_CAL_IF;
	uint8_t          BW;
	uint8_t		   V17M; 
	uint8_t		   HPF_COR;
	uint8_t          FILT_EXT_ENA;
	uint8_t          FILT_EXT_WIDEST;
	uint8_t          FILT_EXT_POINT;
//	uint8_t          AGC_CLK;
	uint8_t		   FILT_COMP;
	uint8_t		   FILT_CUR;  
	uint8_t		   FILT_3DB; 
	uint8_t		   SWBUF_CUR;  
	uint8_t          TF_CUR;              
	uint8_t		   INDUC_BIAS;  
	uint8_t          SWCAP_CLK;
	uint8_t		   NA_PWR_DET;  
}R848_Sys_Info_Type;
*/


typedef struct _R848_Freq_Info_Type
{
	uint8_t		RF_POLY;
	uint8_t		LNA_BAND;
	uint8_t		LPF_CAP;
	uint8_t		LPF_NOTCH;
    uint8_t		XTAL_POW0;
	uint8_t		CP_CUR;
	uint8_t		IMR_MEM;
	uint8_t		Q_CTRL;   
}R848_Freq_Info_Type;

typedef struct _R848_SysFreq_Info_Type
{
	uint8_t		LNA_TOP;
	uint8_t		LNA_VTH_L;
	uint8_t		MIXER_TOP;
	uint8_t		MIXER_VTH_L;
	uint8_t       RF_TOP;
	uint8_t       NRB_TOP;
	uint8_t       NRB_BW;
	uint8_t       BYP_LPF;
	uint8_t       RF_FAST_DISCHARGE;
	uint8_t       RF_SLOW_DISCHARGE;
	uint8_t       RFPD_PLUSE_ENA;
	uint8_t       LNA_FAST_DISCHARGE;
	uint8_t       LNA_SLOW_DISCHARGE;
	uint8_t       LNAPD_PLUSE_ENA;
	uint8_t       AGC_CLK;

}R848_SysFreq_Info_Type;

typedef struct _R848_Cal_Info_Type
{
	uint8_t		FILTER_6DB;
	uint8_t		MIXER_AMP_GAIN;
	uint8_t		MIXER_BUFFER_GAIN;
	uint8_t		LNA_GAIN;
	uint8_t		LNA_POWER;
	uint8_t		RFBUF_OUT;
	uint8_t		RFBUF_POWER;
	uint8_t		TF_CAL;
}R848_Cal_Info_Type;

typedef struct _R848_TF_Result
{
	uint8_t   TF_Set;
	uint8_t   TF_Value;
}R848_TF_Result;

typedef enum _R848_TF_Band_Type
{
    TF_HIGH = 0,
	TF_MID,
	TF_LOW
}R848_TF_Band_Type;

typedef enum _R848_TF_Type
{
	R848_TF_NARROW = 0,             //270n/68n   (ISDB-T, DVB-T/T2)
	R848_TF_BEAD,                   //Bead/68n   (DTMB)
	R848_TF_NARROW_LIN,             //270n/68n   (N/A)
	R848_TF_NARROW_ATV_LIN,		//270n/68n   (ATV)
	R848_TF_BEAD_LIN,               //Bead/68n   (PAL_DK for China Hybrid TV)
	R848_TF_NARROW_ATSC,		//270n/68n   (ATSC, DVB-C, J83B)
	R848_TF_BEAD_LIN_ATSC,		//Bead/68n   (ATSC, DVB-C, J83B)
	R848_TF_82N_BEAD,		//Bead/82n   (DTMB)
	R848_TF_82N_270N,		//270n/82n   (OTHER Standard)
	R848_TF_SIZE
} R848_TF_Type;


typedef enum _R848_UL_TF_Type
{
	R848_UL_USING_BEAD = 0,            
    R848_UL_USING_270NH,                      
}R848_UL_TF_Type;



typedef enum _R848_Cal_Type
{
	R848_IMR_CAL = 0,
	R848_IMR_LNA_CAL,
	R848_TF_CAL,
	R848_TF_LNA_CAL,
	R848_LPF_CAL,
	R848_LPF_LNA_CAL
}R848_Cal_Type;

typedef enum _R848_BW_Type
{
	BW_6M = 0,
	BW_7M,
	BW_8M,
	BW_1_7M,
	BW_10M,
	BW_200K
}R848_BW_Type;


enum R848_XTAL_PWR_VALUE {
	XTAL_SMALL_LOWEST = 0,
	XTAL_SMALL_LOW,
	XTAL_SMALL_HIGH,
	XTAL_SMALL_HIGHEST,
	XTAL_LARGE_HIGHEST,
	XTAL_CHECK_SIZE
};


typedef enum _R848_Xtal_Div_TYPE
{
	XTAL_DIV1 = 0,
	XTAL_DIV2
}R848_Xtal_Div_TYPE;


//----------------------------------------------------------//
//                   R848 Public Parameter                     //
//----------------------------------------------------------//
typedef enum _R848_ErrCode
{
	RT_Success = 0,
	RT_Fail    = 1
}R848_ErrCode;



typedef enum _R848_GPO_Type
{
	HI_SIG = 0,
	LO_SIG = 1
}R848_GPO_Type;

typedef enum _R848_RF_Gain_TYPE
{
	RF_AUTO = 0,
	RF_MANUAL
}R848_RF_Gain_TYPE;

typedef enum _R848_DVBS_OutputSignal_Type
{
	DIFFERENTIALOUT = 0,
	SINGLEOUT     = 1
}R848_DVBS_OutputSignal_Type;

typedef enum _R848_DVBS_AGC_Type
{
	AGC_NEGATIVE = 0,
	AGC_POSITIVE = 1
}R848_DVBS_AGC_Type;





typedef struct _R848_RF_Gain_Info
{
	uint16_t   RF_gain_comb;
	uint8_t   RF_gain1;
	uint8_t   RF_gain2;
	uint8_t   RF_gain3;
}R848_RF_Gain_Info;



int r848_standby(struct r848_priv *priv);
int r848_init(struct r848_priv *priv);
int r848_set_freq(struct r848_priv *priv, uint32_t freq);
int r848_set_gain(struct r848_priv *priv, int set_manual_gain, int gain);
int r848_set_bandwidth(struct r848_priv *priv, int bandwidth,  uint32_t rate);

#endif
