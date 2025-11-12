/*
 * drv8353.c
 *
 *  Created on: Nov 5, 2025
 *      Author: yusha
 */



#include "drv8353.h"
#include <stdio.h>
#include "usart.h"
#include "hw_config.h"

void write_config_registers(BitbangSPI* spi){
	// register 2: DCR
    drv_write_DCR(spi, OCP_ACT_HALF, DIS_GDF_EN, OTW_REP_DIS, PWM_MODE_3X, PWM_1X_COM_SYNC, PWM_1X_DIR_0, 0, 0, 1);
	// register 3: HSR
    drv_write_HSR(spi, LOCK_OFF, IDRIVEP_HS_300MA, IDRIVEN_HS_200MA);
	// register 4: LSR
    drv_write_LSR(spi, 1, TDRIVE_1000NS, IDRIVEP_LS_850MA, IDRIVEN_LS_600MA);
    // register 5: OCPCR
    drv_write_OCPCR(spi, TRETRY_4MS, DEADTIME_50NS, OCP_LATCH, OCP_DEG_4US, VDS_LVL_0_70);
	// register 6: CSACR
    drv_write_CSACR(spi, CSA_FET_SP, VREF_DIV_2, 0, CSA_GAIN_20, DIS_SEN_EN, 0, 0, 0, SEN_LVL_0_5);
}

void read_config_registers(BitbangSPI* spi){
	uint16_t dcr = drv_read_register(spi, DCR);
	uint16_t hsr = drv_read_register(spi, HSR);
	uint16_t lsr = drv_read_register(spi, LSR);
	uint16_t ocpcr = drv_read_register(spi, OCPCR);
	uint16_t csacr = drv_read_register(spi, CSACR);

	printf("DCR: 0x%04X\n\r", dcr);
	printf("HSR: 0x%04X\n\r", hsr);
	printf("LSR: 0x%04X\n\r", lsr);
	printf("OCPCR: 0x%04X\n\r", ocpcr);
	printf("CSACR: 0x%04X\n\r", csacr);
}


uint16_t drv_spi_write(BitbangSPI* spi, uint16_t val){
    return BitbangSPI_Write(spi, val);
}

uint16_t drv_read_register(BitbangSPI* spi, int reg){
	uint16_t val = (1<<15)|(reg<<11);
    return BitbangSPI_Write(spi, val);
}

void drv_write_DCR(BitbangSPI* spi, int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT){
	uint16_t val = (DCR<<11) | (DIS_CPUV<<9) | (DIS_GDF<<8) | (OTW_REP<<7) | (PWM_MODE<<5) | (PWM_COM<<4) | (PWM_DIR<<3) | (COAST<<2) | (BRAKE<<1) | CLR_FLT;
	BitbangSPI_Write(spi, val);
}
void drv_write_HSR(BitbangSPI* spi, int LOCK, int IDRIVEP_HS, int IDRIVEN_HS){
	uint16_t val = (HSR<<11) | (LOCK<<8) | (IDRIVEP_HS<<4) | IDRIVEN_HS;
	BitbangSPI_Write(spi, val);
}
void drv_write_LSR(BitbangSPI* spi, int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS){
	uint16_t val = (LSR<<11) | (CBC<<10) | (TDRIVE<<8) | (IDRIVEP_LS<<4) | IDRIVEN_LS;
	BitbangSPI_Write(spi, val);
}
void drv_write_OCPCR(BitbangSPI* spi, int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL){
	uint16_t val = (OCPCR<<11) | (TRETRY<<10) | (DEAD_TIME<<8) | (OCP_MODE<<6) | (OCP_DEG<<4) | VDS_LVL;
	BitbangSPI_Write(spi, val);
}
void drv_write_CSACR(BitbangSPI* spi, int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL){
	uint16_t val = (CSACR<<11) | (CSA_FET<<10) | (VREF_DIV<<9) | (LS_REF<<8) | (CSA_GAIN<<6) | (DIS_SEN<<5) | (CSA_CAL_A<<4) | (CSA_CAL_B<<3) | (CSA_CAL_C<<2) | SEN_LVL;
	BitbangSPI_Write(spi, val);
}
// void drv_enable_gd(DRVStruct drv){
// 	uint16_t val = (drv_read_register(drv, DCR)) & (~(0x1<<2));
// 	drv_write_register(drv, DCR, val);
// }
// void drv_disable_gd(DRVStruct drv){
// 	uint16_t val = (drv_read_register(drv, DCR)) | (0x1<<2);
// 	drv_write_register(drv, DCR, val);
// }
// void drv_calibrate(DRVStruct drv){
// 	uint16_t val = (0x1<<4) + (0x1<<3) + (0x1<<2);
// 	drv_write_register(drv, CSACR, val);
// }
// void drv_print_faults(DRVStruct drv){
//     uint16_t val1 = drv_read_FSR1(drv);
//     uint16_t val2 = drv_read_FSR2(drv);

//     if(val1 & (1<<10)){printf("\n\rFAULT\n\r");}

//     if(val1 & (1<<9)){printf("VDS_OCP\n\r");}
//     if(val1 & (1<<8)){printf("GDF\n\r");}
//     if(val1 & (1<<7)){printf("UVLO\n\r");}
//     if(val1 & (1<<6)){printf("OTSD\n\r");}
//     if(val1 & (1<<5)){printf("VDS_HA\n\r");}
//     if(val1 & (1<<4)){printf("VDS_LA\n\r");}
//     if(val1 & (1<<3)){printf("VDS_HB\n\r");}
//     if(val1 & (1<<2)){printf("VDS_LB\n\r");}
//     if(val1 & (1<<1)){printf("VDS_HC\n\r");}
//     if(val1 & (1)){printf("VDS_LC\n\r");}

//     if(val2 & (1<<10)){printf("SA_OC\n\r");}
//     if(val2 & (1<<9)){printf("SB_OC\n\r");}
//     if(val2 & (1<<8)){printf("SC_OC\n\r");}
//     if(val2 & (1<<7)){printf("OTW\n\r");}
//     if(val2 & (1<<6)){printf("CPUV\n\r");}
//     if(val2 & (1<<5)){printf("VGS_HA\n\r");}
//     if(val2 & (1<<4)){printf("VGS_LA\n\r");}
//     if(val2 & (1<<3)){printf("VGS_HB\n\r");}
//     if(val2 & (1<<2)){printf("VGS_LB\n\r");}
//     if(val2 & (1<<1)){printf("VGS_HC\n\r");}
//     if(val2 & (1)){printf("VGS_LC\n\r");}

// }

