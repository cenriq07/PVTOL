/*
 * MPU9250.c
 *
 *  Created on: 07/05/2021
 *      Author: Enrique
 */

#include "MPU9250.h"

uint16 ComandoSPI_MPU[4]= {0x0,0x0,0x0,0x0};
uint16 DatoSPI01_MPU[4]= {0,0,0,0};
uint16 DatoSPI02_MPU[4]= {0,0,0,0};
uint16 DatoSPI03_MPU[4]= {0,0,0,0};
uint16 DatoSPI04_MPU[4]= {0,0,0,0};
uint16 DatoSPI05_MPU[4]= {0,0,0,0};
uint16 DatoSPI06_MPU[4]= {0,0,0,0};

uint16_t AK8963ReadOut[7]= {0,0,0,0,0,0,0,0};

unsigned int temp_val;



void MPUInit()
{
    SPI1_data_configCh2.CS_HOLD=FALSE;
    SPI1_data_configCh2.WDEL=TRUE;
    SPI1_data_configCh2.DFSEL=SPI_FMT_0;
    SPI1_data_configCh2.CSNR=SPI_CS_1;
    sciEnviarDatos(sprintf(command," // ************   I AM HERE  ************** //"),command, 1);

    wait(50000);
    wait(50000);
    wait(50000);
    wait(50000);
    _enable_interrupt_();

    config_MPU_9250(spiREG1, &SPI1_data_configCh2); // comfiguramos MPU (Giroscopio, acel, temp)
    config_AK8963(spiREG1, &SPI1_data_configCh2); // configuramos magnetometro

    read_AK8963(spiREG1, &SPI1_data_configCh2, MPU_AK8963_WIA, AK8963ReadOut);

   // sciEnviarDatos(sprintf(command,"MAG ID=  %X ",AK8963ReadOut[0] ),command, 1);
   //  sciEnviarDatos(sprintf(command,"If 48, then press button"),command, 1);
}

void wait(unsigned int id)
{
    volatile int id1=0;
    for (id1 = 0; id1 < id; ++id1)
    {
        asm(" nop");
    }
}


boolean read_MANG_AK8963(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int16_t *ReadOut, boolean imprimir_SN)
{



    uint16_t AK8963RdO[7];
    uint16_t SPITMP=0;
    write_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_CNTL,AK8963RdO,0x0016 );

    wait(15000); // OJO, este retardo es cr�tico para que la lectura est� lista. con 8000 a veces no alcanza. FreeRTOS: 10000 est� chido.

    //        AK8963ReadOut[0]=AK8963ReadOut[0] & 0x00FF;
    //sciEnviarDatos(sprintf(command,"    MPU_AK8963_CNTL=  %X   ",AK8963ReadOut[0] ),command, 0);

    read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_ST1, AK8963RdO);
    //        AK8963ReadOut[0]=AK8963ReadOut[0] & 0x00FF;
    //sciEnviarDatos(sprintf(command,"    MPU_AK8963_ST1 = =  %X",AK8963ReadOut[0] ),command, 0);

    if(AK8963RdO[0] & 0x0001)
    {
        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_HXL, AK8963RdO);
        //sciEnviarDatos(sprintf(command,"        MxL=  %X ",AK8963ReadOut[0] ),command, 0);
        SPITMP = ( AK8963RdO[0] & 0xFF);
        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_HXH, AK8963RdO+1);
        //sciEnviarDatos(sprintf(command,"MxH=  %X ",AK8963ReadOut[1] ),command, 0);
        SPITMP = ( ( (AK8963RdO[1] & 0xFF)<<8) | SPITMP);
        ReadOut[0] = (int32_t)((int16_t) SPITMP);

        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_HYL, AK8963RdO+2);
        //sciEnviarDatos(sprintf(command,"MyL=  %X ",AK8963ReadOut[2] ),command, 0);
        SPITMP = ( AK8963RdO[2] & 0xFF);
        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_HYH, AK8963RdO+3);
        //sciEnviarDatos(sprintf(command,"MyH=  %X ",AK8963ReadOut[3] ),command, 0);
        SPITMP = ( ( (AK8963RdO[3] & 0xFF)<<8) | SPITMP);
        ReadOut[1] = (int32_t)((int16_t) SPITMP);
//            sciEnviarDatos(sprintf(command,"    My= %d",MAGN_Sensout[1] ),command, 0);

//        ftoa(ReadOut[1], cMy, 2);
//
//        if(imprimir_SN)
//        {
//            sciEnviarDatos(sprintf(command,"%s,",cMy), command, 0);         /* MODIFIED */
//        }


        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_HZL, AK8963RdO+4);
        //sciEnviarDatos(sprintf(command,"MzL=  %X ",AK8963ReadOut[4] ),command, 0);
        SPITMP = ( AK8963RdO[4] & 0xFF);
        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_HZH, AK8963RdO+5);
        //sciEnviarDatos(sprintf(command,"MzH=  %X ",AK8963ReadOut[5] ),command, 0);
        SPITMP = ( ( (AK8963RdO[5] & 0xFF)<<8) | SPITMP);
        ReadOut[2] = (int32_t)((int16_t) SPITMP);
//            sciEnviarDatos(sprintf(command,"    Mz= %d",MAGN_Sensout[2] ),command, 0);

//        ftoa(ReadOut[2], cMz, 2);
//
//        if(imprimir_SN)
//        {
//            sciEnviarDatos(sprintf(command,"%s,",cMz), command, 0);         /* MODIFIED */
//        }
        read_AK8963(spiREGISTRO, SPIconfig, MPU_AK8963_TS2, AK8963ReadOut+6);
//            sciEnviarDatos(sprintf(command,"    M_ST2= %X",AK8963ReadOut[6] ),command, 1);
        return(0);
        // ***********************************
    }
    else
    {
        return(1); // no hubo lectura, dato no disonible
    }
}


void read_AK8963(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int16_t regRead,int16_t *ReadOut)
{
    // ****************** BEGIN     CONFIGURAMOS I2C direcci�n base  ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_ADDR)<<8) |  MPU_AK8963_ADDRESS|  0x80); //  Set the I2C slave address of AK8963 and set for read.
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    // ****************** END     CONFIGURAMOS I2C direcci�n base  ************ //
    // ****************** BEGIN     Escribimos registo que deseamos leer  ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_REG)<<8) |  regRead); // I2C slave 0 register address from where to begin data transfer
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    // ****************** END     Escribimos registo que deseamos leer  ************ //
    // ****************** BEGIN     indicamos datos que van a leerse  ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_CTRL)<<8) |  0x81);   // 8: habilitamos lectura del esclavo para almacenar en SLV0. 1: un byte. I2C slave 0 register address from where to begin data transfer, Enable I2C and transfer 1 byte
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    // ****************** END       indicamos datos que van a leerse  ************ //
    // ****************** BEGIN     Leemos registro donde se escribi� el dato  ************ //
    ComandoSPI_MPU[0]=(((MPU_READ | MPU_EXT_SENS_DATA_00)<<8));      // Read the WHO_AM_I byte
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    ReadOut[0] = ( DatoSPI01_MPU[0]);
    // ****************** END       Leemos registro donde se escribi� el dato  ************ //
}


void write_AK8963(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int16_t regwrite, int16_t *ReadOut, int16_t dataWrite)
{
    int16_t readout2[1]= {0};
    // ****************** BEGIN     CONFIGURAMOS I2C direcci�n base  ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_ADDR)<<8) |  MPU_AK8963_ADDRESS|  0x00); //  Set the I2C slave address of AK8963 and set for read.
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    // ****************** BEGIN     CONFIGURAMOS I2C direcci�n base  ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_REG)<<8) |  regwrite); //  Set the I2C slave address of AK8963 and set for read.
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    // ****************** END     CONFIGURAMOS I2C direcci�n base  ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_DO)<<8) |  dataWrite); //  Set the I2C slave address of AK8963 and set for read.
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    // ****************** END     CONFIGURAMOS I2C direcci�n base  ************ //
    read_AK8963(spiREGISTRO, SPIconfig, regwrite,readout2);
    ReadOut[0] = readout2[0];
    // ****************** END       Leemos registro donde se escribi� el dato  ************ //

}


/*----------------------------------------------------------------------------------------*/

boolean mpuGetGyroAccel(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int *Gyro, int *Accel)
{
    uint16_t  SPI_Dat_respH_L=0;

    /* ----------------->|| GIROSCOPIO ||<----------------- */

    /*-----------------| Gx |--------------------*/

    ComandoSPI_MPU[0]=(MPU_GYRO_XOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_GYRO_XOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    Gyro[X]=(int32_t)((int16_t)(SPI_Dat_respH_L));

    /*-----------------| Gy |--------------------*/

    ComandoSPI_MPU[0]=(MPU_GYRO_YOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_GYRO_YOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    Gyro[Y]=(int32_t)((int16_t)(SPI_Dat_respH_L));

    /*-----------------| Gz |--------------------*/

    ComandoSPI_MPU[0]=(MPU_GYRO_ZOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_GYRO_ZOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    Gyro[Z]=(int32_t)((int16_t)(SPI_Dat_respH_L));

    /* ----------------->|| ACELEROMETRO ||<----------------- */

    /*-----------------| Ax |--------------------*/

    ComandoSPI_MPU[0]=(MPU_ACCEL_XOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_ACCEL_XOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    Accel[X]=(int32_t)((int16_t)(SPI_Dat_respH_L));

    /*-----------------| Ay |--------------------*/

    ComandoSPI_MPU[0]=(MPU_ACCEL_YOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_ACCEL_YOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    Accel[Y]=(int32_t)((int16_t)(SPI_Dat_respH_L));

    /*-----------------| Az |--------------------*/

    ComandoSPI_MPU[0]=(MPU_ACCEL_ZOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_ACCEL_ZOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    Accel[Z]=(int32_t)((int16_t)(SPI_Dat_respH_L));
}

float mpuGetAccAngle_Y(float Ax, float Ay,float Az)
{
    float angY = 0.0;
    angY = atan(Ay/(sqrt(pow(Ax,2) + pow(Az,2))));      // Radianes
    angY = angY*180/3.14159;                            // Grados
    return angY;
}

/*-----------------------------------------------------------------------------------*/

boolean read_MPU_ACEL_TMP_VF(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int *ReadOut_TMP_Giro_Acc, boolean imprimir_SN_TMP, boolean imprimir_SN_GIRO, boolean imprimir_SN_ACEL)
{


    uint16_t  SPI_Dat_respH_L=0;
    // *****************   Obtenemos temperatura
    ComandoSPI_MPU[0]=(MPU_TEMP_OUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    ComandoSPI_MPU[0]=(MPU_TEMP_OUT_L | MPU_READ)<<8; // Preguntamos quien es
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[0]=(int32_t)((int16_t)(SPI_Dat_respH_L));



   // if(imprimir_SN_TMP)
    //{
      //  sciEnviarDatos(sprintf(command,"%d,",ReadOut_TMP_Giro_Acc[0]),command, 0);
    //}
    // *****************   Obtenemos Giroscopio

//Gx
    ComandoSPI_MPU[0]=(MPU_GYRO_XOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_GYRO_XOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[1]=(int32_t)((int16_t)(SPI_Dat_respH_L));
    ReadOut_TMP_Giro_Acc[1]=((ReadOut_TMP_Giro_Acc[1]));
//Gy

    ComandoSPI_MPU[0]=(MPU_GYRO_YOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_GYRO_YOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[2]=(int32_t)((int16_t)(SPI_Dat_respH_L));
    ReadOut_TMP_Giro_Acc[2]=(ReadOut_TMP_Giro_Acc[2]);
//Gy
    ComandoSPI_MPU[0]=(MPU_GYRO_ZOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_GYRO_ZOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[3]=(int32_t)((int16_t)(SPI_Dat_respH_L));
    ReadOut_TMP_Giro_Acc[3]=((ReadOut_TMP_Giro_Acc[3]+9));


    // *****************   Obtenemos Aceler�metro
//Ax
    ComandoSPI_MPU[0]=(MPU_ACCEL_XOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_ACCEL_XOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[4]=(int32_t)((int16_t)(SPI_Dat_respH_L));


    ComandoSPI_MPU[0]=(MPU_ACCEL_YOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_ACCEL_YOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[5]=(int32_t)((int16_t)(SPI_Dat_respH_L));

//Az
    ComandoSPI_MPU[0]=(MPU_ACCEL_ZOUT_H | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = ( DatoSPI01_MPU[0]<<8);
    ComandoSPI_MPU[0]=(MPU_ACCEL_ZOUT_L | MPU_READ)<<8; // Preguntamos quien es
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    DatoSPI01_MPU[0]=(DatoSPI01_MPU[0] & 0x00FF);
    SPI_Dat_respH_L = (SPI_Dat_respH_L | DatoSPI01_MPU[0]);
    ReadOut_TMP_Giro_Acc[6]=(int32_t)((int16_t)(SPI_Dat_respH_L));
//    ReadOut_TMP_Giro_Acc[6]=(((int32_t)((int16_t)(SPI_Dat_respH_L)))*Az)+0.02;

}


void config_MPU_9250(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig)
{
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_GYRO_CONFIG)<<8)  |  0x18); // configuramos sensibilidad del giroscopio
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
}
void config_AK8963(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig)
{
    uint16_t AK8963_ReadOUT[1]= {0};
    wait(5000);
    // ****************** BEGIN     CONFIGURAMOS I2C esclavo   ************ //
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_USER_CTRL)<<8)  |  0x20); // // Enable I2C Master mode
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(50000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_MST_CTRL)<<8)  |  0x0D); // I2C configuration multi-master I2C 400KHz
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(50000);
    // ****************** END       CONFIGURAMOS I2C esclavo   ************ //
    write_AK8963_VF(spiREG1, SPIconfig, MPU_AK8963_CNTL,AK8963_ReadOUT,0x0016 ); // configuramos modo continuo y 16 bits
    AK8963_ReadOUT[0]=AK8963_ReadOUT[0] & 0x00FF;
    //sciEnviarDatos(sprintf(command,"Magn CTRL1=  %X ",AK8963ReadOut[0]),command, 0);
    wait(5000);
}

void config_MPU_9250_VF(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig)
{
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_GYRO_CONFIG)<<8)  |  0x18);
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
}

void config_AK8963_VF(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig)
{
    uint16_t AK8963_ReadOUT[1]= {0};
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_USER_CTRL)<<8)  |  0x20);
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_MST_CTRL)<<8)  |  0x0D);
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    write_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_CNTL,AK8963_ReadOUT,0x0016 );
    AK8963_ReadOUT[0]=AK8963_ReadOUT[0] & 0x00FF;
    //sciEnviarDatos(sprintf(command,"Magn CTRL1=  %X ",AK8963ReadOut[0]),command, 0);
    wait(5000);
}


void read_AK8963_VF(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int16_t regRead,int16_t *ReadOut)
{
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_ADDR)<<8) |  MPU_AK8963_ADDRESS|  0x80); //  Set the I2C slave address of AK8963 and set for read.
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_REG)<<8) |  regRead); // I2C slave 0 register address from where to begin data transfer
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_CTRL)<<8) |  0x81);   // 8: habilitamos lectura del esclavo para almacenar en SLV0. 1: un byte. I2C slave 0 register address from where to begin data transfer, Enable I2C and transfer 1 byte
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    ComandoSPI_MPU[0]=(((MPU_READ | MPU_EXT_SENS_DATA_00)<<8));      // Read the WHO_AM_I byte
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(5000);
    ReadOut[0] = ( DatoSPI01_MPU[0]);
}


void write_AK8963_VF(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int16_t regwrite, int16_t *ReadOut, int16_t dataWrite)
{
    int16_t readout2[1]= {0};
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_ADDR)<<8) |  MPU_AK8963_ADDRESS|  0x00);
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(500);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_REG)<<8) |  regwrite);
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(500);
    ComandoSPI_MPU[0]=(((MPU_WRITE | MPU_I2C_SLV0_DO)<<8) |  dataWrite);
    spiSendAndGetData(spiREGISTRO, SPIconfig,(uint32) 1, ComandoSPI_MPU,DatoSPI01_MPU);
    wait(1000);
    read_AK8963_VF(spiREGISTRO, SPIconfig, regwrite,readout2);
    ReadOut[0] = readout2[0];

}


boolean read_MANG_AK8963_VF(spiBASE_t *spiREGISTRO, spiDAT1_t *SPIconfig, int16_t *ReadOut, boolean imprimir_SN)
{
    uint16_t AK8963RdO[7];
    uint16_t SPITMP=0;
    write_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_CNTL,AK8963RdO,0x0016 );
    wait(10000); // sustituir en las tareas
    read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_ST1, AK8963RdO);
    if(AK8963RdO[0] & 0x0001)
    {
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_HXL, AK8963RdO);
        SPITMP = ( AK8963RdO[0] & 0xFF);
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_HXH, AK8963RdO+1);
        SPITMP = ( ( (AK8963RdO[1] & 0xFF)<<8) | SPITMP);
        ReadOut[0] = (int32_t)((int16_t) SPITMP);
        if(imprimir_SN)
        {
            sciEnviarDatos(sprintf(command,"%d, ",ReadOut[0] ),command, 0);
        }
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_HYL, AK8963RdO+2);
        SPITMP = ( AK8963RdO[2] & 0xFF);
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_HYH, AK8963RdO+3);
        SPITMP = ( ( (AK8963RdO[3] & 0xFF)<<8) | SPITMP);
        ReadOut[1] = (int32_t)((int16_t) SPITMP);
        if(imprimir_SN)
        {
            sciEnviarDatos(sprintf(command,"%d, ",ReadOut[1] ),command, 0);
        }
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_HZL, AK8963RdO+4);
        SPITMP = ( AK8963RdO[4] & 0xFF);
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_HZH, AK8963RdO+5);
        SPITMP = ( ( (AK8963RdO[5] & 0xFF)<<8) | SPITMP);
        ReadOut[2] = (int32_t)((int16_t) SPITMP);
        if(imprimir_SN)
        {
            sciEnviarDatos(sprintf(command,"%d, ",ReadOut[2] ),command, 1);
        }
        read_AK8963_VF(spiREGISTRO, SPIconfig, MPU_AK8963_TS2, AK8963ReadOut+6);
        return(0);
        // ***********************************
    }
    else
    {
        return(1); // no hubo lectura, dato no disonible
    }
}

bool sciEnviarDatos(uint32 numOfDat, char* charDat, bool sc) {
    sciSend(scilinREG, numOfDat, (uint8 *)charDat);
    if (sc) {
        sciSend(scilinREG, 0x02, (unsigned char *)"\r\n");
    }
    return true;
}






