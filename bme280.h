#ifndef __BME_280_H__
#define __BME_280_H__

#include "mbed.h"

namespace bme
{
    namespace reg
    {
        uint8_t const id_reg    = 0xD0;
        uint8_t const id_val    = 0x60;
        uint8_t const calib_P1  = 0x88;
        uint8_t const calib_P2  = 0x92;
        uint8_t const calib_P3  = 0x9C;
        uint8_t const calib_P4  = 0xE1;
        uint8_t const ctrl_hum  = 0xF2;
        uint8_t const status    = 0xF3;
        uint8_t const ctrl_meas = 0xF4;
        uint8_t const measures  = 0xF7;

        namespace ctrl
        {
            uint8_t const osrs_h_x1 = 0x01;
            uint8_t const osrs_t_x1 = 0x20;
            uint8_t const osrs_p_x1 = 0x04;
            uint8_t const forced    = 0x01;
        }
        namespace bits
        {
            uint8_t const measuring = 0x08;
        }
    }
}


class BME280
{
    public:
        BME280(Serial *l_pc,I2C *l_i2c,uint8_t slave_add = (0x76<<1));
        bool check_id();
        void measure();
        int32_t getTemperature();
        int32_t getPressure();
        int32_t getHumidity();
        bool available;

    public:
        uint8_t read_reg(uint8_t reg_add);
        void write_reg(uint8_t reg_add,uint8_t val);
        void read_registers(uint8_t start,uint8_t nb, uint8_t*data);
        void set_calibration();
        uint8_t slave_add;
    private:
		void set_all_measures_8(uint8_t *data);
		int32_t		adc_P;//Pressure
		int32_t		adc_T;//Temperature
		int32_t		adc_H;//Humidity
		int32_t		t_fine;
		int32_t	compensate_T_int32();//adc_T provided with m.set_all_measures_8()
		int32_t	compensate_P_int64();
		int32_t	compensate_H_int32();
        //--------------------- calibration ---------------------
        void set_calib_part1_10(uint8_t *data);
        void set_calib_part2_10(uint8_t *data);
        void set_calib_part3_6(uint8_t *data);
        void set_calib_part4_8(uint8_t *data);
        //--------------------- Calib vars ---------------------
		uint16_t	dig_T1;//usigned short
		int16_t		dig_T2;//signed short
		int16_t		dig_T3;//signed short
		
		uint16_t	dig_P1;//usigned short
		int16_t		dig_P2;//signed short
		int16_t		dig_P3;//signed short
		int16_t		dig_P4;//signed short
		int16_t		dig_P5;//signed short
		int16_t		dig_P6;//signed short
		int16_t		dig_P7;//signed short
		int16_t		dig_P8;//signed short
		int16_t		dig_P9;//signed short
		
		uint8_t		dig_H1;//unsigned char
		int16_t		dig_H2;//signed short
		uint8_t		dig_H3;//unsigned char
		int16_t		dig_H4;//signed short
		int16_t		dig_H5;//signed short
		int8_t		dig_H6;//signed char
		
		int32_t		comp_T;
		int32_t		comp_P;
		int32_t		comp_H;
        //--------------------- vars ---------------------
        Serial *pc;
        I2C    *i2c;

};

#endif /*__BME_280_H__*/