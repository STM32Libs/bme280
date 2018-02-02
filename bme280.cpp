#include "bme280.h"

#include "mbed.h"
#include "utils.h"

BME280::BME280(Serial *l_pc,I2C*l_i2c,uint8_t l_slave_add)
{
    pc = l_pc;
    i2c = l_i2c;
    slave_add = l_slave_add;
    available = check_id();
    if(available)
    {
        set_calibration();
    }
}

uint8_t BME280::read_reg(uint8_t reg_add)
{
    uint8_t res;
    i2c->write(slave_add,(char*)&reg_add,1);
    i2c->read(slave_add,(char*)&res,1);
    return res;
}

void BME280::read_registers(uint8_t start,uint8_t nb, uint8_t*data)
{
    //pc->printf("write start 0x%02X\n",start);
    i2c->write(slave_add,(char*)&start,1);
    i2c->read(slave_add,(char*)data,nb);
}

void BME280::write_reg(uint8_t reg_add,uint8_t val)
{
    uint8_t data[2];
    data[0] = reg_add;
    data[1] = val;
    i2c->write(slave_add,(char*)data,2);
}

bool BME280::check_id()
{
    bool res = false;
    uint8_t Id = read_reg(bme::reg::id_reg);
    if(Id == bme::reg::id_val)
    {
        pc->printf("BME280> id 0x%02X\n",Id);
        res = true;
    }
    else
    {
        pc->printf("BME280> No id found\n");
    }
    return res;
}

void BME280::set_calibration()
{
    uint8_t data[10];
    read_registers(bme::reg::calib_P1,10,data);//print_tab(pc,data,10);
    set_calib_part1_10(data);
    read_registers(bme::reg::calib_P2,10,data);//print_tab(pc,data,10);
    set_calib_part2_10(data);
    read_registers(bme::reg::calib_P3,6,data);//print_tab(pc,data,6);
    set_calib_part3_6(data);
    read_registers(bme::reg::calib_P4,8,data);//print_tab(pc,data,8);
    set_calib_part4_8(data);
}

void BME280::measure()
{
    //select humidity
    write_reg(bme::reg::ctrl_hum,bme::reg::ctrl::osrs_h_x1);
    //select temperature and pressur and trigger the measure with fordced bit
    write_reg(bme::reg::ctrl_meas,  bme::reg::ctrl::forced      | 
                                    bme::reg::ctrl::osrs_t_x1   |
                                    bme::reg::ctrl::osrs_p_x1       );
    //wait for the measures to complete
    uint8_t status;
    uint8_t count = 0;
    do
    {
        status = read_reg(bme::reg::status);
        count++;
    }while(((status & bme::reg::bits::measuring) != 0) && (count<30));

    //handle wait overflow error
    if(count == 30)
    {
        pc->printf("BME280> measure wait overflow\n");
    }
    else
    {
        pc->printf("BME280> done measuring in %d\n",count);
        uint8_t data[8];
        //read all the measures from the sensor
        read_registers(bme::reg::measures,8,data);
        //write the measures byte array into the right variables
        set_all_measures_8(data);
        //process the compensation with the calibration parameters !temperature first as t_fine used by others!
        comp_T = compensate_T_int32();pc->printf("Temperature:%ld\n",comp_T);
        comp_P = compensate_P_int64();pc->printf("Pressure:%ld\n",comp_P/256);
        comp_H = compensate_H_int32();pc->printf("Humidity:%ld\n",comp_H);
        
    }

}

void BME280::set_calib_part1_10(uint8_t *data)
{
	//DEBUG std::cout << "Part1:" << utl::data2hextext(data,10) << std::endl;
	
	//data[10] : 0x88 0x89 0x8A 0x8B 0x8C 0x8D 0x8E 0x8F 0x90 0x91
	dig_T1 = 			(uint16_t)data[1]<<8 | data[0];		//0x89<<8 | 0x88
	dig_T2 = (int16_t)( (uint16_t)data[3]<<8 | data[2]);	//0x8B<<8 | 0x8A
	dig_T3 = (int16_t)( (uint16_t)data[5]<<8 | data[4]);	//0x8D<<8 | 0x8C

	dig_P1 = 			(uint16_t)data[7]<<8 | data[6];		//0x8F<<8 | 0x8E
	dig_P2 = (int16_t)( (uint16_t)data[9]<<8 | data[8]);	//0x91<<8 | 0x90
}

void BME280::set_calib_part2_10(uint8_t *data)
{
	//DEBUG std::cout << "Part2:" << utl::data2hextext(data,10) << std::endl;

	//data[10] : 0x92 0x93 0x94 0x95 0x96 0x97 0x98 0x99 0x9A 0x9B
	dig_P3 = (int16_t)( (uint16_t)data[1]<<8 | data[0]);	//0x93<<8 | 0x92
	dig_P4 = (int16_t)( (uint16_t)data[3]<<8 | data[2]);	//0x95<<8 | 0x94
	dig_P5 = (int16_t)( (uint16_t)data[5]<<8 | data[4]);	//0x97<<8 | 0x96
	dig_P6 = (int16_t)( (uint16_t)data[7]<<8 | data[6]);	//0x99<<8 | 0x98
	dig_P7 = (int16_t)( (uint16_t)data[9]<<8 | data[8]);	//0x9B<<8 | 0x9A
}

void BME280::set_calib_part3_6(uint8_t *data)
{
	//DEBUG	std::cout << "Part3:" << utl::data2hextext(data,6) << std::endl;

	//data[6] : 0x9C 0x9D 0x9E 0x9F 0xA0 0xA1
	dig_P8 = (int16_t)( (uint16_t)data[1]<<8 | data[0]);	//0x9D<<8 | 0x9C
	dig_P9 = (int16_t)( (uint16_t)data[3]<<8 | data[2]);	//0x9F<<8 | 0x9E

	dig_H1 = data[5];										//0xA1 : note that 0xA0 (data[4]) is skipped
}

void BME280::set_calib_part4_8(uint8_t *data)
{
	//DEBUG std::cout << "Part4:" << utl::data2hextext(data,8) << std::endl;

	//   0    1    2    3    4    5    6    7
	//0xE1 0xE2 0xE3 0xE4 0xE5 0xE6 0xE7 0xE8
	dig_H2 = (int16_t)( (uint16_t)data[1]<<8 | data[0]);			//0xE2<<8 | 0xE1
	dig_H3 = data[2];												//0xE3
	dig_H4 = (int16_t)( (uint16_t)data[3]<<4 | (0x0F & data[4]));	//0xE4<<4 | 0xE5[3:0] : (12bits) Exception lower is on msb here
	dig_H5 = (int16_t)( (uint16_t)data[5]<<4 | (data[4]>>4));		//0xE6<<4 | 0xE5[7:4] : (12bits) 4 bits left from 0xE5 used here
	dig_H6 = (int8_t)data[6];										//0xE7 : 0xE8 is unused

}

void BME280::set_all_measures_8(uint8_t *data)
{
	//DEBUG std::cout << "Measures:" << utl::data2hextext(data,8) << std::endl;

	//   0    1    2    3    4    5    6    7
	//0xF7 0xF8 0xF9 0xFA 0xFB 0xFC 0xFD 0xFE
	adc_P = (int32_t) (  (uint32_t)data[0]<<12 | (uint32_t)data[1]<<4 | data[2]>>4  );	//0xF7<<12 | 0xF8<<4 | 0xF9>>4
	adc_T = (int32_t) (  (uint32_t)data[3]<<12 | (uint32_t)data[4]<<4 | data[5]>>4  );	//0xFA<<12 | 0xFB<<4 | 0xFC>>4
	adc_H = (int32_t) (  (uint32_t)data[6]<<8  | data[7]  );							//0xF7<<12 | 0xF8<<4 | 0xF9>>4
}

// adc_T provided with m.set_all_measures_8()
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t	BME280::compensate_T_int32()
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (	(		(	((adc_T>>4) - ((int32_t)dig_T1)) * 
							((adc_T>>4) - ((int32_t)dig_T1))
						) >> 12
				) *	((int32_t)dig_T3)
			) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// adc_P provided with m.set_all_measures_8()
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
int32_t	BME280::compensate_P_int64()
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

// adc_H provided with m.set_all_measures_8()
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
int32_t	BME280::compensate_H_int32()
{
	int32_t v_x1_u32r;
	
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (	(	(	(	(adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)
							) + ((int32_t)16384)
						) >> 15
					) * 
					(	(	(	(	(((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10
								) + ((int32_t)2097152)
							) *	((int32_t)dig_H2) + 8192
						) >> 14
					)
				);
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	
	return (uint32_t)(v_x1_u32r>>12);
}

