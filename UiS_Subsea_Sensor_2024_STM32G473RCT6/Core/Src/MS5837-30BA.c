/*
 * MS5837-30BA.c
 *
 *  Created on: Apr 5, 2024
 *      Author: wighus
 */

/* Copyright (c) 2023 Scott Rapson
 * MIT Licenced - see LICENCE for details.
 * Modified by Svein-Thore Wighus 2024
 */

#include "MS5837-30BA.h"
#include "i2c.h"

// Lagret i Programable Read Only Memory(PROM) byte 0
#define MS5837_ID_30BA26 (0x1A)

// MS5837-30BA støtter kun en adresse
#define MS5837_ADDR (0x76)

// Definerer mulige kommandoer som kan sendes til sensoren
typedef enum {
    CMD_RESET = 0x1E,
    CMD_READ = 0x00,
    CMD_READ_PROM_START = 0xA0,
    CMD_READ_PROM_END = 0xAE,

    CMD_PRESSURE_OSR_BASE = 0x40,    // OSR256
    // other OSR requests
    CMD_TEMPERATURE_OSR_BASE = 0x50, // OSR256
    // other OSR requests
} MS5837_COMMANDS;

// Oppsett for håndtering av oversamplingsrate (OSR) og konverteringstid verdier
typedef struct {
    uint8_t offset;          // Addresse offset fra base adresse for denne spesfikke OSR
    uint16_t duration_us;    // Maximum tid for ADC konvertering
} adc_osr_properties_t;

// Offset må legges til Trykk OSR baseadresse eller Temp OSR base addresses
// brukes som referanse
adc_osr_properties_t adc_osr_settings[] = {
    { .offset = 0x00, .duration_us = 560 },
    { .offset = 0x02, .duration_us = 1100 },
    { .offset = 0x04, .duration_us = 2170 },
    { .offset = 0x06, .duration_us = 4320 },
    { .offset = 0x08, .duration_us = 8610 },
    { .offset = 0x0A, .duration_us = 17200 },
};

// ---------------------------------------------------------------------

uint8_t crc4( uint16_t n_prom[7] );

void ms5837_i2c_read( ms5837_t *sensor, uint8_t command, uint8_t *data, uint8_t num_bytes );

void ms5837_i2c_write( ms5837_t *sensor, uint8_t command );

// ---------------------------------------------------------------------


void ms5837_i2c_set_read_fn( ms5837_t *sensor, user_i2c_cb_t callback )
{
    if( sensor && callback )
    {
        sensor->user_read_fn = callback;
    }
}

void ms5837_i2c_set_write_fn( ms5837_t *sensor, user_i2c_cb_t callback )
{
    if( sensor && callback )
    {
        sensor->user_write_fn = callback;
    }
}

//void ms5837_i2c_read( ms5837_t *sensor, uint8_t command, uint8_t *data, uint8_t num_bytes )
//{
//    if( sensor->user_read_fn && sensor->user_write_fn )
//    {
//        sensor->user_read_fn( MS5837_ADDR, command, data, num_bytes );
//    }
//}
//
//void ms5837_i2c_write( ms5837_t *sensor, uint8_t command )
//{
//    if( sensor->user_write_fn )
//    {
//        sensor->user_write_fn( MS5837_ADDR, command, 0, 0 );
//    }
//}

//
//// Wrapper funksjon for HAL til skriving
//void wrapper_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
//    HAL_I2C_Master_Transmit(&hi2c3, dev_addr, &reg_addr, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c3, dev_addr, data, len, HAL_MAX_DELAY);
//}
//
//// Wrapper funksjon for HAL til lesing
//void wrapper_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
//    HAL_I2C_Master_Transmit(&hi2c3, dev_addr, &reg_addr, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Receive(&hi2c3, dev_addr, data, len, HAL_MAX_DELAY);
//}

// Utfør I2C lesing ved bruk av wrapper funksjon
void ms5837_i2c_read(ms5837_t *sensor, uint8_t command, uint8_t *data, uint8_t num_bytes) {
	uint8_t message[1];
	message[0] = command;
	HAL_I2C_Master_Transmit(&hi2c3,(0x76<<1), message, 1,10); // Send kommando
	HAL_I2C_Master_Receive(&hi2c3,(0x76<<1), data, num_bytes,10); // Motta data
}

//Utfør I2C skriving ved bruk av wrapper funksjon
void ms5837_i2c_write(ms5837_t *sensor, uint8_t command) {
	uint8_t message[1];
	message[0] = command;
	HAL_I2C_Master_Transmit(&hi2c3,(0x76<<1), message, 1,10);
}
// ---------------------------------------------------------------------

// Utfør reset av software på sensoren
void ms5837_reset( ms5837_t *sensor )
{
    ms5837_i2c_write( sensor, CMD_RESET );
}


// Be om PROM data fra sensoren og lagre det i structuren
// returnerer True hvis fullført

bool ms5837_read_calibration_data( ms5837_t *sensor )
{
    if( !sensor )
    {
        return false;
    }

    // Les de 7 16-bit verdiene fra PROM
    for( uint8_t i = 0; i < NUM_CALIBRATION_VARIABLES; i++ )
    {
        uint8_t buffer[2] = { 0 };
        ms5837_i2c_read( sensor, CMD_READ_PROM_START+(i*2), &buffer[0], 2 );

        sensor->calibration_data[i] = (buffer[0] << 8);    // MSB
        sensor->calibration_data[i] |= buffer[1];          // LSB
    }

    // Valider Cyclic redudancy check (CRC)
    uint8_t crc_rx = sensor->calibration_data[C0_VERSION] >> 12;
    uint8_t crc_calc = crc4( &sensor->calibration_data[0] );

    sensor->calibration_loaded = ( crc_rx == crc_calc );

    // Sjekk sensor versjon
    uint8_t version = (sensor->calibration_data[C0_VERSION] >> 5) & 0x7F;
    sensor->variant = version;  // TODO map to an enum here

    return sensor->calibration_loaded;
}


// Begynner ADC convertering, returnerer antall mikrosekunder til data er klart
// hvis ugyldig eller feil vil den returnere 0

uint16_t ms5837_start_conversion( ms5837_t *sensor, MS5837_SELECT_SENSOR type, MS5837_ADC_OSR osr )
{
    if( !sensor )
    {
        return 0;
    }

    switch( type )
    {
        case SENSOR_PRESSURE:
            ms5837_i2c_write( sensor, CMD_PRESSURE_OSR_BASE + adc_osr_settings[osr].offset );
            sensor->last_conversion = SENSOR_PRESSURE;
        break;

        case SENSOR_TEMPERATURE:
            ms5837_i2c_write( sensor, CMD_TEMPERATURE_OSR_BASE + adc_osr_settings[osr].offset );
            sensor->last_conversion = SENSOR_TEMPERATURE;
        break;

        default:
     //       sensor->last_conversion = NUM_SENSOR_FIELDS+1; // TODO consider an invalid enum value
            return 0;
    }

    return adc_osr_settings[osr].duration_us;
}


// Les sensor data, 24-bit unsigned (uint32_t)
// Lagres i objektet
// Returnerer 0 for feil, eller konverterignsverdien hvis OK

uint32_t ms5837_read_conversion( ms5837_t *sensor )
{
    if( !sensor || sensor->last_conversion >= NUM_SENSOR_FIELDS )
    {
        return 0;
    }

    uint8_t value[3] = { 0 };
    ms5837_i2c_read( sensor, CMD_READ, &value[0], 3 );

    uint32_t conversion = 0;
    conversion = value[0];
    conversion = (conversion << 8) | value[1];
    conversion = (conversion << 8) | value[2];

    sensor->samples[sensor->last_conversion] = conversion;
    sensor->last_conversion = NUM_SENSOR_FIELDS; // invalidate

    return conversion;
}

bool ms5837_calculate( ms5837_t *sensor )
{
    if( !sensor || !sensor->calibration_loaded )
    {
        return false;
    }

    if( !sensor->samples[0] || !sensor->samples[1] )
    {
        return false;
    }

    uint32_t sample_pressure = sensor->samples[SENSOR_PRESSURE];
    uint32_t sample_temperature = sensor->samples[SENSOR_TEMPERATURE];


    // Første Orden Konvertering

    // deltaTemp, 25-bit signed (int32_t)
    // dT = D2 - TREF
    //    = D2 - C5 * 2^8
    int32_t delta_temp = sample_temperature - (uint32_t)sensor->calibration_data[C5_TEMP_REFERENCE] * (1 << 8);

    // Faktisk Temp - 41-bit signed(int64_t)
    // TEMP = 20°C + dT * TEMPSENS
    //      = 2000 + dT * C6 / 2^23
    int32_t temperature = 2000UL + delta_temp * (int64_t)sensor->calibration_data[C6_TEMP_COEFF] / ((uint32_t)1 << 23);

    // Trykk Offset - 41-bit signed (int64_t)
    // OFF = OFF_T1 + TCO * dT
    //     = C2 * 2^17 + (C4 * dT ) / 2^6
    int64_t pressure_offset = ((int64_t)sensor->calibration_data[C2_PRESSURE_OFFSET] * ((uint32_t)1 << 17))
                              + ((int64_t)sensor->calibration_data[C4_TEMP_PRESSURE_OFFSET_COEFF] * delta_temp)/(1 << 6);

    // Trykk Sensitivitet ved faktisk temp - 41-bit signed (int64_t)
    // SENS = SENS T1 + TCS * dT
    //      = C1 * 2^16 + (C3 * dT ) / 2^7
    int64_t pressure_sensitivity = (int64_t)sensor->calibration_data[C1_PRESSURE_SENSITIVITY] * ((uint32_t)1 << 16)
                                   + ( (int64_t)sensor->calibration_data[C3_TEMP_PRESSURE_SENSITIVITY_COEFF] * delta_temp)/((uint32_t)1 << 7);

    // Trykk (kompansert for temp) - 58-bit signed (int64_t)
    // P = D1 * SENS - OFF
    //   = (D1 * SENS / 2^21 - OFF) / 2^15

    //Første ordens trykk verdiene brukes ikke, men er vedlagt som referanse
    // int32_t trykk = ( sample_pressure * (pressure_sensitivity / ((uint32_t)1 << 21)) - pressure_offset ) / ((uint32_t)1 << 15);


    // Utreginger for andre ordens konvertering

    // Lav temp kompensajon (<20C)
    int32_t temp_i = 0;
    int32_t offset_i = 0;
    int32_t sensitivity_i = 0;

    if( (temperature / 100U) < 20 )
    {
        // Ti = 11 * dT^2 / 2^35
         temp_i = ( 11 * (int64_t)delta_temp*(int64_t)delta_temp ) / ((uint64_t)1 << 35);

        // OFFi = 31 * (TEMP - 2000)^2 / 2^3
         offset_i = ( 31 * (temperature-2000)*(temperature-2000) ) / (1 << 3);

        // SENSi = 63 * (TEMP - 2000)^2 / 2^5
         sensitivity_i = ( 63 * (temperature-2000)*(temperature-2000) ) / (1 << 5);
    }

    // kalkuler andre orden elementer
    // OFF2 = OFF - OFFi
    int64_t offset_2 = pressure_offset - offset_i;

    // SENS2 = SENS - SENSi
    int64_t sensitivity_2 = pressure_sensitivity - sensitivity_i;

    // TEMP2 = (TEMP - Ti)/100      degC
    int32_t temperature_2 = ( temperature - temp_i );

    // P2 = ( (D1*SENS2 / 2^21 - OFF2 ) / 2^15 ) / 100    milibar
    int32_t pressure_2 = ( ( (sample_pressure * sensitivity_2) / ((uint32_t)1 << 21) - offset_2 ) / ((uint32_t)1 << 15) );

    // Lagre resultatene i  sensor structuren
    sensor->measurements[SENSOR_PRESSURE] = pressure_2;
    sensor->measurements[SENSOR_TEMPERATURE] = temperature_2;

    // Nullstill
    sensor->samples[SENSOR_PRESSURE] = 0;
    sensor->samples[SENSOR_TEMPERATURE] = 0;

    return true;
}

// ---------------------------------------------------------------------

float ms5837_temperature_celcius( ms5837_t *sensor )
{
    return sensor->measurements[SENSOR_TEMPERATURE] / 100.0f;
}

//float ms5837_temperature_fahrenheit( ms5837_t *sensor )
//{
//    // degC * 1.8 + 32 = degF
//    // As intermediate value is in centi-degrees, merge the /100 and *1.8
//
//    return ((float)sensor->measurements[SENSOR_TEMPERATURE] / 55.5555f ) + 32.0f;
//}

// ---------------------------------------------------------------------

float ms5837_pressure_bar( ms5837_t *sensor )
{
    return sensor->measurements[SENSOR_PRESSURE] / 100000.0f;
}

float ms5837_pressure_mbar( ms5837_t *sensor )
{
    return sensor->measurements[SENSOR_PRESSURE] / 100.0f;
}

//float ms5837_pressure_atm( ms5837_t *sensor )
//{
//    // 1bar = 0.98692326671 atm
//    // converted value is in 10ths of a bar, so scale the conversion factor
//    return sensor->measurements[SENSOR_PRESSURE] / 101325.0f;
//}
//
//float ms5837_pressure_pascal( ms5837_t *sensor )
//{
//    return sensor->measurements[SENSOR_PRESSURE];
//}

// ---------------------------------------------------------------------

// RROM is 7 unsigned int16 values for 112-bits
uint8_t crc4( uint16_t n_prom[7] )
{
    uint16_t crc_rem = 0; // CRC remainder

    n_prom[0] = n_prom[0] & 0x0FFF; // CRC byte is replaced by 0
    n_prom[7] = 0;                  // Subsidiary value, set to 0

    for( uint8_t byte = 0; byte < 16; byte++ )
    {
        // choose LSB or MSB
        if( byte % 2 == 1 )
        {
            crc_rem ^= (unsigned short)(n_prom[byte >> 1] & 0x00FF);
        }
        else
        {
            crc_rem ^= (unsigned short)(n_prom[byte >>1 ] >> 8);
        }

        for( uint8_t n_bit = 8; n_bit > 0; n_bit-- )
        {
            if( crc_rem & 0x8000 )
            {
                crc_rem = (crc_rem << 1) ^ 0x3000;
            }
            else
            {
                crc_rem = (crc_rem << 1);
            }
        }
    }

    crc_rem = ((crc_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code

    return (crc_rem ^ 0x00);
}
