#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

// TODO: Rounding is not handled properly at the last digit (carry chain)

#define AREF_LEVEL		2.56
#define ADC_BITS        10
#define MAX_SAMPLE  	((1<<ADC_BITS)-1)
#define AVG_N_SAMPLES   (1<<((sizeof(uint16_t)*8) - ADC_BITS))
#define MAX_A_SAMPLE    ((1<<(sizeof(uint16_t)*8))-1)

#define V_POT	        1.8 // Nominal setting of the potentiometer
#define V_R1            120.0
#define V_R2            8.2
#define V_A_S_1         ((uint16_t)(MAX_A_SAMPLE * (1.0 * (V_R2+V_POT) / (V_R1+V_R2+V_POT)) / AREF_LEVEL))

#define A_GAIN          20.0
#define A_POT           2.0 // Nominal setting of the potentiometer
#define A_SENSE_R       0.1
#define A_R1            68.0
#define A_R2            18.0
#define A_A_S_1         ((uint16_t)(MAX_A_SAMPLE * (1.0 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL))

#define TEST_NOISE      0.01   // Maximum noise level (V), using uniform distribution (ideally, should be Gaussian)

uint16_t adc_v_read(double v)
{
    double v_adc;
    double v_noise;
    uint16_t sample;
    
    v_noise = TEST_NOISE * ((2.0 * rand() / RAND_MAX) - 1.0);
    v_adc = (v * (V_R2+V_POT) / (120+V_R2+V_POT)) + v_noise;
    sample = MAX_SAMPLE *  v_adc / AREF_LEVEL;
    return sample;
}

uint16_t adc_a_read(double a)
{
    double v_adc;
    double v_noise;
    uint16_t sample;

    v_noise = TEST_NOISE * ((2.0 * rand() / RAND_MAX) - 1.0);
    v_adc = ((a * A_SENSE_R) * ((A_R2 + A_POT)/(A_R2 + A_POT + A_R1))) + v_noise;    
    sample = MAX_SAMPLE * A_GAIN * v_adc / AREF_LEVEL;
    return sample;
}

void fmt_voltage(uint16_t sample, char* buff)
{
    sample = (sample * 100UL) / (uint16_t)(V_A_S_1);
    buff[0] = 'U';
	buff[1] = '=';
	buff[2] = ' ';

    buff[3] = ' ';
    for (int i = 7;  sample || (i>3); i--, sample /= 10) {
        buff[i] = '0' + (sample % 10);
        if (i == 6) {
            buff[--i] = '.';
        }
    }

	buff[8] = ' ';
	buff[9] = 'V';
	buff[10] = '\0';
}


void fmt_ampere(uint16_t sample, char* buff)
{
    uint8_t ma = 0;
    
    if (sample < A_A_S_1) {
        sample = (sample * 10000UL) / (uint16_t)(A_A_S_1);
        ma = 1;
    }
    else {
        sample = (sample * 100UL) / (uint16_t)(A_A_S_1);
    }

    buff[0] = 'I';
	buff[1] = '=';
	buff[2] = ' ';

    buff[3] = ' ';
    buff[7] = '0';
    for (int i = (ma ? 6 : 7);  sample || (i>3); i--, sample /= 10) {
        buff[i] = '0' + (sample % 10);
        if (i == 6) {
            buff[--i] = '.';
        }
    }

	buff[8] = ma ? 'm' : ' ';
	buff[9] = 'A';
	buff[10] = '\0';

}

int main(int argc, char* argv[])
{
    double v = 0.0;
    double a = 0.0;
    char buff[16];
    
    srand(1976);    
    while (v < 32.0) {
        v += 0.01;
        uint16_t sample_acc = 0;
        for (int i=0; i<AVG_N_SAMPLES; i++) {
            sample_acc += (int)adc_v_read(v); 
        }
        fmt_voltage(sample_acc, buff);
        double vv = sample_acc * (V_R1+V_R2+V_POT) / (V_R2+V_POT) * AREF_LEVEL / MAX_A_SAMPLE;
        printf("%2.2f V: %2.2f V: %s [%5d]\n", v, vv, buff, (int)(sample_acc));
    }
    
    
    
    while (a < 5.0 ) {
        a += 0.001;
        uint16_t sample_acc = 0;
        for (int i=0; i<AVG_N_SAMPLES; i++) {
            sample_acc += (int)adc_a_read(a); 
        }
        fmt_ampere(sample_acc, buff);
        double aa = sample_acc * (A_R1+A_R2+A_POT) / (A_R2+A_POT) * AREF_LEVEL / MAX_A_SAMPLE / A_GAIN / A_SENSE_R;
        printf("%2.2f A: %2.2f A: %s [%5d]\n", a, aa, buff, (int)(sample_acc));
    }
    
    
    return 0;
}