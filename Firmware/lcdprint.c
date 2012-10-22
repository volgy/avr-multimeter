#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


#define AREF_LEVEL		2.56
#define ADC_BITS        10
#define MAX_SAMPLE  	((1<<ADC_BITS)-1)
#define AVG_N_SAMPLES   1<<((sizeof(uint16_t)*8) - ADC_BITS)
#define MAX_A_SAMPLE    ((1<<(sizeof(uint16_t)*8))-1)

#define V_POT	        1.8 // Nominal setting of the potentiometer
#define V_R1            120.0
#define V_R2            8.2
#define V_A_S_10        (MAX_A_SAMPLE * (10.0 * (V_R2+V_POT) / (120+V_R2+V_POT)) / AREF_LEVEL)
#define V_A_S_1         (MAX_A_SAMPLE * (1.0 * (V_R2+V_POT) / (120+V_R2+V_POT)) / AREF_LEVEL)
#define V_A_S_0_1       (MAX_A_SAMPLE * (0.1 * (V_R2+V_POT) / (120+V_R2+V_POT)) / AREF_LEVEL)
#define V_A_S_0_01      (MAX_A_SAMPLE * (0.01 * (V_R2+V_POT) / (120+V_R2+V_POT)) / AREF_LEVEL)

#define A_GAIN          20.0
#define A_POT           2.0 // Nominal setting of the potentiometer
#define A_SENSE_R       0.1
#define A_R1            68.0
#define A_R2            18.0
#define A_A_S_10        (MAX_A_SAMPLE * (10.0 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL)
#define A_A_S_1         (MAX_A_SAMPLE * (1.0 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL)
#define A_A_S_0_1       (MAX_A_SAMPLE * (0.1 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL)
#define A_A_S_0_01      (MAX_A_SAMPLE * (0.01 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL)
#define A_A_S_0_001     (MAX_A_SAMPLE * (0.001 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL)

#define TEST_NOISE      0.01   // Maximum noise level (V), using uniform distribution (ideally, should be Gaussian)



uint16_t adc_v_read(double v)
{
    double v_adc;
    double v_noise;
    uint16_t sample;
    
    v_noise = TEST_NOISE * ((2 * rand() / RAND_MAX) - 1);
    v_adc = (v * (V_R2+V_POT) / (120+V_R2+V_POT)) + v_noise;
    sample = MAX_SAMPLE *  v_adc / AREF_LEVEL;
    return sample;
}

uint16_t adc_a_read(double a)
{
    double v_adc;
    double v_noise;
    uint16_t sample;

    v_noise = TEST_NOISE * ((2 * rand() / RAND_MAX) - 1);
    v_adc = ((a * A_SENSE_R) * ((A_R2 + A_POT)/(A_R2 + A_POT + A_R1))) + v_noise;    
    sample = MAX_SAMPLE * A_GAIN * v_adc / AREF_LEVEL;
    return sample;
}



void lcd_voltage(uint16_t sample, char* buff)
{
    uint16_t r = sample;
    uint16_t d;

    buff[0] = 'U';
    buff[1] = '=';
    buff[2] = ' ';
    
    d = r / V_A_S_10;
    if (d) {
        buff[3] = '0' + d;  // 0 < d < 10, TODO: check & protect
    } 
    else {
        buff[3] = ' ';
    }
    r -= d * V_A_S_10;
    
    d = r / V_A_S_1;
    buff[4] = '0' + d;  // 0 <= d < 10, TODO: check & protect
    r -= d * V_A_S_1;

    buff[5] = '.';

    d = r / V_A_S_0_1;
    buff[6] = '0' + d;  // 0 <= d < 10, TODO: check & protect
    r -= d * V_A_S_0_1;

    d = r / V_A_S_0_01;
    buff[7] = '0' + d;  // 0 <= d < 10, TODO: check & protect

    buff[8] = ' ';
    buff[9] = 'V';
    buff[10] = '\0';
}

void lcd_ampere(uint16_t sample, char* buff)
{
    uint16_t r = sample;
    uint16_t d;

    buff[0] = 'I';
    buff[1] = '=';

    if (sample < A_A_S_1) {
        buff[6] = '\0';

    }
    else {
        buff[2] = ' ';

        d = r / A_A_S_10;
        if (d) {
            buff[3] = '0' + d;  // 0 < d < 10, TODO: check & protect
        } 
        else {
            buff[3] = ' ';
        }
        r -= d * A_A_S_10;
        
        d = r / A_A_S_1;
        buff[4] = '0' + d;  // 0 <= d < 10, TODO: check & protect
        r -= d * A_A_S_1;
    
        buff[5] = '.';
    
        d = r / A_A_S_0_1;
        buff[6] = '0' + d;  // 0 <= d < 10, TODO: check & protect
        r -= d * A_A_S_0_1;
    
        d = r / A_A_S_0_01;
        buff[7] = '0' + d;  // 0 <= d < 10, TODO: check & protect
    
        buff[8] = ' ';
        buff[9] = 'A';
        buff[10] = '\0';
    }
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
        lcd_voltage(sample_acc, buff);
        double vv = sample_acc * (120+V_R2+V_POT) / (V_R2+V_POT) * AREF_LEVEL / MAX_A_SAMPLE;
        printf("%2.2f V: %2.2f V: %s [%5d]\n", v, vv, buff, (int)(sample_acc));
    }
    
    
    
    while (a < 5.0 ) {
        a += 0.001;
        uint16_t sample_acc = 0;
        for (int i=0; i<AVG_N_SAMPLES; i++) {
            sample_acc += (int)adc_a_read(a); 
        }
        lcd_ampere(sample_acc, buff);
        double aa = sample_acc * (A_R1+A_R2+V_POT) / (A_R2+V_POT) * AREF_LEVEL / MAX_A_SAMPLE / A_GAIN / A_SENSE_R;
        printf("%2.2f A: %2.2f A: %s [%5d]\n", a, aa, buff, (int)(sample_acc));
    }
    
    
    return 0;
}