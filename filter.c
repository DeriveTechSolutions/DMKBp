#include "port_filter.h"


double convert_range(double value, double in_hi, double in_lo, double out_hi, double out_lo)
{
    double slope = ((out_hi - out_lo)/ (in_hi - in_lo));
    double point = slope * (value - in_lo) + out_lo;
    return point;

}
void filterLowPassInit(filterLowPass *k, double sampleFreq, double cuttoff) {
    int i;
    k->fs = sampleFreq;
    k->fc = cuttoff;



    for ( i = 0; i < 2; i ++ ) {
        k->a[i] = 0.0;
        k->b[i] = 0.0;
        k->x[i] = 0.0;
        k->y[i] = 0.0;
    }

    filterSetCutoff(k,cuttoff);

}
void filterSetSinglePoleX(filterLowPass *k, double x)
{
    k->a[0] = 1 - x;
    k->b[1] = x;
}

double filterGetXDelay(uint32_t d)
{
    return exp((-1)/(double)d);
}

double filterGetFCDelay(double fc)
{
    return exp(-2*3.14159265*fc);
}

double convertHz2Norm(double fs, double f)
{
    double fNyq = fs/2.0;
    double norm = f/fNyq * 0.5;
    return norm;
}
void filterSetDelay(filterLowPass* k, uint32_t d)
{
    double x = filterGetXDelay(d);
    filterSetSinglePoleX(k,x);
}
void filterSetCutoff(filterLowPass* k, double fc)
{
    k->fc = fc;
    double norm = convertHz2Norm(k->fs,fc);
    filterSetSinglePoleX(k,filterGetFCDelay(norm));
}

double filterGetResult(filterLowPass *k)
{
    return k->lastResult;
}
double filterSingle(filterLowPass *k, double x)
{

    k->x[0] = x;
    k->y[0] = k->a[0]*k->x[0] + k->a[1]*k->x[1] + k->b[1]*k->y[1];

    k->lastResult = k->y[1];

    k->y[1] = k->y[0];
    k->x[1] = k->x[0];
    return k->lastResult;
}

