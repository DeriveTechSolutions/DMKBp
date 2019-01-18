/*
 * port_filter.h
 *
 *  Created on: 12-Oct-2018
 *      Author: Hrishikesh-derive
 */

#ifndef PORT_FILTER_H_
#define PORT_FILTER_H_

#ifndef FILTER_H
#define FILTER_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>



typedef struct filter_tag
{
    uint32_t n;
    double a[2];
    double b[2];
    double x[2];
    double y[2];
    double lastResult;
    double f;
    double gain;
    double fs;
    double fc;
}filterLowPass;

void filterLowPassInit(filterLowPass *k, double sampleFreq, double cuttoff);
void filterSetCutoff(filterLowPass* k, double fc);
double filterGetResult(filterLowPass *k);
double filterSingle(filterLowPass *k, double x);


#endif //FILTER_H




#endif /* PORT_FILTER_H_ */
