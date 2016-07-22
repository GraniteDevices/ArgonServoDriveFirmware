//
//  Biquad.cpp
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the Biquad code:
//  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//


#include <math.h>
#include "Biquad.h"

Biquad::Biquad() {
    type = bq_type_lowpass;
    a0 = 1.0;
    a1 = a2 = b1 = b2 = 0.0;
    Fc = 0.50;
    Q = 0.707;
    peakGain = 0.0;
    z1 = z2 = 0.0;
}

Biquad::Biquad(int type, float Fc, float Q, float peakGainDB) {
    setBiquad(type, Fc, Q, peakGainDB);
    z1 = z2 = 0.0;
}

Biquad::~Biquad() {
}


void Biquad::setPeakGain(float peakGainDB) {
    this->peakGain = peakGainDB;
    calcBiquad();
}

void Biquad::setBiquad(int type, float Fc, float Q, float peakGainDB) {

	//conver to 2nd order bessel
	switch (type) {
	        case bq_type_lowpass_bessel:
	        	Q=0.57735026919;
	        	Fc*=1.27201964951;
	            break;

	        case bq_type_highpass_bessel:
	        	Q=0.57735026919;
	        	Fc/=1.27201964951;
	        	break;
	}

    this->type = type;
    this->Q = Q;
    this->Fc = Fc;
    z1 = z2 = 0.0;
    setPeakGain(peakGainDB);
}

void Biquad::calcBiquad(void) {
    float norm;
    float V = powf(10, fabsf(peakGain) / 20.0);
    float K = tanf(3.14159265358979 * Fc);
    switch (this->type) {
		case bq_type_lowpass_1st_order:
			b1 = powf(2.71828182845905,-2.0 * 3.14159265358979 * Fc);
			a0 = 1.0 - b1;
			b1 = -b1;
			a1 = a2 = b2 = 0;
			break;
		case bq_type_highpass_1st_order:
			b1 = -powf(2.71828182845905,-2.0 * 3.14159265358979 * (0.5 - Fc));
			a0 = 1.0 + b1;
			b1 = -b1;
			a1 = a2 = b2 = 0;
		break;
        case bq_type_lowpass:
        case bq_type_lowpass_bessel:
            norm = 1 / (1 + K / Q + K * K);
            a0 = K * K * norm;
            a1 = 2 * a0;
            a2 = a0;
            b1 = 2 * (K * K - 1) * norm;
            b2 = (1 - K / Q + K * K) * norm;
            break;

        case bq_type_highpass:
        case bq_type_highpass_bessel:
            norm = 1 / (1 + K / Q + K * K);
            a0 = 1 * norm;
            a1 = -2 * a0;
            a2 = a0;
            b1 = 2 * (K * K - 1) * norm;
            b2 = (1 - K / Q + K * K) * norm;
            break;

        case bq_type_bandpass:
            norm = 1 / (1 + K / Q + K * K);
            a0 = K / Q * norm;
            a1 = 0;
            a2 = -a0;
            b1 = 2 * (K * K - 1) * norm;
            b2 = (1 - K / Q + K * K) * norm;
            break;

        case bq_type_notch:
            norm = 1 / (1 + K / Q + K * K);
            a0 = (1 + K * K) * norm;
            a1 = 2 * (K * K - 1) * norm;
            a2 = a0;
            b1 = a1;
            b2 = (1 - K / Q + K * K) * norm;
            break;

        case bq_type_peak:
            if (peakGain >= 0) {    // boost
                norm = 1 / (1 + 1/Q * K + K * K);
                a0 = (1 + V/Q * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - V/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1 - 1/Q * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (1 + V/Q * K + K * K);
                a0 = (1 + 1/Q * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - 1/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1 - V/Q * K + K * K) * norm;
            }
            break;
        case bq_type_lowshelf:
            if (peakGain >= 0) {    // boost
                norm = 1 / (1 + sqrtf(2) * K + K * K);
                a0 = (1 + sqrtf(2*V) * K + V * K * K) * norm;
                a1 = 2 * (V * K * K - 1) * norm;
                a2 = (1 - sqrtf(2*V) * K + V * K * K) * norm;
                b1 = 2 * (K * K - 1) * norm;
                b2 = (1 - sqrtf(2) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (1 + sqrtf(2*V) * K + V * K * K);
                a0 = (1 + sqrtf(2) * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - sqrtf(2) * K + K * K) * norm;
                b1 = 2 * (V * K * K - 1) * norm;
                b2 = (1 - sqrtf(2*V) * K + V * K * K) * norm;
            }
            break;
        case bq_type_highshelf:
            if (peakGain >= 0) {    // boost
                norm = 1 / (1 + sqrtf(2) * K + K * K);
                a0 = (V + sqrtf(2*V) * K + K * K) * norm;
                a1 = 2 * (K * K - V) * norm;
                a2 = (V - sqrtf(2*V) * K + K * K) * norm;
                b1 = 2 * (K * K - 1) * norm;
                b2 = (1 - sqrtf(2) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (V + sqrtf(2*V) * K + K * K);
                a0 = (1 + sqrtf(2) * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - sqrtf(2) * K + K * K) * norm;
                b1 = 2 * (K * K - V) * norm;
                b2 = (V - sqrtf(2*V) * K + K * K) * norm;
            }
            break;
        case bq_type_passthrough:
        default:
        	a0=1;
        	a1=a2=b1=b2=0;
            break;
    }

    return;
}
