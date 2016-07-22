//
//  Biquad.h
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the Biquad code:
//  http://www.earlevel.com/main/2012/11/25/biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//

/*changes by tero:

replace double -> float
funcs pow->powf etc
move enum inside class
examples
test on arm

BTR lots of great stuff at:
http://www.earlevel.com/

Such as FIR generator:
http://www.earlevel.com/main/2010/12/05/building-a-windowed-sinc-filter/

About biquad filters:
http://www.earlevel.com/main/2003/02/28/biquads/

Great filter desging tool online
http://www.micromodeler.com/dsp/

*/

/*
example2

	//tested and works on arm. add code size 12-13kb -O2

    Biquad lpFilter;
    lpFilter.setBiquad(Biquad::bq_type_lowpass, 1000.0/20000.0, 0.707, 0); //0.707 gives non peaking freq response butterworth, but overshoots in time space
    lpFilter.setBiquad(Biquad::bq_type_lowpass_bessel, 1000.0/20000.0, anyvalue, 0); //creates 2nd order bessel. dont use setAnything funcs after this

    float out[50];
    for (int idx = 0; idx < 50; idx++)
        out[idx] = lpFilter.process(1);//step response

example3 - test amplitude response in various filters

    Biquad lpFilter;
    volatile float freqtotest=1.0/32.0;
    while(1)
    {
		//lpFilter.setBiquad(Biquad::bq_type_peak, freqtotest, 1, -3);
		lpFilter.setBiquad(Biquad::bq_type_lowpass_bessel, freqtotest, any, any);
		//lpFilter.setBiquad(Biquad::bq_type_lowpass, freqtotest/2, 0.707, any);
		volatile float out[300];
		volatile float in[300];
		volatile float max=0;

		for (int idx = 0; idx < 300; idx++)
		{
			in[idx]=sinf(((float)idx)*2*3.141*freqtotest);
			out[idx] = lpFilter.process(in[idx]);
			if(idx>200&&out[idx]>max)max=out[idx];
		}
		NOP;//max here should be 0.707 if -3db
    }
*/

/*
Q and fc for bessel lpf/hpf. Bessel is optimum for time response and flat group delay.

https://gist.github.com/endolith/4982787

Calculate the f multipler and Q values for implementing Nth-order Bessel
filters as biquad second-order sections.

Based on description at http://freeverb3.sourceforge.net/iir_filter.shtml
For highpass filter, use fc/fmultiplier instead of fc*fmultiplier
Originally I back-calculated from the denominators produced by the bessel()
filter design tool, which stops at order 25.

Then I made a function to output Bessel polynomials directly, which can be
used to calculate Q, but not f. (TODO: Create real Bessel filters directly
and calculate f.) The two methods disagree with each other somewhat above
8th order. I'm not sure which is more accurate.

Also, these are bilinear-transformed, so they're only good below fs/4
(https://gist.github.com/endolith/4964212)


Q for N =
1: --------------
2: 0.57735026919
3: -------------- 0.691046625825
4: 0.805538281842 0.521934581669
5: -------------- 0.916477373948 0.563535620851
6: 1.02331395383 0.611194546878 0.510317824749
7: -------------- 1.12625754198 0.660821389297 0.5323556979
8: 1.22566942541 0.710852074442 0.559609164796 0.505991069397

f multiplier to get same asymptotes as Butterworth (LPF and HPF are phase-matched), for N =
1: 1.0*
2: 1.0
3: 0.941600026533* 1.03054454544
4: 1.05881751607 0.944449808226
5: 0.926442077388* 1.08249898167 0.959761595159
6: 1.10221694805 0.977488555538 0.928156550439
7: 0.919487155649* 1.11880560415 0.994847495138 0.936949152329
8: 1.13294518316 1.01102810214 0.948341760923 0.920583104484

f multiplier to get -3 dB at fc, for N =
1: 1.0*
2: 1.27201964951
3: 1.32267579991* 1.44761713315
4: 1.60335751622 1.43017155999
5: 1.50231627145* 1.75537777664 1.5563471223
6: 1.9047076123 1.68916826762 1.60391912877
7: 1.68436817927* 2.04949090027 1.82241747886 1.71635604487
8: 2.18872623053 1.95319575902 1.8320926012 1.77846591177
 */

#ifndef Biquad_h
#define Biquad_h



class Biquad {
public:

	enum {
	    bq_type_lowpass = 0,
	    bq_type_highpass,
	    bq_type_lowpass_bessel,
	    bq_type_highpass_bessel,
	    bq_type_bandpass,
	    bq_type_notch,
	    bq_type_peak,
	    bq_type_lowshelf,
	    bq_type_highshelf,
		bq_type_passthrough, /*no operation*/
		bq_type_lowpass_1st_order,/*note 1st order filters have finite attenuation at rejection band (0 or inf depending if hpf/lpf)*/
		bq_type_highpass_1st_order/*note 1st order filters have finite attenuation at rejection band (0 or inf depending if hpf/lpf)*/
	};

    Biquad();
    Biquad(int type, float Fc, float Q, float peakGainDB);
    ~Biquad();

    void setPeakGain(float peakGainDB);
    void setBiquad(int type, float Fc, float Q, float peakGain);
    float process(float in);
    //optimizer version, works only for 1st order LPF & HPF & passthrough
    float process1stOrder(float in);
    void reset()
    {
    	z1=z2=0;
    }

protected:
    void calcBiquad(void);

    int type;
    float a0, a1, a2, b1, b2;
    float Fc, Q, peakGain;
    float z1, z2;
};

inline float Biquad::process(float in) {
    float out = in * a0 + z1;
    z1 = in * a1 + z2 - b1 * out;
    z2 = in * a2 - b2 * out;
    return out;
}

inline float Biquad::process1stOrder(float in) {
	//	we know a1 = a2 = b2 = 0;
    return z1 = in*a0 -z1*b1;
}


#endif // Biquad_h
