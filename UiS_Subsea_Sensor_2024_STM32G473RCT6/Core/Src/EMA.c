#include "EMA.h"

void EMA_Init(EMA *filt, float alpha) {

	/* Setter filterkoeffisient */
	EMA_SetAlpha(filt, alpha);

	/* Clear filter output */
	filt->out = 0.0f;
}


void EMA_SetAlpha(EMA *filt, float alpha){

	/* Alfa må være mellom 0 og 1 */
	if (alpha > 1.0f) {
		alpha = 1.0f;

	} else if (alpha < 0.0f){
		alpha = 0.0f;

	}

	filt->alpha = alpha;
}


float EMA_Update(EMA *filt, float input) {
    filt->out = filt->alpha * input + (1.0f - filt->alpha) * filt->out;
    return filt->out;
}

