#ifndef EMA_H
#define EMA_H

typedef struct {
	/* Filterkoeffisient [0-1] */
	float alpha;

	/* Filter output */
	float out;
} EMA;

void EMA_Init(EMA *filt, float alpha);
void EMA_SetAlpha(EMA *filt, float alpha);
float EMA_Update(EMA *filt, float input);


#endif
