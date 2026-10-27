#ifndef COMMUTATION_H
#define COMMUTATION_H

//Block commutation tables, shared by the main firmware (bldc.c) and PinFinder (its hall detection and test
//rotation must commutate exactly like the firmware that boots from the result).

//TIM1 CCER enables per step: the output of one channel (CCxE: its high side) and the complementary output
//of another (CCxNE: its low side), the third channel off. Phases A,B,C: +1 high side, -1 low side.
#define BLOCK_CCER_INIT { 0, \
	TIM_CCER_CC3EN|TIM_CCER_CC1NEN,    /*1: -1,0,1*/ \
	TIM_CCER_CC3EN|TIM_CCER_CC2NEN,    /*2: 0,-1,1*/ \
	TIM_CCER_CC1EN|TIM_CCER_CC2NEN,    /*3: 1,-1,0*/ \
	TIM_CCER_CC1EN|TIM_CCER_CC3NEN,    /*4: 1,0,-1*/ \
	TIM_CCER_CC2EN|TIM_CCER_CC3NEN,    /*5: 0,1,-1*/ \
	TIM_CCER_CC2EN|TIM_CCER_CC1NEN,    /*6: -1,1,0*/ \
}

//hall state C*4+B*2+A (hallpos(), 7-state backwards) -> step; 0 and 7 are not valid hall states
#define HALL_TO_POS_INIT { 0, 3, 5, 4, 1, 2, 6, 0 }

#endif
