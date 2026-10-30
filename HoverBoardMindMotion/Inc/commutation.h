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

//Output compare mode of channel k (0..2) in a step: the channel with only its low side enabled is forced
//active (with CCxE off, CCxN follows OCxREF itself: that low side stays on for the whole step), the others
//PWM1 (the high side chops). Before, all three were PWM1: the low side chopped together with the high side,
//in the off time the current freewheeled through two body diodes against the bus voltage and collapsed
//every period (2 V: 0.46 A instead of ~3 A; 18.5 % duty before the wheel turned). Now it freewheels
//through the low side that stays on and the other low side's body diode: continuous, no braking.
//All three low side outputs stay enabled (CCxNE) and are switched by the mode: a disabled output goes
//to its off-state level, which on this MM32 ignores the polarity bit (like the idle levels). With
//inverted low sides (invlowside) "disabled" was "on": the open phase's low side conducted (the motor
//braked itself every step: 6 W unloaded at 50 rpm, shaking) and so did the chopping phase's own (shoot
//through in every on-time). Channel with only CCxNE (low side): N follows OCxREF; with both (the
//chopping high side): N is its complement, the low side rectifies in the off-time.
#define BLOCK_CCER_LOW (TIM_CCER_CC1NEN|TIM_CCER_CC2NEN|TIM_CCER_CC3NEN)
static inline uint32_t BlockOcMode(uint32_t ccer, uint8_t k){
	uint32_t c = ccer>>(4*k)&(TIM_CCER_CC1EN|TIM_CCER_CC1NEN);
	return c==TIM_CCER_CC1NEN ? TIM_ForcedAction_Active : (c ? TIM_OCMode_PWM1 : TIM_ForcedAction_InActive);
}

//hall state C*4+B*2+A (hallpos(), 7-state backwards) -> step; 0 and 7 are not valid hall states
#define HALL_TO_POS_INIT { 0, 3, 5, 4, 1, 2, 6, 0 }

#endif
