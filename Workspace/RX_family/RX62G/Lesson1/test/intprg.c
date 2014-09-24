/************************************************************************
*
* Device     : RX/RX600/RX62G
*
* File Name  : intprg.c
*
* Abstract   : Interrupt Program.
*
* History    : 1.00  (2012-06-12)  [Hardware Manual Revision : 1.00]
*            : 1.10  (2013-02-18)  [Hardware Manual Revision : 1.00]
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2013 (2012) Renesas Electronics Corporation and
* Renesas Solutions Corp. All rights reserved.
*
************************************************************************/

#include <machine.h>
#include "vect.h"
#pragma section IntPRG

// Exception(Supervisor Instruction)
void Excep_SuperVisorInst(void){/* brk(); */}

// Exception(Access Instruction)
void Excep_AccessInst(void){/* brk(); */}

// Exception(Undefined Instruction)
void Excep_UndefinedInst(void){/* brk(); */}

// Exception(Floating Point)
void Excep_FloatingPoint(void){/* brk(); */}

// NMI
void NonMaskableInterrupt(void){/* brk(); */}

// Dummy
void Dummy(void){/* brk(); */}

// BRK
void Excep_BRK(void){ wait(); }

// BSC BUSERR
void Excep_BSC_BUSERR(void){ }

// FCUIF FIFERR
void Excep_FCUIF_FIFERR(void){ }

// FCUIF FRDYI
void Excep_FCUIF_FRDYI(void){ }

// ICU SWINT
void Excep_ICU_SWINT(void){ }

// CMT0 CMI0
void Excep_CMT0_CMI0(void){ }

// CMT1 CMI1
void Excep_CMT1_CMI1(void){ }

// CMT2 CMI2
void Excep_CMT2_CMI2(void){ }

// CMT3 CMI3
void Excep_CMT3_CMI3(void){ }

// RSPI0 SPEI0
void Excep_RSPI0_SPEI0(void){ }

// RSPI0 SPRI0
void Excep_RSPI0_SPRI0(void){ }

// RSPI0 SPTI0
void Excep_RSPI0_SPTI0(void){ }

// RSPI0 SPII0
void Excep_RSPI0_SPII0(void){ }

// CAN0 ERS0
void Excep_CAN0_ERS0(void){ }

// CAN0 RXF0
void Excep_CAN0_RXF0(void){ }

// CAN0 TXF0
void Excep_CAN0_TXF0(void){ }

// CAN0 RXM0
void Excep_CAN0_RXM0(void){ }

// CAN0 TXM0
void Excep_CAN0_TXM0(void){ }

// ICU IRQ0
void Excep_ICU_IRQ0(void){ }

// ICU IRQ1
void Excep_ICU_IRQ1(void){ }

// ICU IRQ2
void Excep_ICU_IRQ2(void){ }

// ICU IRQ3
void Excep_ICU_IRQ3(void){ }

// ICU IRQ4
void Excep_ICU_IRQ4(void){ }

// ICU IRQ5
void Excep_ICU_IRQ5(void){ }

// ICU IRQ6
void Excep_ICU_IRQ6(void){ }

// ICU IRQ7
void Excep_ICU_IRQ7(void){ }

// WDT WOVI
void Excep_WDT_WOVI(void){ }

// AD0 ADI0
void Excep_AD0_ADI0(void){ }

// S12AD0 S12ADI0
void Excep_S12AD0_S12ADI0(void){ }

// S12AD1 S12ADI1
void Excep_S12AD1_S12ADI1(void){ }

// CMPB CMPI
void Excep_CMPB_CMPI(void){ }

// MTU0 TGIA0
void Excep_MTU0_TGIA0(void){ }

// MTU0 TGIB0
void Excep_MTU0_TGIB0(void){ }

// MTU0 TGIC0
void Excep_MTU0_TGIC0(void){ }

// MTU0 TGID0
void Excep_MTU0_TGID0(void){ }

// MTU0 TCIV0
void Excep_MTU0_TCIV0(void){ }

// MTU0 TGIE0
void Excep_MTU0_TGIE0(void){ }

// MTU0 TGIF0
void Excep_MTU0_TGIF0(void){ }

// MTU1 TGIA1
void Excep_MTU1_TGIA1(void){ }

// MTU1 TGIB1
void Excep_MTU1_TGIB1(void){ }

// MTU1 TCIV1
void Excep_MTU1_TCIV1(void){ }

// MTU1 TCIU1
void Excep_MTU1_TCIU1(void){ }

// MTU2 TGIA2
void Excep_MTU2_TGIA2(void){ }

// MTU2 TGIB2
void Excep_MTU2_TGIB2(void){ }

// MTU2 TCIV2
void Excep_MTU2_TCIV2(void){ }

// MTU2 TCIU2
void Excep_MTU2_TCIU2(void){ }

// MTU3 TGIA3
void Excep_MTU3_TGIA3(void){ }

// MTU3 TGIB3
void Excep_MTU3_TGIB3(void){ }

// MTU3 TGIC3
void Excep_MTU3_TGIC3(void){ }

// MTU3 TGID3
void Excep_MTU3_TGID3(void){ }

// MTU3 TCIV3
void Excep_MTU3_TCIV3(void){ }

// MTU4 TGIA4
void Excep_MTU4_TGIA4(void){ }

// MTU4 TGIB4
void Excep_MTU4_TGIB4(void){ }

// MTU4 TGIC4
void Excep_MTU4_TGIC4(void){ }

// MTU4 TGID4
void Excep_MTU4_TGID4(void){ }

// MTU4 TCIV4
void Excep_MTU4_TCIV4(void){ }

// MTU5 TGIU5
void Excep_MTU5_TGIU5(void){ }

// MTU5 TGIV5
void Excep_MTU5_TGIV5(void){ }

// MTU5 TGIW5
void Excep_MTU5_TGIW5(void){ }

// MTU6 TGIA6
void Excep_MTU6_TGIA6(void){ }

// MTU6 TGIB6
void Excep_MTU6_TGIB6(void){ }

// MTU6 TGIC6
void Excep_MTU6_TGIC6(void){ }

// MTU6 TGID6
void Excep_MTU6_TGID6(void){ }

// MTU6 TCIV6
void Excep_MTU6_TCIV6(void){ }

// MTU7 TGIA7
void Excep_MTU7_TGIA7(void){ }

// MTU7 TGIB7
void Excep_MTU7_TGIB7(void){ }

// MTU7 TGIC7
void Excep_MTU7_TGIC7(void){ }

// MTU7 TGID7
void Excep_MTU7_TGID7(void){ }

// MTU7 TCIV7
void Excep_MTU7_TCIV7(void){ }

// POE OEI1
void Excep_POE_OEI1(void){ }

// POE OEI2
void Excep_POE_OEI2(void){ }

// POE OEI3
void Excep_POE_OEI3(void){ }

// POE OEI4
void Excep_POE_OEI4(void){ }

// GPT0 GTCIA0
void Excep_GPT0_GTCIA0(void){ }

// GPT0 GTCIB0
void Excep_GPT0_GTCIB0(void){ }

// GPT0 GTCIC0
void Excep_GPT0_GTCIC0(void){ }

// GPT0 GTCIE0
void Excep_GPT0_GTCIE0(void){ }

// GPT0 GTCIV0
void Excep_GPT0_GTCIV0(void){ }

// GPT0 LOCO1
void Excep_GPT0_LOCO1(void){ }

// GPT1 GTCIA1
void Excep_GPT1_GTCIA1(void){ }

// GPT1 GTCIB1
void Excep_GPT1_GTCIB1(void){ }

// GPT1 GTCIC1
void Excep_GPT1_GTCIC1(void){ }

// GPT1 GTCIE1
void Excep_GPT1_GTCIE1(void){ }

// GPT1 GTCIV1
void Excep_GPT1_GTCIV1(void){ }

// GPT2 GTCIA2
void Excep_GPT2_GTCIA2(void){ }

// GPT2 GTCIB2
void Excep_GPT2_GTCIB2(void){ }

// GPT2 GTCIC2
void Excep_GPT2_GTCIC2(void){ }

// GPT2 GTCIE2
void Excep_GPT2_GTCIE2(void){ }

// GPT2 GTCIV2
void Excep_GPT2_GTCIV2(void){ }

// GPT3 GTCIA3
void Excep_GPT3_GTCIA3(void){ }

// GPT3 GTCIB3
void Excep_GPT3_GTCIB3(void){ }

// GPT3 GTCIC3
void Excep_GPT3_GTCIC3(void){ }

// GPT3 GTCIE3
void Excep_GPT3_GTCIE3(void){ }

// GPT3 GTCIV3
void Excep_GPT3_GTCIV3(void){ }

// SCI0 ERI0
void Excep_SCI0_ERI0(void){ }

// SCI0 RXI0
void Excep_SCI0_RXI0(void){ }

// SCI0 TXI0
void Excep_SCI0_TXI0(void){ }

// SCI0 TEI0
void Excep_SCI0_TEI0(void){ }

// SCI1 ERI1
void Excep_SCI1_ERI1(void){ }

// SCI1 RXI1
void Excep_SCI1_RXI1(void){ }

// SCI1 TXI1
void Excep_SCI1_TXI1(void){ }

// SCI1 TEI1
void Excep_SCI1_TEI1(void){ }

// SCI2 ERI2
void Excep_SCI2_ERI2(void){ }

// SCI2 RXI2
void Excep_SCI2_RXI2(void){ }

// SCI2 TXI2
void Excep_SCI2_TXI2(void){ }

// SCI2 TEI2
void Excep_SCI2_TEI2(void){ }

// RIIC0 ICEEI0
void Excep_RIIC0_ICEEI0(void){ }

// RIIC0 ICRXI0
void Excep_RIIC0_ICRXI0(void){ }

// RIIC0 ICTXI0
void Excep_RIIC0_ICTXI0(void){ }

// RIIC0 ICTEI0
void Excep_RIIC0_ICTEI0(void){ }

// LIN0 LIN0
void Excep_LIN0_LIN0(void){ }
