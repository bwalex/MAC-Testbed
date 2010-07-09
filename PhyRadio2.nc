
configuration PhyRadio2
{
	provides {
		interface SplitControl;
		interface PhyComm;
		interface CC2420Control;
		interface MacBackoff;
	}
}

implementation
{
	components PhyRadioM2, CC2420ControlM, HPLCC2420C, TimerJiffyAsyncC, RandomLFSR, LedsC;

	SplitControl = PhyRadioM2;
	PhyComm = PhyRadioM2;
	MacBackoff = PhyRadioM2;
	//PhyState = PhyRadioM;
	//CarrierSense = PhyRadioM;
	//SignalStrength = PhyRadioM;
	CC2420Control = CC2420ControlM;
	//BackoffControl = PhyRadioM;
	//PhyRadioControl = PhyRadioM;

#if 0
	PhyRadioM.CC2420SplitControl -> CC2420ControlM;
	PhyRadioM.CC2420Control -> CC2420ControlM;
	PhyRadioM.HPL -> HPLCC2420C.HPLCC2420;
	PhyRadioM.FIFO -> HPLCC2420C.HPLCC2420FIFO;
	PhyRadioM.FIFOP -> HPLCC2420C.InterruptFIFOP;
	PhyRadioM.SFD -> HPLCC2420C.CaptureSFD;
	PhyRadioM.BackoffTimerControl -> TimerJiffyAsyncC.StdControl;
	PhyRadioM.BackoffTimer -> TimerJiffyAsyncC.TimerJiffyAsync;
	PhyRadioM.Random -> RandomLFSR;

	CC2420ControlM.HPLChipconControl -> HPLCC2420C.StdControl;
	CC2420ControlM.HPLChipcon -> HPLCC2420C.HPLCC2420;
	CC2420ControlM.HPLChipconRAM -> HPLCC2420C.HPLCC2420RAM;
	CC2420ControlM.CCA -> HPLCC2420C.InterruptCCA;
#endif

  PhyRadioM2.CC2420SplitControl -> CC2420ControlM;
  PhyRadioM2.CC2420Control -> CC2420ControlM;
  PhyRadioM2.Random -> RandomLFSR;
  PhyRadioM2.TimerControl -> TimerJiffyAsyncC.StdControl;
  PhyRadioM2.BackoffTimerJiffy -> TimerJiffyAsyncC.TimerJiffyAsync;
  PhyRadioM2.HPLChipcon -> HPLCC2420C.HPLCC2420;
  PhyRadioM2.HPLChipconFIFO -> HPLCC2420C.HPLCC2420FIFO;
  PhyRadioM2.FIFOP -> HPLCC2420C.InterruptFIFOP;
  PhyRadioM2.SFD -> HPLCC2420C.CaptureSFD;

  CC2420ControlM.HPLChipconControl -> HPLCC2420C.StdControl;
  CC2420ControlM.HPLChipcon -> HPLCC2420C.HPLCC2420;
  CC2420ControlM.HPLChipconRAM -> HPLCC2420C.HPLCC2420RAM;
  CC2420ControlM.CCA -> HPLCC2420C.InterruptCCA;

  PhyRadioM2.Leds -> LedsC;
}
