
configuration PhyRadio2
{
	provides {
		interface SplitControl;
		interface PhyState;
		interface PhyComm;
		interface CarrierSense;
		interface SignalStrength;
		interface CC2420Control;
		interface BackoffControl;
		interface PhyRadioControl;
	}
}

implementation
{
	components PhyRadio2M, CC2420ControlM, HPLCC2420C, TimerJiffyAsyncC, RandomLFSR, LedsC;

	SplitControl = PhyRadio2M;
	PhyComm = PhyRadio2M;
	//MacBackoff = PhyRadio2M;
	PhyState = PhyRadio2M;
	CarrierSense = PhyRadio2M;
	SignalStrength = PhyRadio2M;
	CC2420Control = CC2420ControlM;
	BackoffControl = PhyRadio2M;
	PhyRadioControl = PhyRadio2M;

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

  PhyRadio2M.CC2420SplitControl -> CC2420ControlM;
  PhyRadio2M.CC2420Control -> CC2420ControlM;
  PhyRadio2M.Random -> RandomLFSR;
  PhyRadio2M.TimerControl -> TimerJiffyAsyncC.StdControl;
  PhyRadio2M.BackoffTimerJiffy -> TimerJiffyAsyncC.TimerJiffyAsync;
  PhyRadio2M.HPLChipcon -> HPLCC2420C.HPLCC2420;
  PhyRadio2M.HPLChipconFIFO -> HPLCC2420C.HPLCC2420FIFO;
  PhyRadio2M.FIFOP -> HPLCC2420C.InterruptFIFOP;
  PhyRadio2M.SFD -> HPLCC2420C.CaptureSFD;

  CC2420ControlM.HPLChipconControl -> HPLCC2420C.StdControl;
  CC2420ControlM.HPLChipcon -> HPLCC2420C.HPLCC2420;
  CC2420ControlM.HPLChipconRAM -> HPLCC2420C.HPLCC2420RAM;
  CC2420ControlM.CCA -> HPLCC2420C.InterruptCCA;

  PhyRadio2M.Leds -> LedsC;
}
