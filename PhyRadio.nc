
configuration PhyRadio
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
	components PhyRadioM, CC2420ControlM, HPLCC2420C, TimerJiffyAsyncC, RandomLFSR;

	SplitControl = PhyRadioM;
	PhyComm = PhyRadioM;
	PhyState = PhyRadioM;
	CarrierSense = PhyRadioM;
	SignalStrength = PhyRadioM;
	CC2420Control = CC2420ControlM;
	BackoffControl = PhyRadioM;
	PhyRadioControl = PhyRadioM;

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
}
