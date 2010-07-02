configuration PollNode
{
   provides {
      interface SplitControl;
      interface PollNodeComm;
   }
}

implementation
{
   components PollNodeM, PhyRadio, RandomLFSR, TimerC, LedsC, PrecisionTimerC;
  
   SplitControl = PollNodeM;
   PollNodeComm = PollNodeM;
   
   // wiring to lower layers
   
   PollNodeM.PhyControl -> PhyRadio;
   PollNodeM.PhyState -> PhyRadio;
   //PollNodeM.CarrierSense -> PhyRadio;
   PollNodeM.PhyComm -> PhyRadio;
   PollNodeM.WakeupTimer -> TimerC.Timer[unique("Timer")];
   PollNodeM.Leds -> LedsC;
   PollNodeM.Timestamp -> PrecisionTimerC.PrecisionTimer[1];
   PollNodeM.TSControl -> PrecisionTimerC.StdControl;
}
