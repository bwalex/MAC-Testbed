configuration PollNode
{
   provides {
      interface SplitControl;
      interface PollNodeComm;
   }
}

implementation
{
   components PollNodeM, PhyRadio, RandomLFSR, TimerC, LedsC, SysTimeC;
  
   SplitControl = PollNodeM;
   PollNodeComm = PollNodeM;
   
   // wiring to lower layers
   
   PollNodeM.PhyControl -> PhyRadio;
   PollNodeM.PhyState -> PhyRadio;
   //PollNodeM.CarrierSense -> PhyRadio;
   PollNodeM.PhyComm -> PhyRadio;
   PollNodeM.WakeupTimer -> TimerC.Timer[unique("Timer")];
   PollNodeM.Leds -> LedsC;
   PollNodeM.Timestamp -> SysTimeC;
   PollNodeM.TSControl -> SysTimeC.StdControl;
}
