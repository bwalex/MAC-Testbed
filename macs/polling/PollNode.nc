configuration PollNode
{
   provides {
      interface SplitControl;
      interface PollNodeComm;
   }
}

implementation
{
   components PollNodeM, PhyRadio, RandomLFSR, SingleTimer, LedsC;
  
   SplitControl = PollNodeM;
   PollNodeComm = PollNodeM;
   
   // wiring to lower layers
   
   PollNodeM.PhyControl -> PhyRadio;
   PollNodeM.PhyState -> PhyRadio;
   //PollNodeM.CarrierSense -> PhyRadio;
   PollNodeM.PhyComm -> PhyRadio;
   PollNodeM.Timer -> SingleTimer.Timer;
   PollNodeM.Leds -> LedsC;
}
