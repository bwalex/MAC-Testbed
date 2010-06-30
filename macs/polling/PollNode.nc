configuration PollNode
{
   provides {
      interface SplitControl;
      interface PollNodeComm;
   }
}

implementation
{
   components PollNodeM, PhyRadio, RandomLFSR, SingleTimer;
  
   SplitControl = PollNodeM;
   PollHeadComm = PollNodeM;
   
   // wiring to lower layers
   
   PollNodeM.PhyControl -> PhyRadio;
   PollNodeM.PhyState -> PhyRadio;
   PollNodeM.CarrierSense -> PhyRadio;
   PollNodeM.PhyComm -> PhyRadio;
   PollNodeM.Timer -> SingleTimer.Timer;
}
