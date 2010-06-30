configuration PollHead
{
   provides {
      interface SplitControl;
      interface PollHeadComm;
   }
}

implementation
{
   components PollHeadM, PhyRadio, RandomLFSR, SingleTimer;
  
   SplitControl = PollHeadM;
   PollHeadComm = PollHeadM;
   
   // wiring to lower layers
   
   PollHeadM.PhyControl -> PhyRadio;
   PollHeadM.PhyState -> PhyRadio;
   //PollHeadM.CarrierSense -> PhyRadio;
   PollHeadM.PhyComm -> PhyRadio;
   //PollHeadM.Timer -> SingleTimer.Timer;
}
