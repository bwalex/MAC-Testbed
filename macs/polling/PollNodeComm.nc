interface PollNodeComm
{
	event result_t PollNodeComm.dataRequested();
	event result_t PollNodeComm.ackReceived();
	command result_t PollNodeComm.txData(void *data, uint8_t length);
	event result_t PollNodeComm.dataTxFailed();
}
