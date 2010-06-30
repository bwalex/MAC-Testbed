interface PollNodeComm
{
	event result_t dataRequested(void *data);
	event result_t ackReceived(void *data);
	command result_t txData(void *data, uint8_t length);
	event result_t dataTxFailed();
}
