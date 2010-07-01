interface PollHeadComm
{
	event result_t requestDataDone(uint8_t id, void *data, uint8_t error);
	command result_t requestData(uint8_t u_node_id, void *data, uint8_t length);
	command result_t cancelRequest();
	command result_t setSleepInterval(uint32_t sleep_jiffies);
}
