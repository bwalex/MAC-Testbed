interface PrecisionTimer
{
    async command uint32_t getTime32();

    /**
     * This method sets a match register such that when OSCR0 reaches
     * the value, an interrupt is fired.
     */
    async command result_t setAlarm(uint32_t val);

    async command result_t clearAlarm();

    /**
     * This event is triggered by the interrupt mentioned above.
     */
    async event result_t alarmFired(uint32_t val);
}
