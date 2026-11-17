# The inputs of the platform one by one, on hw/inputs (latched; published at the start and with every change). The
# flags of hw/status stay the sum by role. A driver may leave the topic out: the setup program then checks by role.
builtin_interfaces/Time stamp

# The driver started with inputs.learn (REASON_INPUTS_LEARNING of hw/status).
bool learning
# In the order of the table, candidates after it.
HardwareInput[] inputs
