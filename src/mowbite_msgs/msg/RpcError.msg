# Based on xbot_msgs/xbot_mqtt (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
int16 ERROR_INVALID_JSON=-32700
int16 ERROR_INVALID_REQUEST=-32600
int16 ERROR_METHOD_NOT_FOUND=-32601
int16 ERROR_INVALID_PARAMS=-32602
int16 ERROR_INTERNAL=-32603

int16 code
string message
string id
# JSON text for the data of the error, e.g. {"reason": ..., "message": ...} of a refusal; empty: none.
string data
