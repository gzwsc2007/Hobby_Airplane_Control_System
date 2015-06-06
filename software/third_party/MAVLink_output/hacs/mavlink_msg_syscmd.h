// MESSAGE SysCmd PACKING

#define MAVLINK_MSG_ID_SysCmd 155

typedef struct __mavlink_syscmd_t
{
 uint8_t cmd_flag; ///< A set of well-knwon command flags 
} mavlink_syscmd_t;

#define MAVLINK_MSG_ID_SysCmd_LEN 1
#define MAVLINK_MSG_ID_155_LEN 1

#define MAVLINK_MSG_ID_SysCmd_CRC 138
#define MAVLINK_MSG_ID_155_CRC 138



#define MAVLINK_MESSAGE_INFO_SysCmd { \
	"SysCmd", \
	1, \
	{  { "cmd_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_syscmd_t, cmd_flag) }, \
         } \
}


/**
 * @brief Pack a syscmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd_flag A set of well-knwon command flags 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_syscmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t cmd_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SysCmd_LEN];
	_mav_put_uint8_t(buf, 0, cmd_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SysCmd_LEN);
#else
	mavlink_syscmd_t packet;
	packet.cmd_flag = cmd_flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SysCmd_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SysCmd;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SysCmd_LEN, MAVLINK_MSG_ID_SysCmd_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SysCmd_LEN);
#endif
}

/**
 * @brief Pack a syscmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd_flag A set of well-knwon command flags 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_syscmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t cmd_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SysCmd_LEN];
	_mav_put_uint8_t(buf, 0, cmd_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SysCmd_LEN);
#else
	mavlink_syscmd_t packet;
	packet.cmd_flag = cmd_flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SysCmd_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SysCmd;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SysCmd_LEN, MAVLINK_MSG_ID_SysCmd_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SysCmd_LEN);
#endif
}

/**
 * @brief Encode a syscmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param syscmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_syscmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_syscmd_t* syscmd)
{
	return mavlink_msg_syscmd_pack(system_id, component_id, msg, syscmd->cmd_flag);
}

/**
 * @brief Encode a syscmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param syscmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_syscmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_syscmd_t* syscmd)
{
	return mavlink_msg_syscmd_pack_chan(system_id, component_id, chan, msg, syscmd->cmd_flag);
}

/**
 * @brief Send a syscmd message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd_flag A set of well-knwon command flags 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_syscmd_send(mavlink_channel_t chan, uint8_t cmd_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SysCmd_LEN];
	_mav_put_uint8_t(buf, 0, cmd_flag);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, buf, MAVLINK_MSG_ID_SysCmd_LEN, MAVLINK_MSG_ID_SysCmd_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, buf, MAVLINK_MSG_ID_SysCmd_LEN);
#endif
#else
	mavlink_syscmd_t packet;
	packet.cmd_flag = cmd_flag;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, (const char *)&packet, MAVLINK_MSG_ID_SysCmd_LEN, MAVLINK_MSG_ID_SysCmd_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, (const char *)&packet, MAVLINK_MSG_ID_SysCmd_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SysCmd_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_syscmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, cmd_flag);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, buf, MAVLINK_MSG_ID_SysCmd_LEN, MAVLINK_MSG_ID_SysCmd_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, buf, MAVLINK_MSG_ID_SysCmd_LEN);
#endif
#else
	mavlink_syscmd_t *packet = (mavlink_syscmd_t *)msgbuf;
	packet->cmd_flag = cmd_flag;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, (const char *)packet, MAVLINK_MSG_ID_SysCmd_LEN, MAVLINK_MSG_ID_SysCmd_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysCmd, (const char *)packet, MAVLINK_MSG_ID_SysCmd_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SysCmd UNPACKING


/**
 * @brief Get field cmd_flag from syscmd message
 *
 * @return A set of well-knwon command flags 
 */
static inline uint8_t mavlink_msg_syscmd_get_cmd_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a syscmd message into a struct
 *
 * @param msg The message to decode
 * @param syscmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_syscmd_decode(const mavlink_message_t* msg, mavlink_syscmd_t* syscmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	syscmd->cmd_flag = mavlink_msg_syscmd_get_cmd_flag(msg);
#else
	memcpy(syscmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SysCmd_LEN);
#endif
}
