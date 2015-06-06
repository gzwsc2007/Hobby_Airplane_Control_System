// MESSAGE SysState PACKING

#define MAVLINK_MSG_ID_SysState 154

typedef struct __mavlink_sysstate_t
{
 uint8_t mode; ///<  Current mode of the HACS system 
} mavlink_sysstate_t;

#define MAVLINK_MSG_ID_SysState_LEN 1
#define MAVLINK_MSG_ID_154_LEN 1

#define MAVLINK_MSG_ID_SysState_CRC 196
#define MAVLINK_MSG_ID_154_CRC 196



#define MAVLINK_MESSAGE_INFO_SysState { \
	"SysState", \
	1, \
	{  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_sysstate_t, mode) }, \
         } \
}


/**
 * @brief Pack a sysstate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Current mode of the HACS system 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sysstate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SysState_LEN];
	_mav_put_uint8_t(buf, 0, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SysState_LEN);
#else
	mavlink_sysstate_t packet;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SysState_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SysState;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SysState_LEN, MAVLINK_MSG_ID_SysState_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SysState_LEN);
#endif
}

/**
 * @brief Pack a sysstate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  Current mode of the HACS system 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sysstate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SysState_LEN];
	_mav_put_uint8_t(buf, 0, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SysState_LEN);
#else
	mavlink_sysstate_t packet;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SysState_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SysState;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SysState_LEN, MAVLINK_MSG_ID_SysState_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SysState_LEN);
#endif
}

/**
 * @brief Encode a sysstate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sysstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sysstate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sysstate_t* sysstate)
{
	return mavlink_msg_sysstate_pack(system_id, component_id, msg, sysstate->mode);
}

/**
 * @brief Encode a sysstate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sysstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sysstate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sysstate_t* sysstate)
{
	return mavlink_msg_sysstate_pack_chan(system_id, component_id, chan, msg, sysstate->mode);
}

/**
 * @brief Send a sysstate message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  Current mode of the HACS system 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sysstate_send(mavlink_channel_t chan, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SysState_LEN];
	_mav_put_uint8_t(buf, 0, mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, buf, MAVLINK_MSG_ID_SysState_LEN, MAVLINK_MSG_ID_SysState_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, buf, MAVLINK_MSG_ID_SysState_LEN);
#endif
#else
	mavlink_sysstate_t packet;
	packet.mode = mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, (const char *)&packet, MAVLINK_MSG_ID_SysState_LEN, MAVLINK_MSG_ID_SysState_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, (const char *)&packet, MAVLINK_MSG_ID_SysState_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SysState_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sysstate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, buf, MAVLINK_MSG_ID_SysState_LEN, MAVLINK_MSG_ID_SysState_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, buf, MAVLINK_MSG_ID_SysState_LEN);
#endif
#else
	mavlink_sysstate_t *packet = (mavlink_sysstate_t *)msgbuf;
	packet->mode = mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, (const char *)packet, MAVLINK_MSG_ID_SysState_LEN, MAVLINK_MSG_ID_SysState_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SysState, (const char *)packet, MAVLINK_MSG_ID_SysState_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SysState UNPACKING


/**
 * @brief Get field mode from sysstate message
 *
 * @return  Current mode of the HACS system 
 */
static inline uint8_t mavlink_msg_sysstate_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a sysstate message into a struct
 *
 * @param msg The message to decode
 * @param sysstate C-struct to decode the message contents into
 */
static inline void mavlink_msg_sysstate_decode(const mavlink_message_t* msg, mavlink_sysstate_t* sysstate)
{
#if MAVLINK_NEED_BYTE_SWAP
	sysstate->mode = mavlink_msg_sysstate_get_mode(msg);
#else
	memcpy(sysstate, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SysState_LEN);
#endif
}
