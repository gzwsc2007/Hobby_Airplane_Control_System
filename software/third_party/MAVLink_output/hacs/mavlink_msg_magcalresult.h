// MESSAGE MagCalResult PACKING

#define MAVLINK_MSG_ID_MagCalResult 156

typedef struct __mavlink_magcalresult_t
{
 float B_field; ///< radius of the calibrated sphere 
 float hard_iron[3]; ///< hard iron offsets 
 float soft_iron[9]; ///< soft iron matrix (W_inverted) 
} mavlink_magcalresult_t;

#define MAVLINK_MSG_ID_MagCalResult_LEN 52
#define MAVLINK_MSG_ID_156_LEN 52

#define MAVLINK_MSG_ID_MagCalResult_CRC 96
#define MAVLINK_MSG_ID_156_CRC 96

#define MAVLINK_MSG_MagCalResult_FIELD_HARD_IRON_LEN 3
#define MAVLINK_MSG_MagCalResult_FIELD_SOFT_IRON_LEN 9

#define MAVLINK_MESSAGE_INFO_MagCalResult { \
	"MagCalResult", \
	3, \
	{  { "B_field", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_magcalresult_t, B_field) }, \
         { "hard_iron", NULL, MAVLINK_TYPE_FLOAT, 3, 4, offsetof(mavlink_magcalresult_t, hard_iron) }, \
         { "soft_iron", NULL, MAVLINK_TYPE_FLOAT, 9, 16, offsetof(mavlink_magcalresult_t, soft_iron) }, \
         } \
}


/**
 * @brief Pack a magcalresult message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param B_field radius of the calibrated sphere 
 * @param hard_iron hard iron offsets 
 * @param soft_iron soft iron matrix (W_inverted) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magcalresult_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float B_field, const float *hard_iron, const float *soft_iron)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MagCalResult_LEN];
	_mav_put_float(buf, 0, B_field);
	_mav_put_float_array(buf, 4, hard_iron, 3);
	_mav_put_float_array(buf, 16, soft_iron, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MagCalResult_LEN);
#else
	mavlink_magcalresult_t packet;
	packet.B_field = B_field;
	mav_array_memcpy(packet.hard_iron, hard_iron, sizeof(float)*3);
	mav_array_memcpy(packet.soft_iron, soft_iron, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MagCalResult;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MagCalResult_LEN, MAVLINK_MSG_ID_MagCalResult_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
}

/**
 * @brief Pack a magcalresult message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param B_field radius of the calibrated sphere 
 * @param hard_iron hard iron offsets 
 * @param soft_iron soft iron matrix (W_inverted) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magcalresult_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float B_field,const float *hard_iron,const float *soft_iron)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MagCalResult_LEN];
	_mav_put_float(buf, 0, B_field);
	_mav_put_float_array(buf, 4, hard_iron, 3);
	_mav_put_float_array(buf, 16, soft_iron, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MagCalResult_LEN);
#else
	mavlink_magcalresult_t packet;
	packet.B_field = B_field;
	mav_array_memcpy(packet.hard_iron, hard_iron, sizeof(float)*3);
	mav_array_memcpy(packet.soft_iron, soft_iron, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MagCalResult;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MagCalResult_LEN, MAVLINK_MSG_ID_MagCalResult_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
}

/**
 * @brief Encode a magcalresult struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magcalresult C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magcalresult_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magcalresult_t* magcalresult)
{
	return mavlink_msg_magcalresult_pack(system_id, component_id, msg, magcalresult->B_field, magcalresult->hard_iron, magcalresult->soft_iron);
}

/**
 * @brief Encode a magcalresult struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magcalresult C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magcalresult_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magcalresult_t* magcalresult)
{
	return mavlink_msg_magcalresult_pack_chan(system_id, component_id, chan, msg, magcalresult->B_field, magcalresult->hard_iron, magcalresult->soft_iron);
}

/**
 * @brief Send a magcalresult message
 * @param chan MAVLink channel to send the message
 *
 * @param B_field radius of the calibrated sphere 
 * @param hard_iron hard iron offsets 
 * @param soft_iron soft iron matrix (W_inverted) 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magcalresult_send(mavlink_channel_t chan, float B_field, const float *hard_iron, const float *soft_iron)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MagCalResult_LEN];
	_mav_put_float(buf, 0, B_field);
	_mav_put_float_array(buf, 4, hard_iron, 3);
	_mav_put_float_array(buf, 16, soft_iron, 9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, buf, MAVLINK_MSG_ID_MagCalResult_LEN, MAVLINK_MSG_ID_MagCalResult_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, buf, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
#else
	mavlink_magcalresult_t packet;
	packet.B_field = B_field;
	mav_array_memcpy(packet.hard_iron, hard_iron, sizeof(float)*3);
	mav_array_memcpy(packet.soft_iron, soft_iron, sizeof(float)*9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, (const char *)&packet, MAVLINK_MSG_ID_MagCalResult_LEN, MAVLINK_MSG_ID_MagCalResult_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, (const char *)&packet, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MagCalResult_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magcalresult_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float B_field, const float *hard_iron, const float *soft_iron)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, B_field);
	_mav_put_float_array(buf, 4, hard_iron, 3);
	_mav_put_float_array(buf, 16, soft_iron, 9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, buf, MAVLINK_MSG_ID_MagCalResult_LEN, MAVLINK_MSG_ID_MagCalResult_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, buf, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
#else
	mavlink_magcalresult_t *packet = (mavlink_magcalresult_t *)msgbuf;
	packet->B_field = B_field;
	mav_array_memcpy(packet->hard_iron, hard_iron, sizeof(float)*3);
	mav_array_memcpy(packet->soft_iron, soft_iron, sizeof(float)*9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, (const char *)packet, MAVLINK_MSG_ID_MagCalResult_LEN, MAVLINK_MSG_ID_MagCalResult_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCalResult, (const char *)packet, MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MagCalResult UNPACKING


/**
 * @brief Get field B_field from magcalresult message
 *
 * @return radius of the calibrated sphere 
 */
static inline float mavlink_msg_magcalresult_get_B_field(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field hard_iron from magcalresult message
 *
 * @return hard iron offsets 
 */
static inline uint16_t mavlink_msg_magcalresult_get_hard_iron(const mavlink_message_t* msg, float *hard_iron)
{
	return _MAV_RETURN_float_array(msg, hard_iron, 3,  4);
}

/**
 * @brief Get field soft_iron from magcalresult message
 *
 * @return soft iron matrix (W_inverted) 
 */
static inline uint16_t mavlink_msg_magcalresult_get_soft_iron(const mavlink_message_t* msg, float *soft_iron)
{
	return _MAV_RETURN_float_array(msg, soft_iron, 9,  16);
}

/**
 * @brief Decode a magcalresult message into a struct
 *
 * @param msg The message to decode
 * @param magcalresult C-struct to decode the message contents into
 */
static inline void mavlink_msg_magcalresult_decode(const mavlink_message_t* msg, mavlink_magcalresult_t* magcalresult)
{
#if MAVLINK_NEED_BYTE_SWAP
	magcalresult->B_field = mavlink_msg_magcalresult_get_B_field(msg);
	mavlink_msg_magcalresult_get_hard_iron(msg, magcalresult->hard_iron);
	mavlink_msg_magcalresult_get_soft_iron(msg, magcalresult->soft_iron);
#else
	memcpy(magcalresult, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MagCalResult_LEN);
#endif
}
