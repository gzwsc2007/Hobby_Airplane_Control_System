// MESSAGE MagCal PACKING

#define MAVLINK_MSG_ID_MagCal 153

typedef struct __mavlink_magcal_t
{
 int16_t mx; ///< Magnetometer raw X reading 
 int16_t my; ///< Magnetometer raw Y reading 
 int16_t mz; ///< Magnetometer raw Z reading 
} mavlink_magcal_t;

#define MAVLINK_MSG_ID_MagCal_LEN 6
#define MAVLINK_MSG_ID_153_LEN 6

#define MAVLINK_MSG_ID_MagCal_CRC 75
#define MAVLINK_MSG_ID_153_CRC 75



#define MAVLINK_MESSAGE_INFO_MagCal { \
	"MagCal", \
	3, \
	{  { "mx", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_magcal_t, mx) }, \
         { "my", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_magcal_t, my) }, \
         { "mz", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_magcal_t, mz) }, \
         } \
}


/**
 * @brief Pack a magcal message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mx Magnetometer raw X reading 
 * @param my Magnetometer raw Y reading 
 * @param mz Magnetometer raw Z reading 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magcal_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t mx, int16_t my, int16_t mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MagCal_LEN];
	_mav_put_int16_t(buf, 0, mx);
	_mav_put_int16_t(buf, 2, my);
	_mav_put_int16_t(buf, 4, mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MagCal_LEN);
#else
	mavlink_magcal_t packet;
	packet.mx = mx;
	packet.my = my;
	packet.mz = mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MagCal_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MagCal;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MagCal_LEN, MAVLINK_MSG_ID_MagCal_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MagCal_LEN);
#endif
}

/**
 * @brief Pack a magcal message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mx Magnetometer raw X reading 
 * @param my Magnetometer raw Y reading 
 * @param mz Magnetometer raw Z reading 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magcal_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t mx,int16_t my,int16_t mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MagCal_LEN];
	_mav_put_int16_t(buf, 0, mx);
	_mav_put_int16_t(buf, 2, my);
	_mav_put_int16_t(buf, 4, mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MagCal_LEN);
#else
	mavlink_magcal_t packet;
	packet.mx = mx;
	packet.my = my;
	packet.mz = mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MagCal_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MagCal;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MagCal_LEN, MAVLINK_MSG_ID_MagCal_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MagCal_LEN);
#endif
}

/**
 * @brief Encode a magcal struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magcal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magcal_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magcal_t* magcal)
{
	return mavlink_msg_magcal_pack(system_id, component_id, msg, magcal->mx, magcal->my, magcal->mz);
}

/**
 * @brief Encode a magcal struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magcal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magcal_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magcal_t* magcal)
{
	return mavlink_msg_magcal_pack_chan(system_id, component_id, chan, msg, magcal->mx, magcal->my, magcal->mz);
}

/**
 * @brief Send a magcal message
 * @param chan MAVLink channel to send the message
 *
 * @param mx Magnetometer raw X reading 
 * @param my Magnetometer raw Y reading 
 * @param mz Magnetometer raw Z reading 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magcal_send(mavlink_channel_t chan, int16_t mx, int16_t my, int16_t mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MagCal_LEN];
	_mav_put_int16_t(buf, 0, mx);
	_mav_put_int16_t(buf, 2, my);
	_mav_put_int16_t(buf, 4, mz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, buf, MAVLINK_MSG_ID_MagCal_LEN, MAVLINK_MSG_ID_MagCal_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, buf, MAVLINK_MSG_ID_MagCal_LEN);
#endif
#else
	mavlink_magcal_t packet;
	packet.mx = mx;
	packet.my = my;
	packet.mz = mz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, (const char *)&packet, MAVLINK_MSG_ID_MagCal_LEN, MAVLINK_MSG_ID_MagCal_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, (const char *)&packet, MAVLINK_MSG_ID_MagCal_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MagCal_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magcal_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t mx, int16_t my, int16_t mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, mx);
	_mav_put_int16_t(buf, 2, my);
	_mav_put_int16_t(buf, 4, mz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, buf, MAVLINK_MSG_ID_MagCal_LEN, MAVLINK_MSG_ID_MagCal_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, buf, MAVLINK_MSG_ID_MagCal_LEN);
#endif
#else
	mavlink_magcal_t *packet = (mavlink_magcal_t *)msgbuf;
	packet->mx = mx;
	packet->my = my;
	packet->mz = mz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, (const char *)packet, MAVLINK_MSG_ID_MagCal_LEN, MAVLINK_MSG_ID_MagCal_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MagCal, (const char *)packet, MAVLINK_MSG_ID_MagCal_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MagCal UNPACKING


/**
 * @brief Get field mx from magcal message
 *
 * @return Magnetometer raw X reading 
 */
static inline int16_t mavlink_msg_magcal_get_mx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field my from magcal message
 *
 * @return Magnetometer raw Y reading 
 */
static inline int16_t mavlink_msg_magcal_get_my(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field mz from magcal message
 *
 * @return Magnetometer raw Z reading 
 */
static inline int16_t mavlink_msg_magcal_get_mz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a magcal message into a struct
 *
 * @param msg The message to decode
 * @param magcal C-struct to decode the message contents into
 */
static inline void mavlink_msg_magcal_decode(const mavlink_message_t* msg, mavlink_magcal_t* magcal)
{
#if MAVLINK_NEED_BYTE_SWAP
	magcal->mx = mavlink_msg_magcal_get_mx(msg);
	magcal->my = mavlink_msg_magcal_get_my(msg);
	magcal->mz = mavlink_msg_magcal_get_mz(msg);
#else
	memcpy(magcal, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MagCal_LEN);
#endif
}
