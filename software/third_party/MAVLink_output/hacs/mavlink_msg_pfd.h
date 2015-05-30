// MESSAGE PFD PACKING

#define MAVLINK_MSG_ID_PFD 150

typedef struct __mavlink_pfd_t
{
 int16_t roll; ///< Roll angle in 0.01 degrees. Positive means rolling CW (right down). 
 int16_t pitch; ///< Pitch angle in 0.01 degrees. Positive means pitching up. 
 int16_t yaw; ///< Yaw angle (magnetic heading) in 0.01 degrees. Positive means yawing CW (turn to the right). 
 int16_t altitude; ///< altitude (AGL) in meters. 
 int16_t airspeed; ///< airspeed in 0.1 m/s. 
 int16_t battI; ///< Battery current in 0.01 A. 
} mavlink_pfd_t;

#define MAVLINK_MSG_ID_PFD_LEN 12
#define MAVLINK_MSG_ID_150_LEN 12

#define MAVLINK_MSG_ID_PFD_CRC 6
#define MAVLINK_MSG_ID_150_CRC 6



#define MAVLINK_MESSAGE_INFO_PFD { \
	"PFD", \
	6, \
	{  { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_pfd_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_pfd_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_pfd_t, yaw) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_pfd_t, altitude) }, \
         { "airspeed", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_pfd_t, airspeed) }, \
         { "battI", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_pfd_t, battI) }, \
         } \
}


/**
 * @brief Pack a pfd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle in 0.01 degrees. Positive means rolling CW (right down). 
 * @param pitch Pitch angle in 0.01 degrees. Positive means pitching up. 
 * @param yaw Yaw angle (magnetic heading) in 0.01 degrees. Positive means yawing CW (turn to the right). 
 * @param altitude altitude (AGL) in meters. 
 * @param airspeed airspeed in 0.1 m/s. 
 * @param battI Battery current in 0.01 A. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pfd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t roll, int16_t pitch, int16_t yaw, int16_t altitude, int16_t airspeed, int16_t battI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PFD_LEN];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_int16_t(buf, 6, altitude);
	_mav_put_int16_t(buf, 8, airspeed);
	_mav_put_int16_t(buf, 10, battI);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PFD_LEN);
#else
	mavlink_pfd_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.altitude = altitude;
	packet.airspeed = airspeed;
	packet.battI = battI;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PFD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PFD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PFD_LEN, MAVLINK_MSG_ID_PFD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PFD_LEN);
#endif
}

/**
 * @brief Pack a pfd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle in 0.01 degrees. Positive means rolling CW (right down). 
 * @param pitch Pitch angle in 0.01 degrees. Positive means pitching up. 
 * @param yaw Yaw angle (magnetic heading) in 0.01 degrees. Positive means yawing CW (turn to the right). 
 * @param altitude altitude (AGL) in meters. 
 * @param airspeed airspeed in 0.1 m/s. 
 * @param battI Battery current in 0.01 A. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pfd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t roll,int16_t pitch,int16_t yaw,int16_t altitude,int16_t airspeed,int16_t battI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PFD_LEN];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_int16_t(buf, 6, altitude);
	_mav_put_int16_t(buf, 8, airspeed);
	_mav_put_int16_t(buf, 10, battI);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PFD_LEN);
#else
	mavlink_pfd_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.altitude = altitude;
	packet.airspeed = airspeed;
	packet.battI = battI;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PFD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PFD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PFD_LEN, MAVLINK_MSG_ID_PFD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PFD_LEN);
#endif
}

/**
 * @brief Encode a pfd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pfd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pfd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pfd_t* pfd)
{
	return mavlink_msg_pfd_pack(system_id, component_id, msg, pfd->roll, pfd->pitch, pfd->yaw, pfd->altitude, pfd->airspeed, pfd->battI);
}

/**
 * @brief Encode a pfd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pfd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pfd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pfd_t* pfd)
{
	return mavlink_msg_pfd_pack_chan(system_id, component_id, chan, msg, pfd->roll, pfd->pitch, pfd->yaw, pfd->altitude, pfd->airspeed, pfd->battI);
}

/**
 * @brief Send a pfd message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle in 0.01 degrees. Positive means rolling CW (right down). 
 * @param pitch Pitch angle in 0.01 degrees. Positive means pitching up. 
 * @param yaw Yaw angle (magnetic heading) in 0.01 degrees. Positive means yawing CW (turn to the right). 
 * @param altitude altitude (AGL) in meters. 
 * @param airspeed airspeed in 0.1 m/s. 
 * @param battI Battery current in 0.01 A. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pfd_send(mavlink_channel_t chan, int16_t roll, int16_t pitch, int16_t yaw, int16_t altitude, int16_t airspeed, int16_t battI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PFD_LEN];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_int16_t(buf, 6, altitude);
	_mav_put_int16_t(buf, 8, airspeed);
	_mav_put_int16_t(buf, 10, battI);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, buf, MAVLINK_MSG_ID_PFD_LEN, MAVLINK_MSG_ID_PFD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, buf, MAVLINK_MSG_ID_PFD_LEN);
#endif
#else
	mavlink_pfd_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.altitude = altitude;
	packet.airspeed = airspeed;
	packet.battI = battI;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, (const char *)&packet, MAVLINK_MSG_ID_PFD_LEN, MAVLINK_MSG_ID_PFD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, (const char *)&packet, MAVLINK_MSG_ID_PFD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PFD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pfd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t roll, int16_t pitch, int16_t yaw, int16_t altitude, int16_t airspeed, int16_t battI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_int16_t(buf, 6, altitude);
	_mav_put_int16_t(buf, 8, airspeed);
	_mav_put_int16_t(buf, 10, battI);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, buf, MAVLINK_MSG_ID_PFD_LEN, MAVLINK_MSG_ID_PFD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, buf, MAVLINK_MSG_ID_PFD_LEN);
#endif
#else
	mavlink_pfd_t *packet = (mavlink_pfd_t *)msgbuf;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->altitude = altitude;
	packet->airspeed = airspeed;
	packet->battI = battI;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, (const char *)packet, MAVLINK_MSG_ID_PFD_LEN, MAVLINK_MSG_ID_PFD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PFD, (const char *)packet, MAVLINK_MSG_ID_PFD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PFD UNPACKING


/**
 * @brief Get field roll from pfd message
 *
 * @return Roll angle in 0.01 degrees. Positive means rolling CW (right down). 
 */
static inline int16_t mavlink_msg_pfd_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field pitch from pfd message
 *
 * @return Pitch angle in 0.01 degrees. Positive means pitching up. 
 */
static inline int16_t mavlink_msg_pfd_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field yaw from pfd message
 *
 * @return Yaw angle (magnetic heading) in 0.01 degrees. Positive means yawing CW (turn to the right). 
 */
static inline int16_t mavlink_msg_pfd_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field altitude from pfd message
 *
 * @return altitude (AGL) in meters. 
 */
static inline int16_t mavlink_msg_pfd_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field airspeed from pfd message
 *
 * @return airspeed in 0.1 m/s. 
 */
static inline int16_t mavlink_msg_pfd_get_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field battI from pfd message
 *
 * @return Battery current in 0.01 A. 
 */
static inline int16_t mavlink_msg_pfd_get_battI(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a pfd message into a struct
 *
 * @param msg The message to decode
 * @param pfd C-struct to decode the message contents into
 */
static inline void mavlink_msg_pfd_decode(const mavlink_message_t* msg, mavlink_pfd_t* pfd)
{
#if MAVLINK_NEED_BYTE_SWAP
	pfd->roll = mavlink_msg_pfd_get_roll(msg);
	pfd->pitch = mavlink_msg_pfd_get_pitch(msg);
	pfd->yaw = mavlink_msg_pfd_get_yaw(msg);
	pfd->altitude = mavlink_msg_pfd_get_altitude(msg);
	pfd->airspeed = mavlink_msg_pfd_get_airspeed(msg);
	pfd->battI = mavlink_msg_pfd_get_battI(msg);
#else
	memcpy(pfd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PFD_LEN);
#endif
}
