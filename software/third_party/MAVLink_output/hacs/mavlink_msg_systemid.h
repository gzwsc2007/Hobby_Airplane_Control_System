// MESSAGE SystemID PACKING

#define MAVLINK_MSG_ID_SystemID 152

typedef struct __mavlink_systemid_t
{
 int16_t u_a; ///< Aileron control input in raw PWM value. Positive means right wing down. 
 int16_t u_e; ///< Elevator control input in raw PWM value. Positive means pitching up. 
 int16_t u_r; ///< Rudder control input in raw PWM value. Positive means turning to the right. 
 int16_t ax; ///< Acceleration on the body x-axis in 0.01g. Positive front. 
 int16_t ay; ///< Acceleration on the body y-axis in 0.01g. Positive right. 
 int16_t az; ///< Acceleration on the body z-axis in 0.01g. Positive down. 
 int16_t p; ///< Roll rate in 0.1 deg/s. Same sign-convention as roll angle. 
 int16_t q; ///< Pitch rate in 0.1 deg/s. Same sign-convention as pitch angle. 
 int16_t r; ///< Yaw rate in 0.1 deg/s. Same sign-convention as yaw angle. 
} mavlink_systemid_t;

#define MAVLINK_MSG_ID_SystemID_LEN 18
#define MAVLINK_MSG_ID_152_LEN 18

#define MAVLINK_MSG_ID_SystemID_CRC 168
#define MAVLINK_MSG_ID_152_CRC 168



#define MAVLINK_MESSAGE_INFO_SystemID { \
	"SystemID", \
	9, \
	{  { "u_a", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_systemid_t, u_a) }, \
         { "u_e", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_systemid_t, u_e) }, \
         { "u_r", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_systemid_t, u_r) }, \
         { "ax", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_systemid_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_systemid_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_systemid_t, az) }, \
         { "p", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_systemid_t, p) }, \
         { "q", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_systemid_t, q) }, \
         { "r", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_systemid_t, r) }, \
         } \
}


/**
 * @brief Pack a systemid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param u_a Aileron control input in raw PWM value. Positive means right wing down. 
 * @param u_e Elevator control input in raw PWM value. Positive means pitching up. 
 * @param u_r Rudder control input in raw PWM value. Positive means turning to the right. 
 * @param ax Acceleration on the body x-axis in 0.01g. Positive front. 
 * @param ay Acceleration on the body y-axis in 0.01g. Positive right. 
 * @param az Acceleration on the body z-axis in 0.01g. Positive down. 
 * @param p Roll rate in 0.1 deg/s. Same sign-convention as roll angle. 
 * @param q Pitch rate in 0.1 deg/s. Same sign-convention as pitch angle. 
 * @param r Yaw rate in 0.1 deg/s. Same sign-convention as yaw angle. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_systemid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t u_a, int16_t u_e, int16_t u_r, int16_t ax, int16_t ay, int16_t az, int16_t p, int16_t q, int16_t r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SystemID_LEN];
	_mav_put_int16_t(buf, 0, u_a);
	_mav_put_int16_t(buf, 2, u_e);
	_mav_put_int16_t(buf, 4, u_r);
	_mav_put_int16_t(buf, 6, ax);
	_mav_put_int16_t(buf, 8, ay);
	_mav_put_int16_t(buf, 10, az);
	_mav_put_int16_t(buf, 12, p);
	_mav_put_int16_t(buf, 14, q);
	_mav_put_int16_t(buf, 16, r);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SystemID_LEN);
#else
	mavlink_systemid_t packet;
	packet.u_a = u_a;
	packet.u_e = u_e;
	packet.u_r = u_r;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.p = p;
	packet.q = q;
	packet.r = r;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SystemID_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SystemID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SystemID_LEN, MAVLINK_MSG_ID_SystemID_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SystemID_LEN);
#endif
}

/**
 * @brief Pack a systemid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param u_a Aileron control input in raw PWM value. Positive means right wing down. 
 * @param u_e Elevator control input in raw PWM value. Positive means pitching up. 
 * @param u_r Rudder control input in raw PWM value. Positive means turning to the right. 
 * @param ax Acceleration on the body x-axis in 0.01g. Positive front. 
 * @param ay Acceleration on the body y-axis in 0.01g. Positive right. 
 * @param az Acceleration on the body z-axis in 0.01g. Positive down. 
 * @param p Roll rate in 0.1 deg/s. Same sign-convention as roll angle. 
 * @param q Pitch rate in 0.1 deg/s. Same sign-convention as pitch angle. 
 * @param r Yaw rate in 0.1 deg/s. Same sign-convention as yaw angle. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_systemid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t u_a,int16_t u_e,int16_t u_r,int16_t ax,int16_t ay,int16_t az,int16_t p,int16_t q,int16_t r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SystemID_LEN];
	_mav_put_int16_t(buf, 0, u_a);
	_mav_put_int16_t(buf, 2, u_e);
	_mav_put_int16_t(buf, 4, u_r);
	_mav_put_int16_t(buf, 6, ax);
	_mav_put_int16_t(buf, 8, ay);
	_mav_put_int16_t(buf, 10, az);
	_mav_put_int16_t(buf, 12, p);
	_mav_put_int16_t(buf, 14, q);
	_mav_put_int16_t(buf, 16, r);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SystemID_LEN);
#else
	mavlink_systemid_t packet;
	packet.u_a = u_a;
	packet.u_e = u_e;
	packet.u_r = u_r;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.p = p;
	packet.q = q;
	packet.r = r;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SystemID_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SystemID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SystemID_LEN, MAVLINK_MSG_ID_SystemID_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SystemID_LEN);
#endif
}

/**
 * @brief Encode a systemid struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param systemid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_systemid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_systemid_t* systemid)
{
	return mavlink_msg_systemid_pack(system_id, component_id, msg, systemid->u_a, systemid->u_e, systemid->u_r, systemid->ax, systemid->ay, systemid->az, systemid->p, systemid->q, systemid->r);
}

/**
 * @brief Encode a systemid struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param systemid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_systemid_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_systemid_t* systemid)
{
	return mavlink_msg_systemid_pack_chan(system_id, component_id, chan, msg, systemid->u_a, systemid->u_e, systemid->u_r, systemid->ax, systemid->ay, systemid->az, systemid->p, systemid->q, systemid->r);
}

/**
 * @brief Send a systemid message
 * @param chan MAVLink channel to send the message
 *
 * @param u_a Aileron control input in raw PWM value. Positive means right wing down. 
 * @param u_e Elevator control input in raw PWM value. Positive means pitching up. 
 * @param u_r Rudder control input in raw PWM value. Positive means turning to the right. 
 * @param ax Acceleration on the body x-axis in 0.01g. Positive front. 
 * @param ay Acceleration on the body y-axis in 0.01g. Positive right. 
 * @param az Acceleration on the body z-axis in 0.01g. Positive down. 
 * @param p Roll rate in 0.1 deg/s. Same sign-convention as roll angle. 
 * @param q Pitch rate in 0.1 deg/s. Same sign-convention as pitch angle. 
 * @param r Yaw rate in 0.1 deg/s. Same sign-convention as yaw angle. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_systemid_send(mavlink_channel_t chan, int16_t u_a, int16_t u_e, int16_t u_r, int16_t ax, int16_t ay, int16_t az, int16_t p, int16_t q, int16_t r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SystemID_LEN];
	_mav_put_int16_t(buf, 0, u_a);
	_mav_put_int16_t(buf, 2, u_e);
	_mav_put_int16_t(buf, 4, u_r);
	_mav_put_int16_t(buf, 6, ax);
	_mav_put_int16_t(buf, 8, ay);
	_mav_put_int16_t(buf, 10, az);
	_mav_put_int16_t(buf, 12, p);
	_mav_put_int16_t(buf, 14, q);
	_mav_put_int16_t(buf, 16, r);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, buf, MAVLINK_MSG_ID_SystemID_LEN, MAVLINK_MSG_ID_SystemID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, buf, MAVLINK_MSG_ID_SystemID_LEN);
#endif
#else
	mavlink_systemid_t packet;
	packet.u_a = u_a;
	packet.u_e = u_e;
	packet.u_r = u_r;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.p = p;
	packet.q = q;
	packet.r = r;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, (const char *)&packet, MAVLINK_MSG_ID_SystemID_LEN, MAVLINK_MSG_ID_SystemID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, (const char *)&packet, MAVLINK_MSG_ID_SystemID_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SystemID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_systemid_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t u_a, int16_t u_e, int16_t u_r, int16_t ax, int16_t ay, int16_t az, int16_t p, int16_t q, int16_t r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, u_a);
	_mav_put_int16_t(buf, 2, u_e);
	_mav_put_int16_t(buf, 4, u_r);
	_mav_put_int16_t(buf, 6, ax);
	_mav_put_int16_t(buf, 8, ay);
	_mav_put_int16_t(buf, 10, az);
	_mav_put_int16_t(buf, 12, p);
	_mav_put_int16_t(buf, 14, q);
	_mav_put_int16_t(buf, 16, r);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, buf, MAVLINK_MSG_ID_SystemID_LEN, MAVLINK_MSG_ID_SystemID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, buf, MAVLINK_MSG_ID_SystemID_LEN);
#endif
#else
	mavlink_systemid_t *packet = (mavlink_systemid_t *)msgbuf;
	packet->u_a = u_a;
	packet->u_e = u_e;
	packet->u_r = u_r;
	packet->ax = ax;
	packet->ay = ay;
	packet->az = az;
	packet->p = p;
	packet->q = q;
	packet->r = r;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, (const char *)packet, MAVLINK_MSG_ID_SystemID_LEN, MAVLINK_MSG_ID_SystemID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SystemID, (const char *)packet, MAVLINK_MSG_ID_SystemID_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SystemID UNPACKING


/**
 * @brief Get field u_a from systemid message
 *
 * @return Aileron control input in raw PWM value. Positive means right wing down. 
 */
static inline int16_t mavlink_msg_systemid_get_u_a(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field u_e from systemid message
 *
 * @return Elevator control input in raw PWM value. Positive means pitching up. 
 */
static inline int16_t mavlink_msg_systemid_get_u_e(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field u_r from systemid message
 *
 * @return Rudder control input in raw PWM value. Positive means turning to the right. 
 */
static inline int16_t mavlink_msg_systemid_get_u_r(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field ax from systemid message
 *
 * @return Acceleration on the body x-axis in 0.01g. Positive front. 
 */
static inline int16_t mavlink_msg_systemid_get_ax(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field ay from systemid message
 *
 * @return Acceleration on the body y-axis in 0.01g. Positive right. 
 */
static inline int16_t mavlink_msg_systemid_get_ay(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field az from systemid message
 *
 * @return Acceleration on the body z-axis in 0.01g. Positive down. 
 */
static inline int16_t mavlink_msg_systemid_get_az(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field p from systemid message
 *
 * @return Roll rate in 0.1 deg/s. Same sign-convention as roll angle. 
 */
static inline int16_t mavlink_msg_systemid_get_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field q from systemid message
 *
 * @return Pitch rate in 0.1 deg/s. Same sign-convention as pitch angle. 
 */
static inline int16_t mavlink_msg_systemid_get_q(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field r from systemid message
 *
 * @return Yaw rate in 0.1 deg/s. Same sign-convention as yaw angle. 
 */
static inline int16_t mavlink_msg_systemid_get_r(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a systemid message into a struct
 *
 * @param msg The message to decode
 * @param systemid C-struct to decode the message contents into
 */
static inline void mavlink_msg_systemid_decode(const mavlink_message_t* msg, mavlink_systemid_t* systemid)
{
#if MAVLINK_NEED_BYTE_SWAP
	systemid->u_a = mavlink_msg_systemid_get_u_a(msg);
	systemid->u_e = mavlink_msg_systemid_get_u_e(msg);
	systemid->u_r = mavlink_msg_systemid_get_u_r(msg);
	systemid->ax = mavlink_msg_systemid_get_ax(msg);
	systemid->ay = mavlink_msg_systemid_get_ay(msg);
	systemid->az = mavlink_msg_systemid_get_az(msg);
	systemid->p = mavlink_msg_systemid_get_p(msg);
	systemid->q = mavlink_msg_systemid_get_q(msg);
	systemid->r = mavlink_msg_systemid_get_r(msg);
#else
	memcpy(systemid, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SystemID_LEN);
#endif
}
