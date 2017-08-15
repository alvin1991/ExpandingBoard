// MESSAGE EXT_BOARD PACKING

#define MAVLINK_MSG_ID_EXT_BOARD 227

typedef struct MAVLINK_PACKED __mavlink_ext_board_t
{
 int32_t mag_f; /*< mag finder front*/
 int32_t mag_b; /*< mag finder back*/
 int32_t rfid; /*< RFID*/
 float ultrasonic[12]; /*< ultrasonic*/
 int32_t Encoder[4]; /*< ultrasonic*/
} mavlink_ext_board_t;

#define MAVLINK_MSG_ID_EXT_BOARD_LEN 76
#define MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN 76
#define MAVLINK_MSG_ID_227_LEN 76
#define MAVLINK_MSG_ID_227_MIN_LEN 76

#define MAVLINK_MSG_ID_EXT_BOARD_CRC 251
#define MAVLINK_MSG_ID_227_CRC 251

#define MAVLINK_MSG_EXT_BOARD_FIELD_ULTRASONIC_LEN 12
#define MAVLINK_MSG_EXT_BOARD_FIELD_ENCODER_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EXT_BOARD { \
	227, \
	"EXT_BOARD", \
	5, \
	{  { "mag_f", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_ext_board_t, mag_f) }, \
         { "mag_b", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_ext_board_t, mag_b) }, \
         { "rfid", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ext_board_t, rfid) }, \
         { "ultrasonic", NULL, MAVLINK_TYPE_FLOAT, 12, 12, offsetof(mavlink_ext_board_t, ultrasonic) }, \
         { "Encoder", NULL, MAVLINK_TYPE_INT32_T, 4, 60, offsetof(mavlink_ext_board_t, Encoder) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EXT_BOARD { \
	"EXT_BOARD", \
	5, \
	{  { "mag_f", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_ext_board_t, mag_f) }, \
         { "mag_b", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_ext_board_t, mag_b) }, \
         { "rfid", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ext_board_t, rfid) }, \
         { "ultrasonic", NULL, MAVLINK_TYPE_FLOAT, 12, 12, offsetof(mavlink_ext_board_t, ultrasonic) }, \
         { "Encoder", NULL, MAVLINK_TYPE_INT32_T, 4, 60, offsetof(mavlink_ext_board_t, Encoder) }, \
         } \
}
#endif

/**
 * @brief Pack a ext_board message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mag_f mag finder front
 * @param mag_b mag finder back
 * @param rfid RFID
 * @param ultrasonic ultrasonic
 * @param Encoder ultrasonic
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ext_board_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t mag_f, int32_t mag_b, int32_t rfid, const float *ultrasonic, const int32_t *Encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXT_BOARD_LEN];
	_mav_put_int32_t(buf, 0, mag_f);
	_mav_put_int32_t(buf, 4, mag_b);
	_mav_put_int32_t(buf, 8, rfid);
	_mav_put_float_array(buf, 12, ultrasonic, 12);
	_mav_put_int32_t_array(buf, 60, Encoder, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXT_BOARD_LEN);
#else
	mavlink_ext_board_t packet;
	packet.mag_f = mag_f;
	packet.mag_b = mag_b;
	packet.rfid = rfid;
	mav_array_memcpy(packet.ultrasonic, ultrasonic, sizeof(float)*12);
	mav_array_memcpy(packet.Encoder, Encoder, sizeof(int32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EXT_BOARD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EXT_BOARD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
}

/**
 * @brief Pack a ext_board message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mag_f mag finder front
 * @param mag_b mag finder back
 * @param rfid RFID
 * @param ultrasonic ultrasonic
 * @param Encoder ultrasonic
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ext_board_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t mag_f,int32_t mag_b,int32_t rfid,const float *ultrasonic,const int32_t *Encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXT_BOARD_LEN];
	_mav_put_int32_t(buf, 0, mag_f);
	_mav_put_int32_t(buf, 4, mag_b);
	_mav_put_int32_t(buf, 8, rfid);
	_mav_put_float_array(buf, 12, ultrasonic, 12);
	_mav_put_int32_t_array(buf, 60, Encoder, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXT_BOARD_LEN);
#else
	mavlink_ext_board_t packet;
	packet.mag_f = mag_f;
	packet.mag_b = mag_b;
	packet.rfid = rfid;
	mav_array_memcpy(packet.ultrasonic, ultrasonic, sizeof(float)*12);
	mav_array_memcpy(packet.Encoder, Encoder, sizeof(int32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EXT_BOARD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EXT_BOARD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
}

/**
 * @brief Encode a ext_board struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ext_board C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ext_board_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ext_board_t* ext_board)
{
	return mavlink_msg_ext_board_pack(system_id, component_id, msg, ext_board->mag_f, ext_board->mag_b, ext_board->rfid, ext_board->ultrasonic, ext_board->Encoder);
}

/**
 * @brief Encode a ext_board struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ext_board C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ext_board_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ext_board_t* ext_board)
{
	return mavlink_msg_ext_board_pack_chan(system_id, component_id, chan, msg, ext_board->mag_f, ext_board->mag_b, ext_board->rfid, ext_board->ultrasonic, ext_board->Encoder);
}

/**
 * @brief Send a ext_board message
 * @param chan MAVLink channel to send the message
 *
 * @param mag_f mag finder front
 * @param mag_b mag finder back
 * @param rfid RFID
 * @param ultrasonic ultrasonic
 * @param Encoder ultrasonic
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ext_board_send(mavlink_channel_t chan, int32_t mag_f, int32_t mag_b, int32_t rfid, const float *ultrasonic, const int32_t *Encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXT_BOARD_LEN];
	_mav_put_int32_t(buf, 0, mag_f);
	_mav_put_int32_t(buf, 4, mag_b);
	_mav_put_int32_t(buf, 8, rfid);
	_mav_put_float_array(buf, 12, ultrasonic, 12);
	_mav_put_int32_t_array(buf, 60, Encoder, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXT_BOARD, buf, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
#else
	mavlink_ext_board_t packet;
	packet.mag_f = mag_f;
	packet.mag_b = mag_b;
	packet.rfid = rfid;
	mav_array_memcpy(packet.ultrasonic, ultrasonic, sizeof(float)*12);
	mav_array_memcpy(packet.Encoder, Encoder, sizeof(int32_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXT_BOARD, (const char *)&packet, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
#endif
}

/**
 * @brief Send a ext_board message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ext_board_send_struct(mavlink_channel_t chan, const mavlink_ext_board_t* ext_board)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ext_board_send(chan, ext_board->mag_f, ext_board->mag_b, ext_board->rfid, ext_board->ultrasonic, ext_board->Encoder);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXT_BOARD, (const char *)ext_board, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
#endif
}

#if MAVLINK_MSG_ID_EXT_BOARD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ext_board_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t mag_f, int32_t mag_b, int32_t rfid, const float *ultrasonic, const int32_t *Encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, mag_f);
	_mav_put_int32_t(buf, 4, mag_b);
	_mav_put_int32_t(buf, 8, rfid);
	_mav_put_float_array(buf, 12, ultrasonic, 12);
	_mav_put_int32_t_array(buf, 60, Encoder, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXT_BOARD, buf, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
#else
	mavlink_ext_board_t *packet = (mavlink_ext_board_t *)msgbuf;
	packet->mag_f = mag_f;
	packet->mag_b = mag_b;
	packet->rfid = rfid;
	mav_array_memcpy(packet->ultrasonic, ultrasonic, sizeof(float)*12);
	mav_array_memcpy(packet->Encoder, Encoder, sizeof(int32_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXT_BOARD, (const char *)packet, MAVLINK_MSG_ID_EXT_BOARD_MIN_LEN, MAVLINK_MSG_ID_EXT_BOARD_LEN, MAVLINK_MSG_ID_EXT_BOARD_CRC);
#endif
}
#endif

#endif

// MESSAGE EXT_BOARD UNPACKING


/**
 * @brief Get field mag_f from ext_board message
 *
 * @return mag finder front
 */
static inline int32_t mavlink_msg_ext_board_get_mag_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field mag_b from ext_board message
 *
 * @return mag finder back
 */
static inline int32_t mavlink_msg_ext_board_get_mag_b(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field rfid from ext_board message
 *
 * @return RFID
 */
static inline int32_t mavlink_msg_ext_board_get_rfid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field ultrasonic from ext_board message
 *
 * @return ultrasonic
 */
static inline uint16_t mavlink_msg_ext_board_get_ultrasonic(const mavlink_message_t* msg, float *ultrasonic)
{
	return _MAV_RETURN_float_array(msg, ultrasonic, 12,  12);
}

/**
 * @brief Get field Encoder from ext_board message
 *
 * @return ultrasonic
 */
static inline uint16_t mavlink_msg_ext_board_get_Encoder(const mavlink_message_t* msg, int32_t *Encoder)
{
	return _MAV_RETURN_int32_t_array(msg, Encoder, 4,  60);
}

/**
 * @brief Decode a ext_board message into a struct
 *
 * @param msg The message to decode
 * @param ext_board C-struct to decode the message contents into
 */
static inline void mavlink_msg_ext_board_decode(const mavlink_message_t* msg, mavlink_ext_board_t* ext_board)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	ext_board->mag_f = mavlink_msg_ext_board_get_mag_f(msg);
	ext_board->mag_b = mavlink_msg_ext_board_get_mag_b(msg);
	ext_board->rfid = mavlink_msg_ext_board_get_rfid(msg);
	mavlink_msg_ext_board_get_ultrasonic(msg, ext_board->ultrasonic);
	mavlink_msg_ext_board_get_Encoder(msg, ext_board->Encoder);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EXT_BOARD_LEN? msg->len : MAVLINK_MSG_ID_EXT_BOARD_LEN;
        memset(ext_board, 0, MAVLINK_MSG_ID_EXT_BOARD_LEN);
	memcpy(ext_board, _MAV_PAYLOAD(msg), len);
#endif
}
