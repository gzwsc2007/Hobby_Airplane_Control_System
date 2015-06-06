/** @file
 *	@brief MAVLink comm protocol testsuite generated from hacs.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef HACS_TESTSUITE_H
#define HACS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_hacs(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_hacs(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_pfd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pfd_t packet_in = {
		17235,17339,17443,17547,17651,17755
    };
	mavlink_pfd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.altitude = packet_in.altitude;
        	packet1.airspeed = packet_in.airspeed;
        	packet1.battI = packet_in.battI;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pfd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.airspeed , packet1.battI );
	mavlink_msg_pfd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.airspeed , packet1.battI );
	mavlink_msg_pfd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pfd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.airspeed , packet1.battI );
	mavlink_msg_pfd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_navd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_navd_t packet_in = {
		963497464,963497672,17651,17755,17859,17963
    };
	mavlink_navd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.battV = packet_in.battV;
        	packet1.temp = packet_in.temp;
        	packet1.course = packet_in.course;
        	packet1.groundspeed = packet_in.groundspeed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_navd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_pack(system_id, component_id, &msg , packet1.battV , packet1.temp , packet1.latitude , packet1.longitude , packet1.course , packet1.groundspeed );
	mavlink_msg_navd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.battV , packet1.temp , packet1.latitude , packet1.longitude , packet1.course , packet1.groundspeed );
	mavlink_msg_navd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_navd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_send(MAVLINK_COMM_1 , packet1.battV , packet1.temp , packet1.latitude , packet1.longitude , packet1.course , packet1.groundspeed );
	mavlink_msg_navd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_systemid(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_systemid_t packet_in = {
		963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587
    };
	mavlink_systemid_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.u_a = packet_in.u_a;
        	packet1.u_e = packet_in.u_e;
        	packet1.u_r = packet_in.u_r;
        	packet1.ax = packet_in.ax;
        	packet1.ay = packet_in.ay;
        	packet1.az = packet_in.az;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.p = packet_in.p;
        	packet1.q = packet_in.q;
        	packet1.r = packet_in.r;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_systemid_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_systemid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_systemid_pack(system_id, component_id, &msg , packet1.timestamp , packet1.u_a , packet1.u_e , packet1.u_r , packet1.ax , packet1.ay , packet1.az , packet1.roll , packet1.pitch , packet1.yaw , packet1.p , packet1.q , packet1.r );
	mavlink_msg_systemid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_systemid_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.u_a , packet1.u_e , packet1.u_r , packet1.ax , packet1.ay , packet1.az , packet1.roll , packet1.pitch , packet1.yaw , packet1.p , packet1.q , packet1.r );
	mavlink_msg_systemid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_systemid_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_systemid_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.u_a , packet1.u_e , packet1.u_r , packet1.ax , packet1.ay , packet1.az , packet1.roll , packet1.pitch , packet1.yaw , packet1.p , packet1.q , packet1.r );
	mavlink_msg_systemid_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_magcal(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_magcal_t packet_in = {
		17235,17339,17443
    };
	mavlink_magcal_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mx = packet_in.mx;
        	packet1.my = packet_in.my;
        	packet1.mz = packet_in.mz;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_magcal_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_magcal_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_magcal_pack(system_id, component_id, &msg , packet1.mx , packet1.my , packet1.mz );
	mavlink_msg_magcal_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_magcal_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mx , packet1.my , packet1.mz );
	mavlink_msg_magcal_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_magcal_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_magcal_send(MAVLINK_COMM_1 , packet1.mx , packet1.my , packet1.mz );
	mavlink_msg_magcal_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sysstate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sysstate_t packet_in = {
		5
    };
	mavlink_sysstate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mode = packet_in.mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sysstate_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sysstate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sysstate_pack(system_id, component_id, &msg , packet1.mode );
	mavlink_msg_sysstate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sysstate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode );
	mavlink_msg_sysstate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sysstate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sysstate_send(MAVLINK_COMM_1 , packet1.mode );
	mavlink_msg_sysstate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_syscmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_syscmd_t packet_in = {
		5
    };
	mavlink_syscmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.cmd_flag = packet_in.cmd_flag;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_syscmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_syscmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_syscmd_pack(system_id, component_id, &msg , packet1.cmd_flag );
	mavlink_msg_syscmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_syscmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd_flag );
	mavlink_msg_syscmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_syscmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_syscmd_send(MAVLINK_COMM_1 , packet1.cmd_flag );
	mavlink_msg_syscmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hacs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_pfd(system_id, component_id, last_msg);
	mavlink_test_navd(system_id, component_id, last_msg);
	mavlink_test_systemid(system_id, component_id, last_msg);
	mavlink_test_magcal(system_id, component_id, last_msg);
	mavlink_test_sysstate(system_id, component_id, last_msg);
	mavlink_test_syscmd(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // HACS_TESTSUITE_H
