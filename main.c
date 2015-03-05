#define MAVLINK_MAX_PAYLOAD_LEN 96

#include <stdio.h>
#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_exti.h>
#include <misc.h>

#include <common/mavlink.h>
#include <common/common.h>
#include <ardupilotmega/version.h>
#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

void InitUsart1(void);
void InitUsart6(void);
void Delay(__IO uint32_t nCount);
void decodeMAVlinkMsg(mavlink_message_t *msg);
void USART6_IRQHandler(void);
void send_command_arm_disarm(void);
void send_command_takeoff(void);
void send_command_land(void);

//#define MAX_STRLEN 12
//#define MAX_STRLENSEND 12
//volatile char send_string[MAX_STRLENSEND+1];
//volatile char received_string[MAX_STRLEN+1];

//global variables
uint8_t AIRSUPPORT_MAV_MODE = MAV_MODE_PREFLIGHT;
uint8_t AIRSUPPORT_MAV_STATE = MAV_STATE_STANDBY;
int heartbeat_check = 0, beatcount = 0, status = 1, flying = 1;

//init variables
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

void InitUsart1(void) {
	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStructure; // this is for the USART1 initilization

	//Enable clock of pints & UART
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    //setup GPIO
    //GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    //Do pin-assignment
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    // Setup UART
	USART_InitStructure.USART_BaudRate = 57600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
    // Enable Interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // Init interrupt
	NVIC_InitTypeDef NVIC_struct;
	NVIC_struct.NVIC_IRQChannel                      = USART1_IRQn;
	NVIC_struct.NVIC_IRQChannelCmd                   = ENABLE;
	NVIC_struct.NVIC_IRQChannelPreemptionPriority    = 0;
	NVIC_struct.NVIC_IRQChannelSubPriority           = 0;
	NVIC_Init(&NVIC_struct);

    //Enable UART port
    USART_Cmd(USART1, ENABLE);
}

void InitUsart6(void) {
    //Enable clock of pints & UART
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    //setup GPIO
    GPIO_InitTypeDef GPIO_struct;
    GPIO_struct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_struct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_struct.GPIO_OType = GPIO_OType_PP;
    GPIO_struct.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC, &GPIO_struct);

    //Do pin-assignment
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    // Setup UART
	USART_InitTypeDef USART_struct;
	USART_struct.USART_BaudRate              = 57600;
	USART_struct.USART_WordLength            = USART_WordLength_8b;
	USART_struct.USART_StopBits              = USART_StopBits_1;
	USART_struct.USART_Parity                = USART_Parity_No;
	USART_struct.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
	USART_struct.USART_Mode                  = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART6, &USART_struct);

    // Enable Interrupt
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    // Init interrupt
	NVIC_InitTypeDef NVIC_struct;
	NVIC_struct.NVIC_IRQChannel                      = USART6_IRQn;
	NVIC_struct.NVIC_IRQChannelCmd                   = ENABLE;
	NVIC_struct.NVIC_IRQChannelPreemptionPriority    = 0;
	NVIC_struct.NVIC_IRQChannelSubPriority           = 0;
	NVIC_Init(&NVIC_struct);

    //Enable UART port
    USART_Cmd(USART6, ENABLE);
}

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void send_command_takeoff (void){
	//set struct for message
	mavlink_command_long_t command;
	command.command = MAV_CMD_NAV_TAKEOFF;
	command.param1 = 0; //0 to disarm
	command.param2 = 0;
	command.param3 = 3.0;
	command.param4 = 0;
	command.param5 = 0;
	command.param6 = 0;
	command.param7 = 0;
	command.confirmation = 0;
	command.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
	command.target_system = 1;

	//create message
	mavlink_msg_command_long_encode(MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
	&msg, &command);

	//compress & copy to buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	//send to APM
	uint16_t i;
	for (i = 0; i < len; i++) {
	    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
	        USART_SendData(USART6, buf[i]);
	    }
}

void send_command_land (void){
	//set struct for message
	mavlink_command_long_t command;
	command.command = MAV_CMD_NAV_LAND;
	command.param1 = 0; //0 to disarm
	command.param2 = 0;
	command.param3 = 0;
	command.param4 = 3.0;
	command.param5 = 0;
	command.param6 = 0;
	command.param7 = 0;
	command.confirmation = 0;
	command.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
	command.target_system = 1;

	//create message
	mavlink_msg_command_long_encode(MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
	&msg, &command);

	//compress & copy to buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	//send to APM
	uint16_t i;
	for (i = 0; i < len; i++) {
	    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
	        USART_SendData(USART6, buf[i]);
	    }
}

void decodeMAVlinkMsg(mavlink_message_t *msg) {
    //message type definitions (not allowed in switch!)
    mavlink_heartbeat_t heart;
    mavlink_sys_status_t state;
    mavlink_gps_raw_int_t rawpos;
    mavlink_raw_imu_t cords;
    mavlink_scaled_pressure_t pressure;
    mavlink_attitude_t attitude;
    mavlink_global_position_int_t globalpos;
    mavlink_rc_channels_scaled_t channelsr;
    mavlink_rc_channels_raw_t channel;
    mavlink_nav_controller_output_t output;
    mavlink_mission_current_t missioncurrent;
    mavlink_vfr_hud_t vfr;
    mavlink_command_ack_t command;
    mavlink_statustext_t statustext;
    beatcount++;
    if (beatcount > 5) {
    	heartbeat_check = 1;
	}
    //switch on message id
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:    //msg 0
        	//decode heartbeat
        	mavlink_msg_heartbeat_decode(msg, &heart);

        	//show result in printf-UART function
        	printf("msgid:[%d] heartbeat! [pilot:%d type:%d] mode:%d mavlink:%d status:%d \n\r",
        			msg->msgid, heart.autopilot, heart.type,
        			heart.base_mode, heart.mavlink_version,
        			heart.system_status);
        	send_command_arm_disarm();
        	/*if(flying == 0) {
        		flying = 1;
        		send_command_land();
        	    printf("Landing!\n\r");
        	}
        	else if(flying == 1){
        		flying = 0;
        		send_command_takeoff();
        		printf("Takeoff!\n\r");
        	}*/
        	break;
        case MAVLINK_MSG_ID_SYS_STATUS:    //msg 1
            mavlink_msg_sys_status_decode(msg, &state);
            printf("msgid:[%d] mavlink_msg_sys_status: spresent:%d senabled:%d shealth:%d load:%d vbat:%d cbat:%d batr:%d drop:%d err:%d err1:%d err2:%d err3:%d err4:%d \n\r",
            	msg->msgid,
                state.onboard_control_sensors_present,
                state.onboard_control_sensors_enabled,
                state.onboard_control_sensors_health,
                state.load,
                state.voltage_battery,
                state.current_battery,
                state.battery_remaining,
                state.drop_rate_comm,
                state.errors_comm,
                state.errors_count1,
                state.errors_count2,
                state.errors_count3,
                state.errors_count4);
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:    //msg 24
        	mavlink_msg_gps_raw_int_decode(msg, &rawpos);
            printf("msgid:[%d] mavlink_msg_gps_raw_int: lat:%d lon:%d alt:%d fix:%d eph:%d epv:%d vel:%d cog:%d vis:%d \n\r",
            	msg->msgid,
            	rawpos.fix_type,
            	rawpos.lat,
            	rawpos.lon,
            	rawpos.alt,
            	rawpos.eph,
            	rawpos.epv,
            	rawpos.vel,
            	rawpos.cog,
            	rawpos.satellites_visible);
            break;
        case MAVLINK_MSG_ID_RAW_IMU:    //msg 27
        	mavlink_msg_raw_imu_decode(msg, &cords);
            printf("msgid:[%d] mavlink_msg_id_raw_imu: xacc:%d yacc:%d zacc:%d xgyro:%d ygyro:%d zgyro:%d xmag:%d ymag:%d zmag:%d \n\r",
            	msg->msgid,
            	cords.xacc,
            	cords.yacc,
            	cords.zacc,
            	cords.xgyro,
            	cords.ygyro,
            	cords.zgyro,
            	cords.xmag,
            	cords.ymag,
            	cords.zmag);
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:    //msg 29
        	mavlink_msg_scaled_pressure_decode(msg, &pressure);
            printf("msgid:[%d] mavlink_msg_scaled_pressure: press_abs:%f press_diff:%f temperature:%d \n\r",
            	msg->msgid,
            	pressure.press_abs,
            	pressure.press_diff,
            	pressure.temperature);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:    //msg 30
            mavlink_msg_attitude_decode(msg, &attitude);
            printf("msgid:[%d] mavlink_msg_attitude: roll:%f pitch:%f yaw:%f rollspeed:%f pitchspeed:%f yawspeed:%f \n\r",
            	msg->msgid,
                attitude.roll,
                attitude.pitch,
                attitude.yaw,
                attitude.rollspeed,
                attitude.pitchspeed,
                attitude.yawspeed);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    //msg 33
            mavlink_msg_global_position_int_decode(msg, &globalpos);
            printf("msgid:[%d] mavlink_msg_global_position_int: lat:%d lon:%d alt:%d relalt:%d vx:%d vy:%d vz:%d hdg:%d \n\r",
            	msg->msgid,
                globalpos.lat,
                globalpos.lon,
                globalpos.alt,
                globalpos.relative_alt,
                globalpos.vx,
                globalpos.vy,
                globalpos.vz,
                globalpos.hdg);
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:    //msg 34
        	mavlink_msg_rc_channels_scaled_decode(msg, &channelsr);
            printf("msgid:[%d] mavlink_msg_channels_scale: chan1_scaled:%d chan2_scaled:%d chan3_scaled:%d chan4_scaled:%d chan5_scaled:%d chan6_scaled:%d chan7_scaled:%d chan8_scaled:%d port:%d rssi:%d \n\r",
            	msg->msgid,
            	channelsr.chan1_scaled,
            	channelsr.chan2_scaled,
            	channelsr.chan3_scaled,
            	channelsr.chan4_scaled,
            	channelsr.chan5_scaled,
            	channelsr.chan6_scaled,
            	channelsr.chan7_scaled,
            	channelsr.chan8_scaled,
            	channelsr.port,
            	channelsr.rssi);
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:    //msg 35
            mavlink_msg_rc_channels_raw_decode(msg, &channel);
            printf("msgid:[%d] mavlink_msg_channel: chan1_raw:%d chan2_raw:%d chan3_raw:%d chan4_raw:%d chan5_raw:%d chan6_raw:%d chan7_raw:%d chan8_raw:%d port:%d rssi:%d \n\r",
                msg->msgid,
                channel.chan1_raw,
                channel.chan2_raw,
                channel.chan3_raw,
                channel.chan4_raw,
                channel.chan5_raw,
                channel.chan6_raw,
                channel.chan7_raw,
                channel.chan8_raw,
                channel.port,
                channel.rssi);
            break;
        case MAVLINK_MSG_ID_MISSION_CURRENT:    //msg 42
            mavlink_msg_mission_current_decode(msg, &missioncurrent);
            printf("msgid:[%d] mavlink_msg_mission_current: seq:%d \n\r",
            	msg->msgid,
                missioncurrent.seq);
            break;
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT :    //msg 62
            mavlink_msg_nav_controller_output_decode(msg, &output);
            printf("msgid:[%d] mavlink_msg_id_nav_controller_output: nav_roll:%f nav_pitch:%f alt_error:%f aspd_error:%f xtrack_error:%f nav_bearing:%d target_bearing:%d wp_dist:%d \n\r",
                msg->msgid,
                output.nav_roll,
                output.nav_pitch,
                output.alt_error,
                output.aspd_error,
                output.xtrack_error,
                output.nav_bearing,
                output.target_bearing,
            	output.wp_dist);
            break;
        case MAVLINK_MSG_ID_VFR_HUD:    //msg 74
            mavlink_msg_vfr_hud_decode(msg, &vfr);
            printf("msgid:[%d] mavlink_msg_id_vfr_hud: airspeed:%f ground_speed:%f alt:%f climb:%f heading:%d throttle:%d \n\r",
                msg->msgid,
                vfr.airspeed,
                vfr.groundspeed,
                vfr.alt,
                vfr.climb,
                vfr.heading,
                vfr.throttle);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK:    //msg 77
        	mavlink_msg_command_ack_decode(msg, &command);
            printf("msgid:[%d] mavlink_msg_command_ack: command:%d result:%d \n\r",
                msg->msgid,
                command.command,
                command.result);
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:    //msg 253
            mavlink_msg_statustext_decode(msg, &statustext);
            printf("msgid:[%d] mavlink_statustext_t: severity:%d text: \n\r",
                msg->msgid,
                statustext.severity);
            	int j;
            	char str[] = {statustext.text[50]};
                for (j = 0; j < 50; j++)
                {
                    printf("%c\n\r", str[j]);
                }
            break;
        default:
         	printf("msgid:[%d] not decoded.. \n\r", msg->msgid);
            break;
        }
}

void USART6_IRQHandler(void){
	if( USART_GetITStatus(USART6, USART_IT_RXNE) ){

		//static uint8_t cnt = 0;
		uint16_t data = USART6->DR;

		mavlink_message_t msg;
		mavlink_status_t status;
		if (mavlink_parse_char(MAVLINK_COMM_0, data, &msg, &status)) {
			decodeMAVlinkMsg(&msg);
		}
		/*
		if( (data != '\n') && (cnt < MAX_STRLENSEND) ){
			send_string[cnt] = data;
			cnt++;
		}
		else{
			cnt = 0;
			USART_puts(USART6, send_string);
		}*/
	}
}

void send_command_arm_disarm(void){
	//set struct for message
    mavlink_command_long_t command;
    command.command = MAV_CMD_COMPONENT_ARM_DISARM;
    if(status == 0) {
    	command.param1 = 1; //arm
    	status = 1;
    }
    else if(status == 1){
		command.param1 = 0; //disarm
		status = 0;
	}
    command.confirmation = 0;
    command.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
    command.target_system = 1;

    //create message
    mavlink_msg_command_long_encode(MAV_TYPE_QUADROTOR,
    		MAV_AUTOPILOT_ARDUPILOTMEGA,
    		&msg, &command);

    //compress & copy to buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    //send to APM
    uint16_t i;
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
        USART_SendData(USART6, buf[i]);
	}
    if(i == len){
    	if(status == 1) {
    		printf("armed! \n\r");
    	}
    	else if(status == 0){
    		printf("disarmed! \n\r");
    	}
    }
}

int main(void)
{
	InitUsart6();
	InitUsart1();

	printf("Hello World!\r\n");
	printf("***********************************\r\n");
	printf("*                                 *\r\n");
	printf("* Author: Tim Karman & Tom Flipse *\r\n");
	printf("*                                 *\r\n");
	printf("***********************************\r\n");

    while(1)
    {
    	if(heartbeat_check == 1)
    	{
    		//create message
    		mavlink_msg_heartbeat_pack(MAV_TYPE_GCS,
    				MAV_AUTOPILOT_INVALID, &msg, MAV_TYPE_GCS,
    				MAV_AUTOPILOT_INVALID, AIRSUPPORT_MAV_MODE,
    				0, AIRSUPPORT_MAV_STATE);

    		//compress & copy to buffer
    		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    		//send to APM
    		uint16_t i;
    		for (i = 0; i < len; i++) {
    			while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
    			USART_SendData(USART6, buf[i]);
    		}
    		/*if(status != 1) {
    			send_command_arm_disarm();
    		}*/
    	}
    }
}
/*
void USART1_IRQHandler(void){
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		USART_puts(USART1, "status33333");
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t

		 //check if the received character is not the LF character (used to determine end of string)
	     //or the if the maximum string length has been been reached

		if( (t != '\n') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			USART_puts(USART1, received_string);
		}
	}
}
*/
