#include "uav_interface.h"


UAV_Interface::UAV_Interface(Generic_Port *port_)
{
    _port = port_;
    _ptconv = new ProtocolConversion(_port);
}

UAV_Interface::UAV_Interface(Generic_Port *port_, Generic_Port *dest_port_)
{
    _port = port_;
    _dest_port = dest_port_;
    _ptconv = new ProtocolConversion(_port, dest_port_);
}




void UAV_Interface::start()
{
	read_start();


	if (protocol_mode == 1) {
		while ( not sysid )
		{
			usleep(500000); // check at 2Hz
		}

		printf("GOT VEHICLE SYSTEM ID: %i\n", sysid );
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", compid);
	}



	write_start();

	// 定时发送摇杆信息
	Init_Timer0();
}

void UAV_Interface::stop()
{

}

int UAV_Interface::read_port()
{
#define  BUFF_SIZE 1024
    char rx_buff[BUFF_SIZE];
	uint8_t total_bytes;
	bool success;
	mavlink_message_t message = {0};

	//==========串口接收(字符串)============//

	while (1) {
		total_bytes = 0;
		if (protocol_mode == 0) {
            success = _port->read_bz_message(rx_buff, &total_bytes);
            if (success) {					
				if (compareArrays(rx_buff, rx_buff_last, total_bytes)) {
					continue;
				} 

				memcpy(rx_buff_last, rx_buff, total_bytes);
				for (int i = 0; i < total_bytes; i ++) {
					printf("%02x", rx_buff[i]);
				}
				printf("\n");
				_ptconv->bz_telecontrol_decode(rx_buff, total_bytes);
            }
		} else if (protocol_mode == 1) {
			/*接飞控后还是会丢一些包*/
			// memset(rx_buff, 0, sizeof(rx_buff));
			// while (total_bytes < rx_buff[1] + 12) {
			// 	//nbytye只有是1的时候才不会异常
			// 	// Lock
			// 	pthread_mutex_lock(&lock);
			// 	int bytes_read = read(fd, rx_buff + total_bytes, 1);
			// 	// Unlock
			// 	pthread_mutex_unlock(&lock);

			// 	if (rx_buff[0] != 0xFD) {
			// 		break;					
			// 	}
			// 	if (bytes_read <= 0) {
			// 		break;
			// 	}

			// 	total_bytes += bytes_read;
			// }

			// ptconv.bz_telemetry_decode(rx_buff, total_bytes);

			/*直接调用下面的也可以解析*/
			success = _port->read_message(message);
			if (success) {
				sysid = message.sysid;
				compid = message.compid;
				_ptconv->handle_message(message);
			}
		} else {
			printf("no supported protocol conversion\n");
			throw;
		}
		
		// this->dest_port->write_port(rx_buff, rx_len);
	}

	return 0;
}

int UAV_Interface::write_port(char *buf, unsigned len)
{
	return _dest_port->write_bz_message(buf, len);
}


void *serial_read(void *args)
{
	UAV_Interface *uav_interface = (UAV_Interface *)args;

	int result = uav_interface->read_port();
	return NULL;
}

int UAV_Interface::read_start()
{
	pthread_t read_tid;
	int result;

	if (protocol_mode == 0) {
		// Init_Timer0(); //目的时过滤重复帧：bz遥控指令针对每次数据都会连续发送3帧，帧间时间≤10ms
	}

	result = pthread_create( &read_tid, NULL, serial_read, this);
	if ( result ) throw result;
}

void UAV_Interface::platform_feedback()
{
	bz_message_ground_down_t bz_message = {0};

	_ptconv->uav_platform_feedback(&bz_message, 
								   sysid, 
								   _ptconv->receiver_sysid, 
								   _ptconv->feedback_data);
	_ptconv->ground_down_t_to_qbyte(_ptconv->send_buff, 
									&_ptconv->send_len,
									&bz_message);
	int result = write_port(_ptconv->send_buff,
							_ptconv->send_len);
}

//当前是使用线程平台状态反馈
void *serial_write(void *args)
{	
	UAV_Interface *uav_interface = (UAV_Interface *)args;

	while (1) {
		uav_interface->platform_feedback();
		usleep(200000);
	}

	return NULL;

}

int UAV_Interface::write_start()
{
	pthread_t write_tid;
	int result;
	if (this->protocol_mode == 0) {
		return 0;
	}

	result = pthread_create( &write_tid, NULL, serial_write, this);
	if ( result ) throw result;
}


bool UAV_Interface::compareArrays(char arr1[], char arr2[], uint8_t size) 
{
    for (int i = 0; i < size; i++) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}


void UAV_Interface::exec_feedback_handle()
{
	memset(rx_buff_last, 0, 1024);
	_flag = 0;
}

void UAV_Interface::timer_handler()
{
	static int count = 0;
	_flag = 0;
	
	if (protocol_mode == 1) {
		return;
	}

	timer.start(1000000); //start 应该初始化一次，过多cpu会发烫
	while (true) {
		if (timer.wait()) {
			// 执行定时任务, 25Hz, same as real joystick rate
			if (count >= 40) {
				count = 0;
				// exec_feedback_handle();
				virtualTabletJoystickValue();
			}
			count++;
		}
	}
}

void *exec_timer(void *args)
{	
	UAV_Interface *uav_interface = (UAV_Interface *)args;

	uav_interface->timer_handler();

	return NULL;
}

void UAV_Interface::Init_Timer0()
{
	pthread_t tid;

	pthread_create( &tid, NULL, exec_timer, this);

    return;
}

void UAV_Interface::virtualTabletJoystickValue()
{
	_ptconv->sendJoystickDataThreadSafe();
}
