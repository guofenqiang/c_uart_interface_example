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
	write_start();
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

	result = pthread_create( &read_tid, NULL, serial_read, this);
	if ( result ) throw result;
}

void UAV_Interface::platform_feedback()
{
	bz_message_ground_down_t bz_message = {0};

	_ptconv->uav_platform_feedback(&bz_message, 
								   _ptconv->sender_sysid, 
								   10, 
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
		usleep(1000000);
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
