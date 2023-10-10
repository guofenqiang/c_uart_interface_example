#ifndef UAV_INTERFACE_H_
#define UAV_INTERFACE_H_


#include "generic_port.h"
#include "protocol_conversion.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>

#include <common/mavlink.h>

class UAV_Interface
{
public:
    UAV_Interface(Generic_Port *port_);
    UAV_Interface(Generic_Port *port_, Generic_Port *dest_port_);

	void start();
	void stop();

    // Serial_Port *dest_port;
	int protocol_mode; // 0：telecontrol 1: telemetry
	int read_port();
	int write_port (char *buf, unsigned len);
	int read_start();
	int write_start();

    ProtocolConversion *_ptconv;
	void platform_feedback();

private:
    Generic_Port *_port;
    Generic_Port *_dest_port;

};

void *serial_read(void *args);
void *serial_write(void *args);

#endif // UAV_INTERFACE_H_