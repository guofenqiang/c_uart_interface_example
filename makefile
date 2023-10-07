all: git_submodule mavlink_control

mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp receiver.cpp sender.cpp protocol_conversion.cpp CRCCheck.cpp BZConv.cpp
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp receiver.cpp sender.cpp protocol_conversion.cpp CRCCheck.cpp BZConv.cpp -o mavlink_control -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o mavlink_control
