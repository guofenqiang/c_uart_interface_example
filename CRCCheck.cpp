#include "CRCCheck.h"



unsigned short GetCRC16(unsigned char *buf, int nLen) 
{ 	
	int i = 0; 	
	unsigned short wCRC = 0; 	 	
	if (!buf)  	
	{	 		
		return 0; 	
	} 	 	
	for(i = 0; i < nLen; i++)   	
	{
		wCRC=(wCRC<<8)^CRC16Tab[((wCRC>>8)^*(unsigned char *)buf++)&0x00FF]; 
	} 	
	return wCRC; 
}
