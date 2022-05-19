#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define I2C_ADDR 0x10
char buffer[1];

int main(void) 
{
	int fd = open("/dev/i2c-1",O_RDWR);
	ioctl(fd,I2C_SLAVE,I2C_ADDR);
	buffer[0] = 0b00000010;

	for(;;) 
	{
		write(fd,buffer,1);
		read(fd,buffer,1);
		printf("0x%02X\n",buffer[0]);
		usleep(100000);
	}

	return 0;

}

