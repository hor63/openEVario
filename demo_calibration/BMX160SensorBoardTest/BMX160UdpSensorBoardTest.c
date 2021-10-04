
#include <stdio.h>
#include <memory.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>


#include "BMX160net.h"

#define BMM150_OVERFLOW_OUTPUT_FLOAT		0.0f

static struct bmm150_trim_registers trimData;
static bool trimDataReceived = false;

/** CRC-16 CCIT taken from the PPP implementation of lwip
 *
 * \see http://git.savannah.nongnu.org/cgit/lwip.git/tree/src/netif/ppp/pppos.c line 97ff
 */
static const uint16_t fcstab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
#define PPP_FCS(fcs, c) (((fcs) >> 8) ^ fcstab[((fcs) ^ (c)) & 0xff])
static inline uint16_t crcBlock(uint16_t crc, const void* const block,uint16_t len) {
	uint16_t rc = crc;
	uint16_t i;

	for (i=0; i < len; i++){
		rc = PPP_FCS(rc,((const char* const)block)[i]);
	}

	return rc;
}
/*
 * Values for FCS calculations.
 */
#define PPP_INITFCS     0xffffU  /* Initial FCS value */
#define PPP_GOODFCS     0xf0b8U  /* Good final FCS value */
// End of lwip section. Thank you.


/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer x axis data(micro-tesla) in float.
 */
static float compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_x0;
	float process_comp_x1;
	float process_comp_x2;
	float process_comp_x3;
	float process_comp_x4;

	/* Overflow condition check */
	if ((mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) &&
		(data_rhall != 0) && (trimData.dig_xyz1 != 0)) {
			/*Processing compensation equations*/
			process_comp_x0 = (((float)trimData.dig_xyz1) * 16384.0f / data_rhall);
			retval = (process_comp_x0 - 16384.0f);
			process_comp_x1 = ((float)trimData.dig_xy2) * (retval * retval / 268435456.0f);
			process_comp_x2 = process_comp_x1 + retval * ((float)trimData.dig_xy1) / 16384.0f;
			process_comp_x3 = ((float)trimData.dig_x2) + 160.0f;
			process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
			retval = ((process_comp_x4 / 8192.0f) + (((float)trimData.dig_x1) * 8.0f)) / 16.0f;
	} else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer y axis data(micro-tesla) in float.
 */
static float compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_y0;
	float process_comp_y1;
	float process_comp_y2;
	float process_comp_y3;
	float process_comp_y4;

	/* Overflow condition check */
	if ((mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
		&& (data_rhall != 0) && (trimData.dig_xyz1 != 0)) {
			/*Processing compensation equations*/
			process_comp_y0 = ((float)trimData.dig_xyz1) * 16384.0f / data_rhall;
			retval = process_comp_y0 - 16384.0f;
			process_comp_y1 = ((float)trimData.dig_xy2) * (retval * retval / 268435456.0f);
			process_comp_y2 = process_comp_y1 + retval * ((float)trimData.dig_xy1) / 16384.0f;
			process_comp_y3 = ((float)trimData.dig_y2) + 160.0f;
			process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
			retval = ((process_comp_y4 / 8192.0f) + (((float)trimData.dig_y1) * 8.0f)) / 16.0f;
	} else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer z axis data(micro-tesla) in float.
 */
static float compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_z0;
	float process_comp_z1;
	float process_comp_z2;
	float process_comp_z3;
	float process_comp_z4;
	float process_comp_z5;

	 /* Overflow condition check */
	if ((mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) &&
		(trimData.dig_z2 != 0) && (trimData.dig_z1 != 0)
		&& (trimData.dig_xyz1 != 0) && (data_rhall != 0)) {
			/* Processing compensation equations */
			process_comp_z0 = ((float)mag_data_z) - ((float)trimData.dig_z4);
			process_comp_z1 = ((float)data_rhall) - ((float)trimData.dig_xyz1);
			process_comp_z2 = (((float)trimData.dig_z3) * process_comp_z1);
			process_comp_z3 = ((float)trimData.dig_z1) * ((float)data_rhall) / 32768.0f;
			process_comp_z4 = ((float)trimData.dig_z2) + process_comp_z3;
			process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
			retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
	} else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	return retval;
}

// Nested macros. Simple stringizing macro would not expand PORT_I2C_BRIDGE
#define MAKE_PORT_STR(a) MAKE_PORT_STR_a(a)
#define MAKE_PORT_STR_a(a) #a

#define MAKE_IP_ADDR(a,b,c,d) #a "." #b "." #c "." #d

static struct addrinfo *sendToAddr = NULL;


static int openSocket() {

	int sock = -1;
	int rc;
	struct addrinfo addrHint;
	struct addrinfo *addr = NULL;
	struct protoent *prot = NULL;

	memset (&addrHint,0,sizeof(addrHint));

	prot = getprotobyname("udp");
	if (prot != NULL) {
		addrHint.ai_protocol = prot->p_proto;
		endprotoent();
	} else {
		// Fallback. Use documented value for TCP.
		addrHint.ai_protocol = 17;
	}

	addrHint.ai_family = AF_INET;
	addrHint.ai_socktype = SOCK_DGRAM;

	printf ("Addr = \"%s\", port = \"%s\"\n",MAKE_IP_ADDR_I2C_BRIDGE,"19463");

	rc = getaddrinfo(MAKE_IP_ADDR_I2C_BRIDGE,"19463",&addrHint,&sendToAddr);
	if (rc != 0){
		fprintf (stderr,"getaddrinfo error: %s\n",gai_strerror(rc));
	}


	rc = getaddrinfo("0.0.0.0","19463",&addrHint,&addr);

	if (rc == 0) {
		struct addrinfo *ad = addr;

		while (ad) {

			sock = socket(ad->ai_family,ad->ai_socktype,ad->ai_protocol);

			if (sock == -1) {
				rc = errno;
				fprintf(stderr,"Socket error: %s\n",strerror(rc));
			}

			if (sock != -1) {
				if(bind(sock,ad->ai_addr,ad->ai_addrlen) == -1) {
					rc = errno;
					fprintf(stderr,"bind error: %s\n",strerror(rc));
				}
				break;
			}
			ad = ad->ai_next;
		}
	} else {
		fprintf (stderr,"getaddrinfo error: %s\n",gai_strerror(rc));
	}
	if (addr != NULL) {
		freeaddrinfo(addr);
	}

	return sock;
}

int main (int argc, char** argv) {

	FILE* f = NULL;
	int n;
	const double accFactor = (double)0x8000 / 4.0;
	const double gyrFactor = (double)0x8000 / 250.0;

	struct BMX160Data data;

	int sock = -1;
	uint32_t timestamp = 0;
	uint32_t lastTimstamp = 0;

	sock = openSocket();
//	if (sock != -1) {
//		close (sock);
//	}

	memset (&trimData,0,sizeof(trimData));


/*
	if (argc < 2) {
		fprintf(stderr,"Usage: %s <File containing BMX160 sensorboard data>\n",argv[0]);
		return 1;
	}

	f = fopen(argv[1],"rb");

	if (!f) {
		fprintf(stderr,"Cannot open file %s \n",argv[1]);
		return 2;
	}
	n = fread(&data.header,sizeof(data.header),1,f);

	while (n == 1) {
*/

	n = recv(sock,&data,sizeof(data),0);
	printf("recv returned %i, buffer size = %i\n",n,(int)sizeof(data));
	while (n > 0) {
        uint16_t crc2 = PPP_INITFCS;

        lastTimstamp = timestamp;

		timestamp =
				(data.header.sensorTime2 << 16) |
				(data.header.sensorTime1 << 8) |
				data.header.sensorTime0;

        

		switch (data.header.unionCode) {
		case BMX160DATA_TRIM:
			if (data.header.length == (sizeof(data.header)+sizeof(data.trimData))) {

					/*
                    data.header.crc = 0U;
                    crc2 = crcBlock(PPP_INITFCS,&data,data.header.length);
                    printf("%04X\t",crc2);
					*/

					trimData = data.trimData;
					trimDataReceived = true;

					/*
					printf ("TrimData = \n");
					printf ("\tdig_x1   = %d\n",trimData.dig_x1);
					printf ("\tdig_y1   = %d\n",trimData.dig_y1);
					printf ("\tdig_z1   = %d\n",trimData.dig_z1);
					printf ("\tdig_x2   = %d\n",trimData.dig_x2);
					printf ("\tdig_y2   = %d\n",trimData.dig_y2);
					printf ("\tdig_z2   = %d\n",trimData.dig_z2);
					printf ("\tdig_z3   = %d\n",trimData.dig_z3);
					printf ("\tdig_z4   = %d\n",trimData.dig_z4);
					printf ("\tdig_xy1  = %d\n",trimData.dig_xy1);
					printf ("\tdig_xy2  = %d\n",trimData.dig_xy2);
					printf ("\tdig_xyz1 = %d\n",trimData.dig_xyz1);
					*/
			} else {
				fprintf(stderr,
						"Alignment error. data.header.length = %d but sizeof(data.header)+sizeof(data.trimData) = %lu\n",
						data.header.length,
						sizeof(data.header)+sizeof(data.trimData));
				return 3;
			}
			break;
		case BMX160DATA_ACC_GYR_MAG:
			if (data.header.length == (sizeof(data.header)+sizeof(data.accGyrMagData))) {

				// print the time in uSeconds. One tick is 39uSec.
				printf("%6d\t%1d.%1d\t",(timestamp-lastTimstamp)*39,data.header.versionMajor,data.header.versionMinor);
				printf("%04X ",data.header.crc);

				data.header.crc = 0U;
				crc2 = crcBlock(PPP_INITFCS,&data,data.header.length);
				printf("%04X\t",crc2);

				printf (
						"% 10.6f % 10.6f % 10.6f",
						compensate_x(data.accGyrMagData.magX,data.accGyrMagData.magRHall),
						compensate_y(data.accGyrMagData.magY,data.accGyrMagData.magRHall),
						compensate_z(data.accGyrMagData.magZ,data.accGyrMagData.magRHall)
						);
				printf (
						"% 10.6f % 10.6f % 10.6f",
						((double)data.accGyrMagData.gyrX/ gyrFactor),
						((double)data.accGyrMagData.gyrY/ gyrFactor),
						((double)data.accGyrMagData.gyrZ/ gyrFactor)
						);
				printf (
						"% 10.6f % 10.6f % 10.6f\n",
						(double)data.accGyrMagData.accX/ accFactor,
						(double)data.accGyrMagData.accY/ accFactor,
						(double)data.accGyrMagData.accZ/ accFactor
						);
			}

			if (!trimDataReceived) {
				static struct BMX160RecvData reqTrimData = {
							.header.unionCode = BMX160RECV_DATA_RESEND_MAG_TRIM_DATA,
							.header.filler = 0,
							.header.length = sizeof(struct BMX160RecvData),
							.header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR,
							.header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR,
							.header.crc = 0xffff,
							.dummy = 0
				};
				n = sendto(sock,&reqTrimData,sizeof(reqTrimData),0,sendToAddr->ai_addr,sendToAddr->ai_addrlen);
			}

			break;
		default:
			fprintf(stderr,"Unknown union code %d\n",(int)data.header.unionCode);
			return 4;
			break;
		}
		n = read(sock,&data,sizeof(data));
	}

	printf("Read %d bytes from %s",n,argv[1]);

	return 0;
}
