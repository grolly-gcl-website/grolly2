/*
=== Cadi Data Exchange Daemon ===
This is Protobuzzz data exchange protocol implementation for GCC C.

General protocol description could be found here:
http://gcl.engineering/kb/index.php?title=Protobuzz_protocol



*/



#define TEST_MODE

#define  _DEFAULT_SOURCE
#define	_POSIX_C_SOURCE	200809L

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>     
#include <sys/shm.h>	//Used for shared memory
#include <sys/sem.h>	//Used for semaphores   
#define BAUDRATE B9600
#define MODEMDEVICE "/dev/rfcomm0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

unsigned char tempxor = 0;	// temporary XOR
char *csx_dump_fn="cadi_settings_dump";

#define RXM_SET		11
#define RXM_CMD		12
#define RXM_STATUS	13
#define RXM_CONF	17


#define PING_DIVIDER		10		//
#define SETTINGS_PACKET_SIZE	410		// Settings Dump Packet size, received from Cadi

#define RX_BUFF_SIZE		1000		//
#define FIFO_RX_BUFF_SIZE	1000		//
#define FIFO_TX_BUFF_SIZE	1000		//
#define SET_BUFF_SIZE		1000		//
#define SET_DUMP_SIZE		1000		//
#define CONF_BUFF_SIZE		1000		//
#define STAT_BUFF_SIZE		1000		// 
#define CSV_STR_SIZE		400		// 


#define TX_TIMEOUT		100		// number of times te try sending packet, until it is executrd (ZX7 received)
  
#define CSD_SIZE		512		// Cadi Settings Dump size, bytes (for automatic recieve)
#define CSD_OFFSET		0x05C0		// (SETTINGS_START_ADDR)
#define LOCAL_DUMP_SIZE		4096		// local dump file size

//----- SEMAPHORE -----
//On linux systems this union is probably already defined in the included sys/sem.h, but if not use this default basic definition:
union semun {
	int val;
	struct semid_ds *buf;
	unsigned short *array;
};

#define	SEMAPHORE_KEY			674839575  			//Semaphore unique key
#define	SHARED_MEMORY_KEY 		910538672   		//Shared memory unique key
// ===== Semaphore functions ====
static int semaphore1_get_access(void);
static int semaphore1_release_access(void);
// ===== Semaphore variables ====
static int semaphore1_id;

//----- SHARED MEMORY -----
// first 100 bytes for single vars
struct sm1_struct {
	// RX
	unsigned int w_rx_pointer;				// general RX FIFO write pointer
	unsigned int r_rx_pointer;				// general RX FIFO read pointer
	unsigned int w_stat_bp;					// write pointer for Status Buffer
	unsigned int w_set_bp;					// write pointer for Settings Buffer
	unsigned int w_setd_bp;					// write pointer for Settings Dump Buffer
	unsigned int w_conf_bp;					// write pointer for Command Execution Confirmations Buffer
	// TX
	unsigned int w_tx_pointer;				// general TX FIFO write pointer
	unsigned int r_tx_pointer;				// general TX FIFO read pointer

	// daemon settings
	unsigned char stream_status;				// stream Cadi status flag
	unsigned char txbuff_ne;				// 'TX Buffer Not Empty' flag
	unsigned char tx_priority;				// TX priority (daemon buffer or shared memory buffer)
	unsigned char dstate;				// TX priority (daemon buffer or shared memory buffer)


	unsigned char ping_delay;				// number of cycles to delay ping for

	// buffers
	unsigned char empty_arr[63];				// reserved space (to fulfill 100 bytes space)
	unsigned char csv_string[CSV_STR_SIZE];			// CSV string, extracted from binary buffer
	unsigned char status_buf[STAT_BUFF_SIZE];		// FIFO for status packets
	unsigned char settings_buf[SET_BUFF_SIZE];		// FIFO for settings packets
	unsigned char settings_dump[SET_DUMP_SIZE];		// FIFO for settings dump
	unsigned char confirm_buf[CONF_BUFF_SIZE];		// FIFO for conmand execution confirmations
	unsigned char fifo_rx[FIFO_RX_BUFF_SIZE];		// general binary data buffer for data coming from Cadi
	unsigned char fifo_tx[FIFO_TX_BUFF_SIZE];		// general binary data buffer for data coming from Cadi
};

int n=0;

int sm1_id;			// shared memory ID (issue 'ipcs' command in console)
void *sm1_pointer = (void *)0;
struct sm1_struct *sm1;



void initialise(void);
unsigned int push2fifo(unsigned int amount);
unsigned char crc_block(unsigned char input, unsigned char *start_byte, unsigned char length);
unsigned char packet_rx_handler(unsigned char packet_type);
unsigned int push_rx_packet(unsigned char packet_type, unsigned int packet_length);
unsigned char cadi_ping(void);
void push_tx(void);
void tx_flush(void);
unsigned char smtx2dtx(void);		// copy packet from shared memory array into local daemon's TxBuffer storage
void tx_flush(void);
void flush_sm(void);
unsigned char new_packet_id(void);
void dump_csd(void);
void dump_csd_(unsigned int csd_packet_size, unsigned int csd_packet_offset);

/*
TRANSMISSION MACHINE. Priority: TX speed
Transmission machine has 3 main states


              |--------------<-----------<--------<-------<-----<-<-----<-------------------------------|
              |                                                                                         |
              |                                                                                         ^
              |                                 |-<--<---<--                                            |
              |                                 |          |                                            |
              |                 ------------    |    ------^-----                                       |
              |----<----<-------| dstate=2 |    |    |parse_pckt|                                       |
              |                 ------------    |    ------------                                       |
              |                      ^ yes      |          ^ yes                                        |
        ______|_____yes---------^----------  no | ---------^----------                                  |
   IDLE |dstate==0?|->-| wtxpntr>rtxpntr? |--->->-| N = bytes read   |                                  ^
	------------   --------------------       |      N > 0?      |                                  |
	     | no                                 --------------------                                  |
	     |	                                           | no                                         |
	     |	                                      -------------- yes----------                      |
	     |	                                      |tx_repeat>0?| ->-|dstate=2|-->---->----->----->--|
	     |	                                      --------------    ----------                      |
	     |	                                            | no                                        |
	     |	                                     ------------------- yes------------------------- no|
	     |	                                     |stream_status==1?|->--|cycle_cntr%ping_div==0?|->-|
	     |	                                     -------------------    -------------------------   |
	     |	                                                                         | yes          |
	     |	                                                                     ----------         |
	     |			                                                     |dstate=3|---->>>--|
	     |                                                                       ----------         |
	     |	                                                                                        |
	------------   yes  --------------  yes                                                         |
        |dstate==2?|-->-->--|tx_repeat>0?|->-->------->------>------------->------>-----|               |
	------------        --------------                                              |               |
	     |	                   | no                                                 |               |
	     |                     |             ---------------                        |               |
	     |                     |             |tx_repeat=100|->--------->--------->--|               |
	     |                     |             ---------------                        |               |
	     |                     |                    ^                               |               |
	     |                     |                    | yes                    --------------         |
	     |             ---------------------   -----------   -------------   |send packet |         |
	     |             | TX: fill TxBuffer |->-|pkt_id>0?|->-|tx_repeat=1|->-|  dstate=0  |---------^
	     |             ---------------------   -----------   -------------   |tx_repeat-- |         |
	     |                                                                   --------------         |
	____________  yes  -----------------------   -------------------------                          |
	|dstate==3?|--->---| PING: fill TxBuffer |->-| send packet, dstate=0 |----->----->-------->---->|
	------------       -----------------------   -------------------------
	     |
	   -----
	   |STOP|
	   ------




TRANSMISSION MACHINE. Priority: TX quality
Transmission machine has 3 main states

          =========
          | START |
              |---------<--------<-------<--------<-------<------<------<-------<--------<-------<------|
              |             ------^-----                ^                                               ^
              |             | dstate=1 |                ^                                               |
              |             ------------                ^                                               |
              |                   ^ yes                 ^                                               |
              |      no ----------^---------            ^                                               |
              |<----<---|ZX7 && setrxok==5?|            ^                                               |
              |         --------------------      ------^-----                                          |
              |                   ^               | dstate=2 |                                          |
              |             ------^-------        ------------                                          |
              |             |parse_packet|              ^                                               |
              |             --------------              ^                                               |
              |                   ^ yes                 ^ yes                                           |
        ------------  yes---------^----------   no------^-------                                        |
   IDLE |dstate==0?|->->-| N = bytes read   |->---|tx_repeat>0?|                                        |
	------------     |      N > 0?      |     --------------                                        |
	     | no        --------------------           | no                                            |
	     |	                                ------------------ yes   ----------                     |
                                                |wtxpntr>rtxpntr?|->-->--|dstate=2|-->-->->--->--->-----^
                                                ------------------       ----------                     |
                                                        | no                                            |
	     |	                                ------------------- yes------------------------- no     |
	     |	                                |stream_status==1?|->--|cycle_cntr%ping_div==0?|->--->--^
	     |	                                -------------------    -------------------------        |
	     |	                                         | no                       | yes               |
	     |	                                         |                      ----------              |
	     |			                         |                      |dstate=3|-->--->--->---^
	     |                                           |                      ----------              |
             |                                            ._____________________________________________^
	------------   yes  --------------  yes                                                         |
        |dstate==2?|-->-->--|tx_repeat>0?|->-->------->------>------------->------>----.                |
	------------        --------------                                              |               |
	     |	                   | no                                                 |               |
	     |          yes ----------------     ---------------                        |               ^
	     |       .----<-| get settings |     |tx_repeat=100|->--------->--------->--|               |
	     | -----------  |   request?   |     ---------------                        |               |
	     | |setrxok=5|  ----------------            ^                               |               |
	     | -----------         | no                 | yes                    -------^------         |
	     |      |      ---------------------   -----^-----   -------------   |send packet |         |
	     |       >-->--| TX: fill TxBuffer |->-|pkt_id>0?|->-|tx_repeat=1|->-|  dstate=0  |---->----^
	     |             ---------------------   -----------   -------------   |tx_repeat-- |         |
	     |                                                                   --------------         |
	____________  yes  -----------------------   -------------------------                          |
	|dstate==3?|--->---| PING: fill TxBuffer |->-| send packet, dstate=0 |----->----->-------->-----^
	------------       -----------------------   -------------------------                          |
	     |                                                                    -----------           |
	     |                                                       .-------->---|setrxok=0|->---->----^
	     |                                                      | no          -----------           |
	     |                                               yes-----^------                             |
             |                                          .--<---|timeout>0?|<----.                       ^
             |                                          |      ------------      |                      |
	------------  yes---------------      ----------|------- no-----------   |                      |
	|dstate==1?|-->--|             |->-->-| N = bytes read |->-|timeout--|->-^                      |
	------------     | timeout=100 |      |     N > 0?     |   -----------   |                      |
                         |             |      ------------------                 |                      ^
                         ---------------               | yes                     ^                      |
                                                       |                         |                      |
                                            -----------------------              |                      |
                                            |    setrxpntr+=N     | no           |                      |
                                            |setrxpntr>setblksize?|->--->--->----^                      ^
                                            -----------------------                                     |
                                                       | yes                                            |
                                              -------------------- no        -----------                |
                                              | settings CRC ok? |->->---->--|setrxok=0|--->--->---->---^
                                              --------------------           -----------                |
                                                       | yes                                            |
                                              -------------------     ----------        -----------     |
                                              |save dump to file|-->--|dstate=0|-->-->--|setrxok=1|-->--^
                                              -------------------     ----------        -----------



*/


unsigned char buf[255];		// buffer for Serial read 
unsigned char rxm_state = 0;
unsigned char rx_buf[255];	// buffer for packet detector
unsigned char z = 0;		// for loop pointer 
unsigned char stream_status_recovery = 0;

int rx_pntr = 0;
int packet_length = 0;
int prefixDetectionIdx = 0;
unsigned int b = 0;


#define LOCAL_RX_BUFF_SIZE	2000
#define LOCAL_TX_BUFF_SIZE	100

unsigned char RxBuffer[LOCAL_RX_BUFF_SIZE];
unsigned char TxBuffer[LOCAL_TX_BUFF_SIZE];		// temp TX buffer
unsigned int RxCounter = 0;		// temp RX buffer pointer
unsigned char TxCounter = 0;		// temp TX buffer pointer

unsigned int ping_delay = 0;

volatile int STOP=FALSE;       
int fd, res;
unsigned char dstate = 0;		// daemon transmission state
unsigned int cycle_cntr = 0;		// daemon cycle runs couter
unsigned int tx_repeat = 0;		// number of timer to repeat the TX packet, until corresponding ZX7 packet received
unsigned char packet_id = 0;		// compared to ZX7 response
unsigned int timeout = 0;		// machine timeout
unsigned char setrxok = 0;		// settings received fine
unsigned int NbrOfDataToTransfer = 0;	// Number of bytes to transfer
unsigned int NbrOfDataToRead = 0;	// Number of bytes to read
unsigned int ping_div = PING_DIVIDER;	// status request executes every PING_DIVIDER cycle run
unsigned char rx_packet_crc = 0;	// RX packet CRC
unsigned char packet_ready = 0;		// Packet Ready flag for Data Exchange machine
unsigned char old_stream_status = 0;	// previous status stream for changes detection
unsigned int csd_offset = 0;		// csd offset of received settings packet


int main()
      {

	//GENERAL INITIALISE
	initialise();

	FILE *fp = NULL;
	fp=fopen("/srv/http/cm/serialresp.out", "w+");
	if(fp != NULL) {	// if serialresp.out opened ok
	    
		struct termios oldtio,newtio;
		
//		fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );	// blocking read 
		fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK); 
		if (fd <0) {perror(MODEMDEVICE); exit(-1); }
		
		tcgetattr(fd,&oldtio); /* save current port settings */
		
		bzero(&newtio, sizeof(newtio));
		newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		
		/* set input mode (non-canonical, no echo,...) */
		newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		// Disable Software Flow control
		newtio.c_iflag &= ~(IXON | IXOFF | IXANY);


		newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
		newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 char received */
		
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd,TCSANOW,&newtio);

		flush_sm();	// flush shared memory

		// ue nanosleep() if needed
		struct timespec reqtime;
        	reqtime.tv_sec = 0;
        	reqtime.tv_nsec = 1000000;


		while (dstate<4) {       /* loop for input */
			sm1->dstate = dstate;
			usleep(100000);	// delay between cycles		HARDCODE!!!
        		// printf("cyclecntr++\n");
			// printf("dstate = \n");
			// printf("%x\n\n",dstate);

			

			// Status Stream change detection
			if (old_stream_status == sm1->stream_status){

			}
			else {
				old_stream_status = sm1->stream_status;
				printf("=== Status Stream Changed to: ");
				printf("%d\n ", (sm1->stream_status));
			}


			switch (dstate) {

				case 0:		// IDLE
					res=1; // to enter 'while' (NEEDS COMPENSATION)
					while (res>0) {
						res = 0;	// COMPENSATION
						res = read(fd,buf,1);   /* returns after 1 char have been input */
						buf[res]=0;               /* so we can printf... */
						if (res>0) {
							push2fifo(res);
#ifdef TEST_MODE
							printf("%d: ", rx_pntr, res);
							printf("%x\n\n", buf[0], res);
#endif
						}
					}
					if (tx_repeat>0) {
						dstate=2;
#ifdef TEST_MODE
						printf("1: dstate = 2\n");
#endif
						sm1->ping_delay = 10;
						//sm1->stream_status = 0;
					}
					else {		// 'wtxpntr' and 'rtxpntr'
						if (sm1->txbuff_ne == 1) {
							usleep(1000000);
							dstate = 2;
#ifdef TEST_MODE
							//stream_status_recovery = sm1->stream_status;
							//sm1->stream_status = 0;
							 
							printf("sm1->txbuff_ne = 1\n");
							printf("2: dstate = 2\n");
#endif
						}
						else {
							printf("sm1->txbuff_ne=0, tx_repeat=0\n");
							if (sm1->stream_status==1 && cycle_cntr%ping_div==0 && (sm1->ping_delay)==0) {
								dstate = 3;	// going to ping remote device
#ifdef TEST_MODE
								printf("3: dstate = 3\n");
#endif
							}
							else {
								if ((sm1->ping_delay)>0) {
									sm1->ping_delay--;
								}
#ifdef TEST_MODE
								printf(" \n");
#endif
							}
						}
					}
				break;

				case 1:		// receive settings
#ifdef TEST_MODE
					printf("\n");
#endif
					RxCounter = 0;
					timeout = 1000;
					NbrOfDataToRead = SETTINGS_PACKET_SIZE;
					tempxor = 51;		// xor for 'ZX1'

					printf("NbrOfDataToRead: ");
					printf("%d\n", NbrOfDataToRead);

					while (NbrOfDataToRead>RxCounter && timeout>0) {
						res = read(fd,buf,1);
						tempxor ^= buf[0];
						if (RxCounter==0) {
							packet_length = ((unsigned int)(buf[0]<<8))&0xFF00;
						}
						if (RxCounter==1) {
							packet_length |= ((unsigned int)buf[0])&0xFF;
							NbrOfDataToRead = (packet_length-5);
#ifdef TEST_MODE
							printf("=== PACKET LENGTH: ");
							printf("%d\n\n", packet_length, res);
							printf("=== NbrOfDataToRead: ");
							printf("%d\n", NbrOfDataToRead);
#endif
						}
						if (res>0) {
							//printf("receiving settings\n");
#ifdef TEST_MODE
							printf("VAL: ");
							printf("%d", buf[0], res);
							printf(" / XOR: ");
							printf("%d\n", tempxor, res);
							printf("counter: ");
							printf("%d\n", RxCounter, res);
							timeout = 1000;
#endif
							RxBuffer[RxCounter++] = buf[0];
						}
						else {
							usleep(3000);
							timeout--;
						}
					}
					
					printf("timeout: ");
					printf("%d\n", timeout);
					dstate = 0;



					csd_offset = (((unsigned int)RxBuffer[3])&0x00FF) + (((unsigned int)RxBuffer[2]<<8)&0xFF00);
					dump_csd_(packet_length, csd_offset);
#ifdef TEST_MODE
					printf("Switching to IDLE\n");
#endif
					break;

				case 2:		// TX
					printf("tx_repeat = ");
					printf("%d\n", tx_repeat);
					usleep(50000);	// delay, before sending paket, to ensure, all transfers stopped
					if (tx_repeat>0) {
						// repeating previous packet
					}
					else {
						packet_id = smtx2dtx();	// copy shared memory TX buffered packet into daemon TxBuff					
						
						if (TxBuffer[2]==50 && TxBuffer[4]==61) {  // settings dump request packet
							setrxok = 5;	// get ready for receiving memory dump
#ifdef TEST_MODE
							printf("setrxok = 5\n");
#endif
						}
						if (packet_id>0) {	// packet is going to be repeated until ZX7 received
							tx_repeat = TX_TIMEOUT;
							printf("tx_repeat = TX_TIMEOUT\n");
						}
						else {
							tx_repeat = 1;
							printf("tx_repeat = 1\n");
						}
					}

					push_tx();	// send buffered packet
					tx_repeat--;
					dstate = 0;
#ifdef TEST_MODE
					printf("4: dstate = 0\n");
#endif
				break;

				case 3:		// PING: Status packet request
					if ((sm1->stream_status) == 1 && ping_delay==0) {
						cadi_ping();
						dstate = 0;
#ifdef TEST_MODE
						printf("5: dstate = 0\n");
#endif
					}

					// used to pause Cadi Status streaming
					if (ping_delay>0) {
						ping_delay--;
					}
				break;
			}
		
			cycle_cntr++;
			if (cycle_cntr>1000000000) {
				cycle_cntr = 0;
			}
		}
		tcsetattr(fd,TCSANOW,&oldtio);
	}
	fclose(fp);
	return 0;
}

#define CSD_SIZE		512		// Cadi Settings Dump size, bytes (for automatic recieve)
#define CSD_OFFSET		0x05C0		// (SETTINGS_START_ADDR)
#define LOCAL_DUMP_SIZE		4096		// local dump file size


void dump_csd_(unsigned int csd_packet_size, unsigned int csd_packet_offset){
	unsigned int i = 0;
	unsigned int dump_size = 0;
	
	// unsigned char buffer[10];


	printf("csd_packet_size ");
	printf("%d\n", csd_packet_size);
	printf("csd_packet_offset: ");
	printf("%d\n", csd_packet_offset);



	// mozhno ne ispol'zovat' vhodjashie argumenty
	// get size of CSD packet (LSB first)
//	csd_packet_size = (((unsigned int)RxBuffer[1])&0x00FF) + (((unsigned int)RxBuffer[0]<<8)&0xFF00);
	// and Start Address offset
//	csd_packet_offset = (((unsigned int)RxBuffer[3])&0x00FF) + (((unsigned int)RxBuffer[2]<<8)&0xFF00);


	FILE *write_ptr;

	write_ptr = fopen(csx_dump_fn,"wb");  // w for write, b for binary
	
	fseek(write_ptr, (csd_packet_offset*2), SEEK_SET);
	
	// build array for saving into dump file
	dump_size = csd_packet_size - 12;	// 7 bytes preliminary, and 5 bytes postliminary
	unsigned char dumparr[10000];		// temporary byte array for dump file
	for (i=0;i<dump_size;i++) {
		dumparr[i] = RxBuffer[(i+4)];
	}
	fwrite(dumparr,dump_size,1,write_ptr); 		// HARDCODE - offset within RxBuffer
	fflush(write_ptr);
	fclose(write_ptr);
	
	printf("Written ");
	printf("%d", dump_size);
	printf("bytes\n Starting from: ");
	printf("%d", (csd_packet_offset*2));
	printf("\n");
}

void flush_sm(void){
		// init pointers and Shared Memory arrays
		sm1->w_rx_pointer = 0;
		sm1->r_rx_pointer = 0;
		sm1->w_stat_bp = 0;
		sm1->w_set_bp = 0;
		sm1->w_setd_bp = 0;
		sm1->w_conf_bp = 0;
		sm1->stream_status = 0;
		sm1->dstate = 0;

		unsigned int x=0;
		for (x=0;x<(STAT_BUFF_SIZE-1);x++) {
			sm1->status_buf[x] = 0;
		}

		for (x=0;x<(CONF_BUFF_SIZE-1);x++) {
			sm1->confirm_buf[x] = 0;
		}

		for (x=0;x<(SET_BUFF_SIZE-1);x++) {
			sm1->settings_buf[x] = 0;
		}

		for (x=0;x<(SET_DUMP_SIZE-1);x++) {
			sm1->settings_dump[x] = 0;
		}

		for (x=0;x<(CSV_STR_SIZE-1);x++	) {
			sm1->csv_string[x] = 0;
		}

		for (x=0;x<(FIFO_RX_BUFF_SIZE-1);x++	) {
			sm1->fifo_rx[x] = 0;
		}

		for (x=0;x<(FIFO_TX_BUFF_SIZE-1);x++	) {
			sm1->fifo_tx[x] = 0;
		}

		for (x=0;x<60;x++) {
			sm1->empty_arr[x] = 0;
		} 


}


void push_tx(void){
#ifdef TEST_MODE
	printf(">>> Starting Pushing TX packet into serial...\n");
#endif
	sm1->txbuff_ne=0;	// stop Tx transfers
	NbrOfDataToTransfer = TxBuffer[3];				// packet size	
	TxBuffer[TxBuffer[3]] = 13;
	TxCounter=0;							// reset tx buffer pointer
	sm1->txbuff_ne = 1;
	// BT_USART->DR = 0;						// volshebnyj pendal (STM32)

#ifdef TEST_MODE
	printf("NbrOfDataToTransfer = ");
	printf("%d\n",NbrOfDataToTransfer);
	printf("TxCounter = ");
	printf("%d\n",TxCounter);

	printf("---------------- \n");
#endif
	while (TxCounter<NbrOfDataToTransfer){
//		vTaskDelay(5);
		res = write(fd,&TxBuffer[TxCounter],1);
		TxCounter++;
		// IWDG_ReloadCounter();				// watchdog reload (STM32)
	}


	NbrOfDataToTransfer=0;
	sm1->txbuff_ne=0;	// packet sent, reset tx not empty flag
#ifdef TEST_MODE
	printf("NbrOfDataToTransfer = ");
	printf("%d\n",NbrOfDataToTransfer);
	printf("TxCounter = ");
	printf("%d\n",TxCounter);
	printf("sm1->txbuff_ne = ");
	printf("%d\n",sm1->txbuff_ne);



#endif
	// tx_flush();		// flush Tx buffer
#ifdef TEST_MODE
	printf(">>> PUSHED!\n");
#endif
}

// copy packet from Shared Memory TX FIFO into daemon's TxBuffer local array
unsigned char smtx2dtx(void){
	unsigned int i = 0;
	unsigned int packet_length = 0;
	unsigned char pckt_id = 0;
#ifdef TEST_MODE	
			printf("Packet length2\n");
			printf("%d\n",sm1->fifo_tx[2]);
			printf("Packet length3\n");
			printf("%d\n",sm1->fifo_tx[3]);
			printf("Packet length4\n");
			printf("%d\n",sm1->fifo_tx[4]);
			printf("Packet length5\n");
			printf("%d\n",sm1->fifo_tx[5]);
			printf("Loading sm1->fifo_tx into TxBuffer\n");

#endif
		packet_length = sm1->fifo_tx[3];
		
		for (i=0;i<packet_length;i++) {
			TxBuffer[i] = sm1->fifo_tx[i];
// #ifdef TEST_MODE	
			printf("sm1->fifo_tx[");
			printf("%d",i);
			printf("]: ");
			printf("%d\n",sm1->fifo_tx[i]);
// #endif
		}
		sm1->txbuff_ne = 0;

	printf("TxBuffer[4]");
	printf("%d\n",TxBuffer[4]);
	
	if (TxBuffer[4]<50) {		// check if command if > than 50 or not. 0..50 need confirmations
		pckt_id = new_packet_id(); // the last byte of packet is Packet ID
		printf("Assigned pckt_id=");
		printf("%d\n",pckt_id);
	}
	else if (TxBuffer[4]>99 && TxBuffer[4]<200) {
		pckt_id = new_packet_id();	// same bahavior
		printf("Assigned pckt_id=");
		printf("%d\n",pckt_id);
	}
	else {
		pckt_id = 0;	// packet_id=0 does not need ZX7 response
	}
	
	printf("TxBuffer[4] (cmd_id)");
	printf("%d\n",TxBuffer[4]);

	TxBuffer[(TxBuffer[3]-1)] = pckt_id;

	printf("TxBuffer[");
	printf("%d",(TxBuffer[3]-1));
	printf("]: ");
	printf("%d\n",pckt_id);

	printf("pckt_id");
	printf("%d\n",pckt_id);


	return pckt_id;

}

unsigned char new_packet_id(void){
	if (packet_id<255) {
		packet_id++;
	}
	else {
		packet_id = 0;
	}
	return packet_id;
}

void tx_flush(void){
	unsigned int i=0;
	for (i=0;i<LOCAL_TX_BUFF_SIZE;i++) {
		TxBuffer[i] = 0;
	}
}


// function sends status request packet to Cadi
unsigned char cadi_ping(void){
// \x5a\x58\x32\x04\x33\x01\x06\x00 - Protobuzz v2 Ping Packet

#ifdef TEST_MODE
	printf(">>> Pinging Cadi\n");
#endif

	unsigned char i=0;

	// Protobuzzz v3 style packet: status request
	TxBuffer[i++] = 90;		// 'Z'
	TxBuffer[i++] = 88;		// 'X'
	TxBuffer[i++] = 50;		// '2'
	TxBuffer[i++] = 8;		// packet size
	TxBuffer[i++] = 51;		// command id (51 - get status block)
	TxBuffer[i++] = 1;		// Argument (status block id)
	TxBuffer[i++] = 10;		// CRC
	TxBuffer[i++] = 0;		// packet id
	push_tx();
	return 1;		// ok
}


unsigned int push_rx_packet(unsigned char packet_type, unsigned int packet_length){
	unsigned int tmcntr = 0;

#ifdef TEST_MODE
	printf("III BEFORE: RxBuffer[3] = ");
	printf("%d\n",RxBuffer[3]);

	tmcntr=0;
	for (tmcntr=0; tmcntr<100;tmcntr++) {

	/*	printf("III Contents of: ");
		printf("%d",tmcntr);
		printf(" is  ");
		printf("%d\n",sm1->status_buf[tmcntr]); */
	}
#endif

	for (tmcntr=0; tmcntr<packet_length;tmcntr++) {
		switch (packet_type) {
			case 11:
				sm1->settings_buf[(tmcntr+3)] = RxBuffer[tmcntr];
				break;
			case 13:
				sm1->status_buf[(tmcntr+3)] = RxBuffer[tmcntr];
#ifdef TEST_MODE
				// printf("+++ Storing RxBuffer into sm1->status_buf. RxBuffer[");
				//printf("%d",(tmcntr+3));
				//printf("]: ");
				//printf("%d\n",RxBuffer[tmcntr]);
#endif
				break;
			case 17:
				sm1->confirm_buf[(tmcntr+3)] = RxBuffer[tmcntr];
				break;
		}
		
	}

#ifdef TEST_MODE
/*	printf("III AFTER: RxBuffer[3] = ");
	printf("%d\n",RxBuffer[3]);
	tmcntr=0;
	for (tmcntr=0; tmcntr<100;tmcntr++) {
		printf("III Conetents of: ");
		printf("%d",tmcntr);
		printf(" is  ");
		printf("%d\n",sm1->status_buf[tmcntr]);
	} */
#endif

	// handle packet
	switch (packet_type) {
		case 11:
			// process settings data
			break;
		case 13:
			// parse status packet
			break;
		case 17:
			// reset tx_repeat
			if (RxBuffer[(RxBuffer[3]-1)] == (packet_id-1)) {
				tx_repeat = 0;
				dstate = 0;
				//sm1->stream_status = stream_status_recovery;
			}
			else {	// packet id wrong or expired
	
			}
			break;
	}
	for (z=0; z<packet_length;z++) {
		RxBuffer[z] = 0;	// flush rx buff afterwards
	}
	
	return 1;	// ok
}


unsigned int push_rx_packet_fifo(unsigned char packet_type, unsigned int packet_length){
	for (z=0; z<packet_length;z++) {
		switch (packet_type) {
			case 11:
				sm1->settings_buf[sm1->w_set_bp++] = RxBuffer[z];
				if (sm1->w_set_bp==(SET_BUFF_SIZE-1)) {
					sm1->w_set_bp = 0;	// reset pointer if needed
				}
				break;
			case 13:
				sm1->status_buf[sm1->w_stat_bp++] = RxBuffer[z];
				if (sm1->w_stat_bp==(STAT_BUFF_SIZE-1)) {
					sm1->w_stat_bp = 0;	// reset pointer if needed
				}
				break;
			case 17:
				sm1->confirm_buf[sm1->w_conf_bp++] = RxBuffer[z];
				if (sm1->w_conf_bp==(CONF_BUFF_SIZE-1)) {
					sm1->w_conf_bp = 0;	// reset pointer if needed
				}
				break;
		}
		
	}

	// handle packet
	switch (packet_type) {
		case 11:
			// process settings data
			break;
		case 13:
			// parse status packet
			break;
		case 17:
			// reset tx_repeat
			if (RxBuffer[(RxBuffer[3]-1)] == packet_id) {
				tx_repeat = 0;
				//sm1->stream_status = stream_status_recovery;
			}
			else {	// packet id wrong or expired
	
			}
			break;
	}
	for (z=0; z<packet_length;z++) {
		RxBuffer[z] = 0;	// flush rx buff afterwards
	}
	
	return 1;	// ok
}




unsigned int push2fifo(unsigned int amount){
	int i = 0;
	  //----- SEMAPHORE GET ACCESS -----
	if (semaphore1_get_access()) {
		
		//printf("push2fifo2\n");
		while (i < amount) {
			//----- ACCESS THE SHARED MEMORY -----
			//Just an example of reading 2 bytes values passed from the php web page that will cause us to exit
			/*sm1->fifo_rx[sm1->w_rx_pointer++] = buf[i];  // write ok

			if (!(sm1->w_rx_pointer<RX_BUFF_SIZE)) {
				sm1->w_rx_pointer = 0;
			} */


		//printf("push2fifo2-2\n");
			if (rxm_state==RXM_STATUS) {
				RxBuffer[rx_pntr] = buf[i];	// packet buffer
				tempxor ^= buf[i];
#ifdef TEST_MODE
//				printf(" Temp XOR:  ");
//				printf("%x\n",tempxor);
#endif
				if (rx_pntr==0) {
					packet_length = buf[i];
				
#ifdef TEST_MODE
					printf("=== STATUS Packet LENGTH:  ");
					printf("%x\n",packet_length);
#endif
				}
				
				
		//printf("push2fifo2-3\n");
			
				// when last byte of the packet received, add header for next packet and stop RXM =0)
				if (rx_pntr==(packet_length-3)) {	// move packet from temp buffer to orresponding one
				
					rx_packet_crc = crc_block(49, &RxBuffer[0], (packet_length-4));
#ifdef TEST_MODE
					printf("=== Counted CRC = ");
					printf("%x\n\n",rx_packet_crc);
#endif
				
					if (rx_packet_crc == 0) {	// packet is accepted if CRC is OK only
						rx_pntr = 0;
						prefixDetectionIdx = 0;
						//rxm_state -= 38;
						
						
						sm1->status_buf[0]= 90;	// Z
						sm1->status_buf[1]= 88;	// X
						sm1->status_buf[2]= 51;	// 3
#ifdef TEST_MODE
						printf("Got status packet from Cadi!\n");
						printf("Pushig status binary string into sm1->status_buf[]\n");
#endif
						push_rx_packet(rxm_state, packet_length);
					
					}
					
#ifdef TEST_MODE
					printf("Resetting rx_pntr, rxm_state=0\n");
#endif
					rx_pntr = 0;		// reset rx pointer (NOT NEEDED?)
					rxm_state = 0;		// finish packet reception
				}
			 
				rx_pntr++;
			}


			if (rxm_state==RXM_CONF) {
				RxBuffer[rx_pntr] = buf[i];	// packet buffer
				tempxor ^= buf[i];
#ifdef TEST_MODE
				printf("<<< Confirmation XOR: ");
				printf("%x\n",tempxor);
#endif
				if (rx_pntr==0) {
					packet_length = buf[i];
				
#ifdef TEST_MODE			
					printf("<<< Confirmation Packet_length: ");
					printf("%x\n",packet_length);
#endif
				}
				
				
		//printf("push2fifo2-3\n");
			
				// when last byte of the packet received, add header for next packet and stop RXM =0)
				if (rx_pntr==(packet_length-3)) {	// move packet from temp buffer to orresponding one
				
					rx_packet_crc = crc_block(53, &RxBuffer[0], (packet_length-4));		// 53 = xor(ZX7)
#ifdef TEST_MODE
					printf("<<< Confirmation CRC: ");
					printf("%d\n",rx_packet_crc);
#endif
				
					if (rx_packet_crc == 0) {	// packet is accepted if CRC is OK only
						rx_pntr = 0;
						prefixDetectionIdx = 0;
						
						
						sm1->confirm_buf[0]= 90;	// Z
						sm1->confirm_buf[1]= 88;	// X
						sm1->confirm_buf[2]= 55;	// 1
#ifdef TEST_MODE
						printf("DBG Got Confirmation! ");
						printf("%d\n",RxBuffer[1]);

						printf("DBG Expected ");
						printf("%d\n",(packet_id));
						
#endif
						if (RxBuffer[1]==(packet_id)) {
							printf("AND Confirmation is OK! \n ");
							tx_flush();
							tx_repeat = 0;
							dstate = 0;
							sm1->txbuff_ne = 0;
							//sm1->stream_status = stream_status_recovery;
						} 
						push_rx_packet(rxm_state, packet_length);
					
					}
#ifdef TEST_MODE
					printf("III Resetting rx_pntr: ");
#endif
					rx_pntr = 0;		// reset rx pointer (NOT NEEDED?)
					rxm_state = 0;		// finish packet reception
				}
			 
				rx_pntr++;
			 }



			if (rxm_state==RXM_SET) {
			

			}


			// === RXMachine states processing, depending on packet headers
			if (buf[i]==90) {
				prefixDetectionIdx = 1;		// ready
#ifdef TEST_MODE
				printf("Ready!\n");
#endif
				sm1->empty_arr[0]++;
			}
			else if (prefixDetectionIdx == 1 && buf[i]==88) {
				prefixDetectionIdx = 2;		// steady
#ifdef TEST_MODE
				printf("Steady!\n");
#endif
				sm1->empty_arr[1]++;
			}
			else if (prefixDetectionIdx==2) {
				if (buf[i]==48) {
					// escaping zero
				}
				else if (buf[i]==49){
					// settings
					// rxm_state=RXM_SET;
					RxCounter = 0;
					rx_pntr=0;
					dstate = 1;
					setrxok = 5;
					res = 0;	// to exit IDLE state
#ifdef TEST_MODE
					printf("Setrxok = 5\n");
					printf("6: dstate=1\n");
#endif
				}
				else if (buf[i]==51) {
					RxCounter = 0;
					// status
#ifdef TEST_MODE
					printf("STAAAATTUUUS\n");
#endif
					rxm_state=RXM_STATUS;
					tempxor = 49;
					rx_pntr=0;
					// rx_flush();
				}
				else if (buf[i]==55) {
					RxCounter = 0;
					// confirmation
#ifdef TEST_MODE
					printf("=== CONFIRMATION RECEIVED ===\n");
#endif
					rxm_state=17;
					rx_pntr=0;
					// rx_flush();
				}
				else {
					RxCounter = 0;
					rxm_state = 0;
					rx_pntr=0;
				}
				prefixDetectionIdx = 0;
			}


			i++;

		}

		//----- SEMAPHORE RELEASE ACCESS -----
		semaphore1_release_access();
		//char *message5="push2fifo message5";
		//printf("%s\n",message5);
		return(9);
	}
	else {
		return(11);
	}
}



//********************************
//********************************
//********** INITIALISE **********
//********************************
//********************************
void initialise (void)
{

	//..... Do init stuff ....

	//-----------------------------------------------
	//----- CREATE SHARED MEMORY WITH SEMAPHORE -----
	//-----------------------------------------------
	printf("Creating shared memory with semaphore...\n");
	semaphore1_id = semget((key_t)SEMAPHORE_KEY, 3, 0666 | IPC_CREAT);  //Semaphore key, number of semaphores required, flags
	//	Semaphore key 
	//		Unique non zero integer (usually 32 bit).  Needs to avoid clashing with another other processes semaphores (you just have to pick a random value and hope - ftok() can help with this but it still doesn't guarantee to avoid colision)

	//Initialize the semaphore using the SETVAL command in a semctl call (required before it can be used)
	union semun sem_union_init;
	sem_union_init.val = 1;
	if (semctl(semaphore1_id, 0, SETVAL, sem_union_init) == -1)
	{
		fprintf(stderr, "Creating semaphore failed to initialize\n");
		exit(EXIT_FAILURE);
	}

	//Create the shared memory
	sm1_id = shmget((key_t)SHARED_MEMORY_KEY, sizeof(struct sm1_struct), 0666 | IPC_CREAT);		//Shared memory key , Size in bytes, Permission flags
	 
	//	Shared memory key
	//		Unique non zero integer (usually 32 bit).  Needs to avoid clashing with another other processes shared memory (you just have to pick a random value and hope - ftok() can help with this but it still doesn't guarantee to avoid colision)
	//	Permission flags
	//		Operation permissions 	Octal value
	//		Read by user 			00400
	//		Write by user 			00200
	//		Read by group 			00040
	//		Write by group 			00020
	//		Read by others 			00004
	//		Write by others			00002
	//		Examples:
	//			0666 Everyone can read and write

	if (sm1_id == -1)
	{
		fprintf(stderr, "Shared memory shmget() failed\n");
		exit(EXIT_FAILURE);
	}

	//Make the shared memory accessible to the program
	sm1_pointer = shmat(sm1_id, (void *)0, 0);
	if (sm1_pointer == (void *)-1)
	{
		fprintf(stderr, "Shared memory shmat() failed\n");
		exit(EXIT_FAILURE);
	}
	printf("Shared memory attached at %X\n", (int)sm1_pointer);

	//Assign the shared_memory segment
	sm1 = (struct sm1_struct *)sm1_pointer;


	//----- SEMAPHORE GET ACCESS -----
	if (!semaphore1_get_access()) {
		exit(EXIT_FAILURE);
	}

	//----- WRITE SHARED MEMORY -----
	int Index;
	for (Index = 0; Index < sizeof(struct sm1_struct); Index++) {
//		sm1->fifo_rx[Index] = 49;
	}

	//Write initial values
	sm1->fifo_rx[0] = 'C';
	sm1->fifo_rx[1] = 'B';
	sm1->fifo_rx[2] = 'T';
	sm1->fifo_rx[3] = 'D';
	sm1->fifo_rx[4] = 'a';
	sm1->fifo_rx[5] = 'e';
	sm1->fifo_rx[6] = 'm';
	sm1->fifo_rx[7] = 'o';
	sm1->fifo_rx[8] = 'n';

	sm1->csv_string[0] = 'C';
	sm1->csv_string[1] = 'S';
	sm1->csv_string[2] = 'V';
	sm1->csv_string[3] = 'S';
	sm1->csv_string[4] = 't';
	sm1->csv_string[5] = 'r';
	sm1->csv_string[6] = 'i';
	sm1->csv_string[7] = 'n';
	sm1->csv_string[8] = 'g';
	sm1->csv_string[9] = 'B';
	sm1->r_rx_pointer = 270;	// init r_rx_pointer with random value



	// RX
	unsigned int w_rx_pointer;				// general RX FIFO write pointer
	unsigned int r_rx_pointer;				// general RX FIFO read pointer
	unsigned int w_stat_bp;					// write pointer for Status Buffer
	unsigned int w_set_bp;					// write pointer for Settings Buffer
	unsigned int w_setd_bp;					// write pointer for Settings Dump Buffer
	unsigned int w_conf_bp;					// write pointer for Command Execution Confirmations Buffer
	// TX
	unsigned int w_tx_pointer;				// general TX FIFO write pointer
	unsigned int r_tx_pointer;				// general TX FIFO read pointer

	// daemon settings
	unsigned char stream_status;				// stream Cadi status flag
	unsigned char txbuff_ne;				// 'TX Buffer Not Empty' flag
	unsigned char tx_prioority;				// TX priority (daemon buffer or shared memory buffer)



	printf("stream_status=%d\n", sm1->stream_status);
	printf("w_rx_pointer=%d\n", sm1->w_rx_pointer);
	printf("r_rx_pointer=%d\n", sm1->r_rx_pointer);
	printf("w_stat_bp=%d\n", sm1->w_stat_bp);
	printf("w_set_bp=%d\n", sm1->w_set_bp);
	printf("w_setd_bp=%d\n", sm1->w_setd_bp);
	printf("w_conf_bp=%d\n", sm1->w_conf_bp);


	printf("w_tx_pointer=%d\n", sm1->w_tx_pointer);
	printf("r_tx_pointer=%d\n", sm1->r_tx_pointer);
	printf("stream_status=%d\n", sm1->stream_status);
	printf("txbuff_ne=%d\n", sm1->txbuff_ne);
	printf("tx_priority=%d\n", sm1->tx_priority);



	// print size of shared memory block
	char *message="Shared memory block size: ";
	printf("%s\n",message);
   	printf("%x %x\n\n",sizeof(struct sm1_struct),sizeof(struct sm1_struct));


	//----- SEMAPHORE RELEASE ACCESS -----
	if (!semaphore1_release_access()) {
		exit(EXIT_FAILURE);
	}
}



//***********************************************************
//***********************************************************
//********** WAIT IF NECESSARY THEN LOCK SEMAPHORE **********
//***********************************************************
//***********************************************************
//Stall if another process has the semaphore, then assert it to stop another process taking it
static int semaphore1_get_access(void)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = -1; /* P() */
	sem_b.sem_flg = SEM_UNDO;
	if (semop(semaphore1_id, &sem_b, 1) == -1)		//Wait until free
	{
		printf("semaphore1_get_access failed\n", 1);
		return(0);
	}
	return(1);
}

//***************************************
//***************************************
//********** RELEASE SEMAPHORE **********
//***************************************
//***************************************
//Release the semaphore and allow another process to take it
static int semaphore1_release_access(void)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = 1; /* V() */
	sem_b.sem_flg = SEM_UNDO;
	if (semop(semaphore1_id, &sem_b, 1) == -1)
	{
		fprintf(stderr, "semaphore1_release_access failed\n");
		return(0);
	}
	return(1);
}


unsigned char crc_block(unsigned char input, unsigned char *start_byte, unsigned char length){
	unsigned char i=0;
	for (i=0; i<length; i++) {
		input ^= start_byte[i];
	}
	return input;
}
