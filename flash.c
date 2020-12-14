#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* Used for Macronix flashes only. */
#define	OPCODE_EN4B		0xb7	/* Enter 4-byte mode */
#define	OPCODE_EX4B		0xe9	/* Exit 4-byte mode */

/* Used for Spansion flashes only. */
#define	OPCODE_BRWR		0x17	/* Bank register write */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		5

#define JEDEC_MFR(_jedec_id)	((_jedec_id) >> 16)
#define ADDR_WIDTH  3
#define PAGE_SIZE   256

int SpiWriteAndRead(int spi_device, unsigned char *data, int length)
{
    struct spi_ioc_transfer spi[1];
    int retVal = -1;

    memset(&spi[0], 0, sizeof(spi[0]));
    spi[0].tx_buf = (unsigned long)data;
    spi[0].len = length;
    spi[0].rx_buf = (unsigned long)data;
    retVal = ioctl(spi_device, SPI_IOC_MESSAGE(1), &spi);

    if (retVal < 0)
        perror("Error - Problem transmitting spi data..ioctl");

    return retVal;
}

int spi_write_then_read(int spi_device, const void *txbuf, unsigned n_tx, void *rxbuf, unsigned n_rx)
{
    struct spi_ioc_transfer spi[2];
    int retVal = -1, count = 1;

    memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
    spi[0].tx_buf = (unsigned long)txbuf;
    spi[0].len = n_tx;

    if (rxbuf != NULL && n_rx > 0)
    {
        count++;
        memset(&spi[1], 0, sizeof(struct spi_ioc_transfer));
        spi[1].rx_buf = (unsigned long)rxbuf;
        spi[1].len = n_rx;
    }

    retVal = ioctl(spi_device, SPI_IOC_MESSAGE(count), &spi);

    if (retVal < 0)
        perror("Error - Problem transmitting spi data..ioctl");

    return retVal;
}

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(int spi_device)
{
	ssize_t retval;
	unsigned char code = OPCODE_RDSR;
	unsigned char val;

	retval = spi_write_then_read(spi_device, &code, 1, &val, 1);

	if (retval < 0)		
		return retval;	

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(int spi_device, unsigned char val)
{
    unsigned char command[2];
	command[0] = OPCODE_WRSR;
	command[1] = val;

	return spi_write_then_read(spi_device, command, 2 ,NULL, 0);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(int spi_device)
{
	unsigned char	code = OPCODE_WREN;

	return spi_write_then_read(spi_device, &code, 1, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(int spi_device)
{
	unsigned char	code = OPCODE_WRDI;

	return spi_write_then_read(spi_device, &code, 1, NULL, 0);
}

static int wait_till_ready(int spi_device)
{	
	int sr;
	int count =0;

	do {
		if ((sr = read_sr(spi_device)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;
        usleep(2);
	} while (count++ < 30000000);

	return 1;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(int spi_device)
{	
    int ret;
    unsigned char command[2];
	/* Wait until finished previous write command. */
	if (wait_till_ready(spi_device))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(spi_device);
    
	/* Set up command buffer. */
	command[0] = OPCODE_CHIP_ERASE;    
	ret = spi_write_then_read(spi_device, command, 1, NULL, 0);

	return 0;
}

static void m25p_addr2cmd(unsigned int addr, unsigned char *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (ADDR_WIDTH * 8 -  8);
	cmd[2] = addr >> (ADDR_WIDTH * 8 - 16);
	cmd[3] = addr >> (ADDR_WIDTH * 8 - 24);
	cmd[4] = 0;
}

static int m25p_cmdsz(void)
{
	return 1 + ADDR_WIDTH;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(int spi_device, unsigned int offset)
{	
    int ret;
    unsigned char command[8];
	/* Wait until finished previous write command. */
	if (wait_till_ready(spi_device))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(spi_device);

	/* Set up command buffer. */
	command[0] = OPCODE_SE;
	m25p_addr2cmd(offset, command);

	ret = spi_write_then_read(spi_device, command, m25p_cmdsz(), NULL, 0);

	return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25p80_read(int spi_device, loff_t from, size_t len, size_t *retlen, u_char *buf)
{		
	uint8_t opcode;
    uint8_t command[8];
	int ret;

	/* Wait till previous write/erase is done. */
	if (wait_till_ready(spi_device)) {
		/* REVISIT status return?? */		
		return 1;
	}
	
	/* Set up the write data buffer. */
	opcode = OPCODE_NORM_READ;
	command[0] = opcode;
	m25p_addr2cmd(from, command);
	
    ret = spi_write_then_read(spi_device, command, m25p_cmdsz(), buf, len);	
    *retlen = ret - m25p_cmdsz();

	return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int m25p80_write(int spi_device, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{    
	uint8_t opcode;
    uint8_t command[8];
	int ret;
	uint32_t page_offset, page_size;
	struct spi_ioc_transfer t[2];

    *retlen = 0;
    memset(t, 0, (sizeof t));
	t[0].tx_buf = (unsigned long)command;
	t[0].len = m25p_cmdsz();	
	t[1].tx_buf = (unsigned long)buf;
		
	/* Wait until finished previous write command. */
	if (wait_till_ready(spi_device)) {		
		return 1;
	}

	write_enable(spi_device);

	/* Set up the opcode in the write buffer. */
	command[0] = OPCODE_PP;
	m25p_addr2cmd(to, command);

	page_offset = to & (PAGE_SIZE - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= PAGE_SIZE) {
		t[1].len = len;

		ret = ioctl(spi_device, SPI_IOC_MESSAGE(2), &t);

		*retlen += ret - m25p_cmdsz();
	} else {
		uint32_t i;

		/* the size of data remaining on the first page */
		page_size = PAGE_SIZE - page_offset;

		t[1].len = page_size;		
		ret = ioctl(spi_device, SPI_IOC_MESSAGE(2), &t);
		*retlen += ret - m25p_cmdsz();

		/* write everything in PAGE_SIZE chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > PAGE_SIZE)
				page_size = PAGE_SIZE;

			/* write the next page to flash */
			m25p_addr2cmd(to + i, command);

			t[1].tx_buf = (unsigned long)(buf + i);
			t[1].len = page_size;

			wait_till_ready(spi_device);

			write_enable(spi_device);

			ret = ioctl(spi_device, SPI_IOC_MESSAGE(2), &t);
		    *retlen += ret - m25p_cmdsz();
		}
	}
    write_disable(spi_device);
	return 0;
}

void hexPrintf(char *title,unsigned char *a,int iLength)
{   
    int i = 0;
    printf("%s\n", title);
    for ( i = 0; i < iLength; i++)
    {
        printf("%02X ", a[i]);                        
        if ((i + 1) % 32 == 0)
                printf("\n");
    }
    if (i != 0)
    {
        printf("\n");
    }
}

int main(int argc, char **argv)
{
	FILE *fdRead = NULL;
	FILE *fdWrite = NULL;
    char *name;
    int fd, i;
    unsigned char wBuf[1024];
    unsigned char buf[1024];
    int len, status = 0, offset, readSize;
	struct stat statbuf;

    name = argv[1];
    fd = open(name, O_RDWR);
    if (fd < 0)
    {
        perror("open");
		status = 1;
        goto ExitFlash;
    }
	if(argc >=3)
	{
		stat(argv[2], &statbuf);
    	fdRead = fopen(argv[2],"rb");
	}
	if(fdRead==NULL)
	{
		perror("open file");
		status = 1;
		goto ExitFlash;
	}
	if(argc >=4)
		fdWrite = fopen(argv[3],"wb");

    buf[0] = OPCODE_RDID;
    len = 4;
    status = SpiWriteAndRead(fd, buf, len);	
    hexPrintf("OPCODE_RDID", buf, len);
	for(len=0, i=0; i<4; i++)
		len += buf[i];
	if(len == 0 || len == (0xFF*4))
	{
		status = 2;
        goto ExitFlash;
	}	
	for(i=0; i<statbuf.st_size; i+=65536)
	{
    	status = erase_sector(fd, i);
    	printf("erase_sector_%d :ret=%d \n", i/65536, status);
	}
	offset = 0;
	do{
		readSize = fread(wBuf, 1, sizeof(wBuf), fdRead);			
		if(readSize > 0)
		{
			status = m25p80_write(fd, offset, readSize, &len, wBuf);
    		//printf("m25p80_write :ret=%d len=%d\n", status, len);
			status = m25p80_read(fd, offset, readSize, &len, buf);
			//printf("m25p80_read :ret=%d len=%d\n", status, len);
			if(memcmp(wBuf, buf, readSize) != 0)
			{
        		printf("FLASH R/W TEST FAIL\n");
				status = 3;
			}
			if(fdWrite != NULL)
				fwrite(buf, 1, readSize, fdWrite);
		}
		offset += readSize;
	}while(readSize > 0);

ExitFlash:	
	if(status == 0)
		printf("Write Flash OK: size = %u\n",statbuf.st_size);
	else	
		printf("Write Flash Fail:%d\n",status);
    if(fd >=0)
		close(fd);
	if(fdRead != NULL)
		fclose(fdRead);
	if(fdWrite != NULL)
		fclose(fdWrite);	
	return status;
}
