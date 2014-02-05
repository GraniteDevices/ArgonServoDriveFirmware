/*
	makefirmware - create firmware file for GD Argon STM32 & GC processor
	Copyright (C) 2012 Tero Kontkanen
	
	Code partially based on srec2bin Copyright (C) 2010 Anthony Goffart

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This app takes raw binary file for STM32 as input and optionally a GraniteCore firmware and writes them into a 
 * Granity loadable file
 * 
 * If GC file is not specified, then anly STM32 file is being uploaded to device by Granity and old GC file stays intact
 */

 /* Change log. 
 1.1.1 Fix typo Ganite->Granite 
 */
 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (~FALSE)
#endif

#include <sys/stat.h>

#define HEADER1 "makefirmware 1.1.1 - Create bootloadable STM32 binary for Granite Devices Argon\n"
#define HEADER2 "Copyright (c) 2012-2014 Tero Kontkanen\n\n"

int verbose = TRUE;


char *STM32infilename;
char *GCFWinfilename=NULL;
char *outfilename;

FILE *STMinfile,*GCinfile=NULL;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

u32 outFileChecksum=0;//simple checksum for output sfile

//offset from start of app binary
#define FW_HEADER_OFFSET 0x188

/* Header of STM32 main app. This struct is used to read some constants from main app flash space.
 * These values are checked by bootloader before jumping to main app to ensure that wrong code wont be executed */
typedef struct
{
	u32 Checksum; /*flash checksum, replaced by GD modified srec2bin with correct checksum */
	u32 BinaryLength; /*size of flashed FW in bytes */
	u32 TargetHardware; /*hardware id where this FW is intended */
	u32 TargetHardwareMinVer; /*The lowest HW version which is supported by this FW*/
	u32 TargetHardwareMaxVer; /*The highest HW version that has been supported by this FW. Set this as the current HW version which is used on FW development*/
	u32 Reserved; /*for future use*/
} FWHeader;

/***************************************************************************/
uint32_t file_size(FILE *f)
{
	struct stat info;

	if (!fstat(fileno(f), &info))
		return((uint32_t)info.st_size);
	else
		return(0);
}

int writeU8( FILE *f, u8 w )
{
	fwrite((char*)&w,1,1,f);
	outFileChecksum+=w;//simple checksum for output sfile
}

int writeU16( FILE *f, u16 w )
{
	writeU8(f,w);
	writeU8(f,w>>8);
}

int writeU32( FILE *f, u32 w )
{
	writeU8(f,w);
	writeU8(f,w>>8);
	writeU8(f,w>>16);
	writeU8(f,w>>24);
}

//load file to memory. args:
//pass pointer to "data". this func will allocate the memory
//return number of 8bit bytes or -1 if load failed
//rounds upwards to 4 byte boundary, padded at end with 1-3 zero bytes if file length not even
int load_file( FILE *f, u8 **data, int roundupwards )
{
	int bytes, bytesout;
	int i;
	u8 *buf;
	
	//find file size
	bytes = file_size(f);
	
	//round upwards to 32bit boundary for stm32 file
	if(roundupwards)
		bytesout=((bytes+3)/4)*4;
	else
		bytesout=bytes;

	if(verbose)
		printf("Input size=%d bytes, output=%d bytes\n",bytes,bytesout);

	//allocate
	buf=(u8*)malloc(bytesout);
	if(buf==NULL)
		return -1;
	
	//reset buffer
	for(i=0;i<bytesout;i++)
		buf[i]=0;
		
	//load file
	for(i=0;i<bytes;i++)
	{
		int get=fgetc(f);
		if(get==EOF) 
			return -1;
		
		//treat data as byte array
		buf[i]=get;
	}
	
	*data=buf;
	
	return (bytesout);
}

void write_file(FILE *outfile, int len, u8 *buf)
{
	//append actual binary files
	int i;
	for(i=0;i<len;i++)
	{
		writeU8(outfile,buf[i]);
	}
}

//from wikipedia
const int MOD_ADLER = 65521;
uint64_t adler32(unsigned char *data, int32_t len, u32 skipaddr ) /* where data is the location of the data in physical memory and
                                                       len is the length of the data in bytes */
{
    uint64_t a = 1, b = 0;
    int32_t index;
    u8 in;

    /* Process each byte of the data in order */
    for (index = 0; index < len; ++index)
    {
		if(index/4==skipaddr/4)//skip checkum store position (write zeros to 4 bytes)
			in=0;
		else
			in=data[index];

        a = (a + in) % MOD_ADLER;
        b = (b + a) % MOD_ADLER;
    }

    return (b << 16) | a;
}

/***************************************************************************/
void embed_checksum( u8* data, int len )
{
	FWHeader *header;
	u32 cksum=0,i,readaddr,readword;
	header=(FWHeader*)((int)data+FW_HEADER_OFFSET);

	header->BinaryLength=len;
	
	//header->Checksum=(u32)adler32(data,len,(u32)&header->Checksum);
	header->Checksum=(u32)adler32(data,len,FW_HEADER_OFFSET);//addr 0x188 is skipped 32bit dword from input data as it will be checsum store position
}

int verify_GC_firmware( u8 *data, int len)
{
	//just check version & target device type
	u16 filever=data[4]+(data[5]<<8);
	u16 targetdev=data[6]+(data[7]<<8);
	if(filever!=199 || targetdev<4000 || targetdev>5000)
	{
		printf("Read %d %d",filever,targetdev);
		return 0;//wrong file error
	}
	return 1;//ok
}

/***************************************************************************/

void process(void)
{
	FILE *outfile;
	u8 *fwdata;
	u8 *GCfwdata;
	int bytes, GCbytes=-1;

	if(verbose) printf("Create file '%s'\n", outfilename);

	if ((outfile = fopen(outfilename, "wb")) == NULL)
	{
		fprintf(stderr, "Can't open output file '%s'\n", outfilename);
		return;
	}
	else
	{
		FWHeader *header;
		//write_file(outfile, f);
		bytes=load_file(STMinfile,&fwdata,1);
		
			
		
		//read stm part
		if(bytes<FW_HEADER_OFFSET+sizeof(FWHeader))
		{
			fprintf(stderr,"File too small to be valid\n");
			return;
		}
		//embed cksum to stm32 fw
		embed_checksum(fwdata,bytes);

		//read gc part
		if(GCinfile!=NULL)
		{
			GCbytes=load_file(GCinfile,&GCfwdata,0);
			if(verify_GC_firmware(GCfwdata,GCbytes)==0)
			{
				fprintf(stderr,"GC firmware file not correct for this device\n");
				return;
			}
		}
		
		//write header
		writeU32(outfile,'WFDG');//writes "GDFW"
		writeU16(outfile,300); //BL file version
		writeU16(outfile,4000); //Target drive type
		writeU32(outfile,bytes); //num of bytes of STM32 FW
		writeU32(outfile,GCbytes); //num of bytes of GC FW		
		
		//write stm32 part
		write_file(outfile,bytes,fwdata);

		//write gc part
		if(GCinfile!=NULL)
		{
			write_file(outfile,GCbytes,GCfwdata);
		}
		
		writeU32(outfile,outFileChecksum);
		
		fclose(outfile);
	}
}

/***************************************************************************/

void syntax(void)
{
	fprintf(stderr, HEADER1);
	fprintf(stderr, HEADER2);
	fprintf(stderr, "Syntax: makefirmware <options> OUTFILE STM32BINARY <GCBINARY> - GCBINARY is optional\n\n");
	fprintf(stderr, "-help            Show this help.\n");
	fprintf(stderr, "-q               Quiet mode\n");
}

/***************************************************************************/

int main(int argc, char *argv[])
{
	int i;

	for (i = 1; i < argc; i++)
	{	if (!strcmp(argv[i], "-q"))
		{
			verbose = FALSE;
			continue;
		}
		else if (!strncmp(argv[i], "-h", 2))			/* -h or -help */
		{
			syntax();
			return(0);
		}
		else
		{
			outfilename = argv[i];
			STM32infilename = argv[++i];
			if(argc>i+1)//if there is also GCFW file specified
			{
				GCFWinfilename=argv[++i];
			}
		}
	}

	if (STM32infilename == NULL)
	{
		syntax();
		fprintf(stderr, "\n** No input filename specified\n");
		return(1);
	}

	if (outfilename == NULL)
	{
		syntax();
		fprintf(stderr, "\n** No output filename specified\n");
		return(1);
	}

	if(GCFWinfilename!=NULL)
		if ((GCinfile = fopen(GCFWinfilename, "rb")) == NULL)
		{
			printf("Can't open file %s\n", GCFWinfilename);
			return(5);
		}
		
	if ((STMinfile = fopen(STM32infilename, "rb")) != NULL)
	{
		process();
		fclose(STMinfile);
		return(0);
	}
	else
	{
		printf("Can't open file %s\n", STM32infilename);
		return(2);
	}
}

/***************************************************************************/
