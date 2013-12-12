
#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////
//TYPES ///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

#ifndef u32
typedef unsigned long u32;
#endif
#ifndef u16
typedef unsigned short u16;
#endif
#ifndef u8
typedef unsigned char u8;
#endif

#ifndef s32
typedef signed long s32;
#endif
#ifndef s16
typedef signed short s16;
#endif
#ifndef s8
typedef signed char s8;
#endif

#ifndef NULL
#define NULL 0
#endif


//C++ like type castings. ie U32(variable) = (u32)variable
#define U8(v) ((u8)(v))
#define U16(v) ((u16)(v))
#define U32(v) ((u32)(v))
#define S8(v) ((s8)(v))
#define S16(v) ((s16)(v))
#define S32(v) ((s32)(v))


/* Header of STM32 main app. This struct is used to read some constants from main app flash space.
 * These values are checked by bootloader before jumping to main app to ensure that wrong code wont be executed
 * This struct is always located in same mem address in main app FW so bootloader finds it */
typedef struct
{
	u32 Checksum; /*flash checksum, replaced by GD modified srec2bin with correct checksum */
	u32 BinaryLength; /*size of flashed FW in bytes */
	u32 TargetHardwareID; /*hardware id where this FW is intended */
	u32 TargetHardwareIDMin; /*The lowest HW version which is supported by this FW*/
	u32 TargetHardwareIDMax; /*The highest HW version that has been supported by this FW. Set this as the current HW version which is used on FW development*/
	u32 FWBackwardsCompVersion; /*The lowest FW version which backwards compatible to this version*/
	u32 FWVersion; /*current FW version*/
	u32 Reserved; /*for future use*/
} FWHeader;

#define BLHEADER_ADDRESS 0x08000188

/* These values are stored in bootloader code as constants in fixed location 0x08000184
 * main app may read these from the location to see underlying HW version etc. */
typedef struct {
	u32 Checksum; /*BL program area flash checksum, replaced by GD modified srec2bin with correct checksum */
	u32 BinaryLength; /*size of used BL area in bytes */
	u32 HardwareID; /*hardware id http://granitedevices.com/wiki/Hardware_version_ID */
	u32 BLFWversion; /*bootloader version*/
	u32 BLFWCompatVersion;/*bootloader version compatible with old version number*/
	u32 Reserved; /*for future use*/
} BLHeader;


/*
 * The struct is located just behind vector table at flash.
 *
 * The Struct start is 0x184 from beginning of program flash (+start offset, i.e. 0x08000184 for zero offset [or 0x08008184 for 32kb offset = WTF?])
 */
#if 0
extern const volatile BLHeader VSDR_BL_Header __attribute__((section(".firmware_consts"))) = {
		0xc7ecc7ec, /*bootloader checksum, TODO actually use this!*/
		0x0, /*bootloader length in bytes */
		DEVICE_TYPE_HW_ID,
		90,/*hw version*/
		1250,/*bl fw version*/
		1200,/*bl fw compat version*/
		0xFFFFFFFF/*reserved*/
};
#endif


//SM payload command structure
typedef struct {
	/* ID=0 param size 30 bits (cmd total 4 bytes)
	 * ID=1 param size 22 bits (cmd total 3 bytes)
	 * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
	 * ID=3 reserved
	 */
	u8 ID :2;
	s32 param :30;
	bool discardRetData; //if true, return data will not be inserted in ret queue
} __attribute__ ((packed)) SMPayloadCommandForQueue;


typedef struct {
	/* ID=0 param size 30 bits (cmd total 4 bytes)
	 * ID=1 param size 22 bits (cmd total 3 bytes)
	 * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
	 * ID=3 reserved
	 */
	s32 param :30; //LSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommand32;

typedef struct {
	/* ID=0 param size 30 bits (cmd total 4 bytes)
	 * ID=1 param size 22 bits (cmd total 3 bytes)
	 * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
	 * ID=3 reserved
	 */
	s32 param :14; //LSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommand16;

typedef struct {
	/* ID=0 param size 30 bits (cmd total 4 bytes)
	 * ID=1 param size 22 bits (cmd total 3 bytes)
	 * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
	 * ID=3 reserved
	 */
	s32 param :22; //MSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommand24;

//SM payload command return data structure
typedef struct {
	/* ID=0 ret data 30 bits (tot 4 bytes)
	 * ID=1 ret data 22 bits (tot 3 bytes)
	 * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
	 * ID=3 reserved
	 */
	s32 retData: 30; //LSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet32;
//SM payload command return data structure
typedef struct {
	/* ID=0 ret data 30 bits (tot 4 bytes)
	 * ID=1 ret data 22 bits (tot 3 bytes)
	 * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
	 * ID=3 reserved
	 */
	s32 retData: 22; //LSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet24;
//SM payload command return data structure
typedef struct {
	/* ID=0 ret data 30 bits (tot 4 bytes)
	 * ID=1 ret data 22 bits (tot 3 bytes)
	 * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
	 * ID=3 reserved
	 */
	s32 retData: 14; //LSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet16;
//SM payload command return data structure
typedef struct {
	/* ID=0 ret data 30 bits (tot 4 bytes)
	 * ID=1 ret data 22 bits (tot 3 bytes)
	 * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
	 * ID=3 reserved
	 */
	s32 retData: 6; //LSB 30 bits
	u8 ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet8;



#ifdef __cplusplus
}
#endif

#endif
