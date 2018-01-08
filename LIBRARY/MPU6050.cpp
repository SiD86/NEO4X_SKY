/* ================================================================================================ *
| Default FIFO packet structure:                                           |                                                                                              |
| [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ]  |
|   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23   |
|                                                                                                   |
| [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                          |
|  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                           |
* ================================================================================================ */
/* ================================================================================================ *
| Current FIFO packet structure:                                                                    |                                                                                               |
| [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][      ]                          |
|   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17                           |
* ================================================================================================ */
#include <Arduino.h>
#include "I2C.h"
#include "MPU6050.h"
#define MPU6050_FIFO_PACKET_SIZE	18							// ������ ������ FIFO
#define MPU6050_ADDRESS_AD0_LOW     0x68						// �����, ���� �� ����� ������ �������
#define MPU6050_ADDRESS_AD0_HIGH    0x69						// �����, ���� �� ����� ������� �������
#define MPU6050_FREQUENCY_DIV		0x01						// �������� ������� 200Hz / (1 + DIV) 
#define MPU6050_DATA_READY_TIMEOUT	100							// 100 ms

#define ADDRESS						MPU6050_ADDRESS_AD0_LOW		// ����� �� ���������

#pragma region MPU6050_DMP_BANKS
static const uint8_t DMP_MEMORY_BINARY[] PROGMEM = {
	// bank 0, 256 bytes
	0xFB,0x00,0x00,0x3E,0x00,0x0B,0x00,0x36,0x00,0x01,0x00,0x02,0x00,0x03,0x00,0x00,
	0x00,0x65,0x00,0x54,0xFF,0xEF,0x00,0x00,0xFA,0x80,0x00,0x0B,0x12,0x82,0x00,0x01,
	0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x28,0x00,0x00,0xFF,0xFF,0x45,0x81,0xFF,0xFF,0xFA,0x72,0x00,0x00,0x00,0x00,
	0x00,0x00,0x03,0xE8,0x00,0x00,0x00,0x01,0x00,0x01,0x7F,0xFF,0xFF,0xFE,0x80,0x01,
	0x00,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x3E,0x03,0x30,0x40,0x00,0x00,0x00,0x02,0xCA,0xE3,0x09,0x3E,0x80,0x00,0x00,
	0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x60,0x00,0x00,0x00,
	0x41,0xFF,0x00,0x00,0x00,0x00,0x0B,0x2A,0x00,0x00,0x16,0x55,0x00,0x00,0x21,0x82,
	0xFD,0x87,0x26,0x50,0xFD,0x80,0x00,0x00,0x00,0x1F,0x00,0x00,0x00,0x05,0x80,0x00,
	0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x03,0x00,0x00,
	0x40,0x00,0x00,0x00,0x00,0x00,0x04,0x6F,0x00,0x02,0x65,0x32,0x00,0x00,0x5E,0xC0,
	0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xFB,0x8C,0x6F,0x5D,0xFD,0x5D,0x08,0xD9,0x00,0x7C,0x73,0x3B,0x00,0x6C,0x12,0xCC,
	0x32,0x00,0x13,0x9D,0x32,0x00,0xD0,0xD6,0x32,0x00,0x08,0x00,0x40,0x00,0x01,0xF4,
	0xFF,0xE6,0x80,0x79,0x02,0x00,0x00,0x00,0x00,0x00,0xD0,0xD6,0x00,0x00,0x27,0x10,
	// bank 1, 256 bytes
	0xFB,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,
	0x00,0x00,0xFA,0x36,0xFF,0xBC,0x30,0x8E,0x00,0x05,0xFB,0xF0,0xFF,0xD9,0x5B,0xC8,
	0xFF,0xD0,0x9A,0xBE,0x00,0x00,0x10,0xA9,0xFF,0xF4,0x1E,0xB2,0x00,0xCE,0xBB,0xF7,
	0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x04,0x00,0x02,0x00,0x02,0x02,0x00,0x00,0x0C,
	0xFF,0xC2,0x80,0x00,0x00,0x01,0x80,0x00,0x00,0xCF,0x80,0x00,0x40,0x00,0x00,0x00,
	0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x14,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x03,0x3F,0x68,0xB6,0x79,0x35,0x28,0xBC,0xC6,0x7E,0xD1,0x6C,
	0x80,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0xB2,0x6A,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,0x30,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x25,0x4D,0x00,0x2F,0x70,0x6D,0x00,0x00,0x05,0xAE,0x00,0x0C,0x02,0xD0,
	// bank 2, 256 bytes
	0x00,0x00,0x00,0x00,0x00,0x65,0x00,0x54,0xFF,0xEF,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x01,0x00,0x00,0x44,0x00,0x00,0x00,0x00,0x0C,0x00,0x00,0x00,0x01,0x00,
	0x00,0x00,0x00,0x00,0x00,0x65,0x00,0x00,0x00,0x54,0x00,0x00,0xFF,0xEF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
	0x00,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	// bank 3, 256 bytes
	0xD8,0xDC,0xBA,0xA2,0xF1,0xDE,0xB2,0xB8,0xB4,0xA8,0x81,0x91,0xF7,0x4A,0x90,0x7F,
	0x91,0x6A,0xF3,0xF9,0xDB,0xA8,0xF9,0xB0,0xBA,0xA0,0x80,0xF2,0xCE,0x81,0xF3,0xC2,
	0xF1,0xC1,0xF2,0xC3,0xF3,0xCC,0xA2,0xB2,0x80,0xF1,0xC6,0xD8,0x80,0xBA,0xA7,0xDF,
	0xDF,0xDF,0xF2,0xA7,0xC3,0xCB,0xC5,0xB6,0xF0,0x87,0xA2,0x94,0x24,0x48,0x70,0x3C,
	0x95,0x40,0x68,0x34,0x58,0x9B,0x78,0xA2,0xF1,0x83,0x92,0x2D,0x55,0x7D,0xD8,0xB1,
	0xB4,0xB8,0xA1,0xD0,0x91,0x80,0xF2,0x70,0xF3,0x70,0xF2,0x7C,0x80,0xA8,0xF1,0x01,
	0xB0,0x98,0x87,0xD9,0x43,0xD8,0x86,0xC9,0x88,0xBA,0xA1,0xF2,0x0E,0xB8,0x97,0x80,
	0xF1,0xA9,0xDF,0xDF,0xDF,0xAA,0xDF,0xDF,0xDF,0xF2,0xAA,0xC5,0xCD,0xC7,0xA9,0x0C,
	0xC9,0x2C,0x97,0x97,0x97,0x97,0xF1,0xA9,0x89,0x26,0x46,0x66,0xB0,0xB4,0xBA,0x80,
	0xAC,0xDE,0xF2,0xCA,0xF1,0xB2,0x8C,0x02,0xA9,0xB6,0x98,0x00,0x89,0x0E,0x16,0x1E,
	0xB8,0xA9,0xB4,0x99,0x2C,0x54,0x7C,0xB0,0x8A,0xA8,0x96,0x36,0x56,0x76,0xF1,0xB9,
	0xAF,0xB4,0xB0,0x83,0xC0,0xB8,0xA8,0x97,0x11,0xB1,0x8F,0x98,0xB9,0xAF,0xF0,0x24,
	0x08,0x44,0x10,0x64,0x18,0xF1,0xA3,0x29,0x55,0x7D,0xAF,0x83,0xB5,0x93,0xAF,0xF0,
	0x00,0x28,0x50,0xF1,0xA3,0x86,0x9F,0x61,0xA6,0xDA,0xDE,0xDF,0xD9,0xFA,0xA3,0x86,
	0x96,0xDB,0x31,0xA6,0xD9,0xF8,0xDF,0xBA,0xA6,0x8F,0xC2,0xC5,0xC7,0xB2,0x8C,0xC1,
	0xB8,0xA2,0xDF,0xDF,0xDF,0xA3,0xDF,0xDF,0xDF,0xD8,0xD8,0xF1,0xB8,0xA8,0xB2,0x86,
	// bank 4, 256 bytes
	0xB4,0x98,0x0D,0x35,0x5D,0xB8,0xAA,0x98,0xB0,0x87,0x2D,0x35,0x3D,0xB2,0xB6,0xBA,
	0xAF,0x8C,0x96,0x19,0x8F,0x9F,0xA7,0x0E,0x16,0x1E,0xB4,0x9A,0xB8,0xAA,0x87,0x2C,
	0x54,0x7C,0xB9,0xA3,0xDE,0xDF,0xDF,0xA3,0xB1,0x80,0xF2,0xC4,0xCD,0xC9,0xF1,0xB8,
	0xA9,0xB4,0x99,0x83,0x0D,0x35,0x5D,0x89,0xB9,0xA3,0x2D,0x55,0x7D,0xB5,0x93,0xA3,
	0x0E,0x16,0x1E,0xA9,0x2C,0x54,0x7C,0xB8,0xB4,0xB0,0xF1,0x97,0x83,0xA8,0x11,0x84,
	0xA5,0x09,0x98,0xA3,0x83,0xF0,0xDA,0x24,0x08,0x44,0x10,0x64,0x18,0xD8,0xF1,0xA5,
	0x29,0x55,0x7D,0xA5,0x85,0x95,0x02,0x1A,0x2E,0x3A,0x56,0x5A,0x40,0x48,0xF9,0xF3,
	0xA3,0xD9,0xF8,0xF0,0x98,0x83,0x24,0x08,0x44,0x10,0x64,0x18,0x97,0x82,0xA8,0xF1,
	0x11,0xF0,0x98,0xA2,0x24,0x08,0x44,0x10,0x64,0x18,0xDA,0xF3,0xDE,0xD8,0x83,0xA5,
	0x94,0x01,0xD9,0xA3,0x02,0xF1,0xA2,0xC3,0xC5,0xC7,0xD8,0xF1,0x84,0x92,0xA2,0x4D,
	0xDA,0x2A,0xD8,0x48,0x69,0xD9,0x2A,0xD8,0x68,0x55,0xDA,0x32,0xD8,0x50,0x71,0xD9,
	0x32,0xD8,0x70,0x5D,0xDA,0x3A,0xD8,0x58,0x79,0xD9,0x3A,0xD8,0x78,0x93,0xA3,0x4D,
	0xDA,0x2A,0xD8,0x48,0x69,0xD9,0x2A,0xD8,0x68,0x55,0xDA,0x32,0xD8,0x50,0x71,0xD9,
	0x32,0xD8,0x70,0x5D,0xDA,0x3A,0xD8,0x58,0x79,0xD9,0x3A,0xD8,0x78,0xA8,0x8A,0x9A,
	0xF0,0x28,0x50,0x78,0x9E,0xF3,0x88,0x18,0xF1,0x9F,0x1D,0x98,0xA8,0xD9,0x08,0xD8,
	0xC8,0x9F,0x12,0x9E,0xF3,0x15,0xA8,0xDA,0x12,0x10,0xD8,0xF1,0xAF,0xC8,0x97,0x87,
	// bank 5, 256 bytes
	0x34,0xB5,0xB9,0x94,0xA4,0x21,0xF3,0xD9,0x22,0xD8,0xF2,0x2D,0xF3,0xD9,0x2A,0xD8,
	0xF2,0x35,0xF3,0xD9,0x32,0xD8,0x81,0xA4,0x60,0x60,0x61,0xD9,0x61,0xD8,0x6C,0x68,
	0x69,0xD9,0x69,0xD8,0x74,0x70,0x71,0xD9,0x71,0xD8,0xB1,0xA3,0x84,0x19,0x3D,0x5D,
	0xA3,0x83,0x1A,0x3E,0x5E,0x93,0x10,0x30,0x81,0x10,0x11,0xB8,0xB0,0xAF,0x8F,0x94,
	0xF2,0xDA,0x3E,0xD8,0xB4,0x9A,0xA8,0x87,0x29,0xDA,0xF8,0xD8,0x87,0x9A,0x35,0xDA,
	0xF8,0xD8,0x87,0x9A,0x3D,0xDA,0xF8,0xD8,0xB1,0xB9,0xA4,0x98,0x85,0x02,0x2E,0x56,
	0xA5,0x81,0x00,0x0C,0x14,0xA3,0x97,0xB0,0x8A,0xF1,0x2D,0xD9,0x28,0xD8,0x4D,0xD9,
	0x48,0xD8,0x6D,0xD9,0x68,0xD8,0xB1,0x84,0x0D,0xDA,0x0E,0xD8,0xA3,0x29,0x83,0xDA,
	0x2C,0x0E,0xD8,0xA3,0x84,0x49,0x83,0xDA,0x2C,0x4C,0x0E,0xD8,0xB8,0xB0,0xA8,0x8A,
	0x9A,0xF5,0x20,0xAA,0xDA,0xDF,0xD8,0xA8,0x40,0xAA,0xD0,0xDA,0xDE,0xD8,0xA8,0x60,
	0xAA,0xDA,0xD0,0xDF,0xD8,0xF1,0x97,0x86,0xA8,0x31,0x9B,0x06,0x99,0x07,0xAB,0x97,
	0x28,0x88,0x9B,0xF0,0x0C,0x20,0x14,0x40,0xB8,0xB0,0xB4,0xA8,0x8C,0x9C,0xF0,0x04,
	0x28,0x51,0x79,0x1D,0x30,0x14,0x38,0xB2,0x82,0xAB,0xD0,0x98,0x2C,0x50,0x50,0x78,
	0x78,0x9B,0xF1,0x1A,0xB0,0xF0,0x8A,0x9C,0xA8,0x29,0x51,0x79,0x8B,0x29,0x51,0x79,
	0x8A,0x24,0x70,0x59,0x8B,0x20,0x58,0x71,0x8A,0x44,0x69,0x38,0x8B,0x39,0x40,0x68,
	0x8A,0x64,0x48,0x31,0x8B,0x30,0x49,0x60,0xA5,0x88,0x20,0x09,0x71,0x58,0x44,0x68,
	// bank 6, 256 bytes
	0x11,0x39,0x64,0x49,0x30,0x19,0xF1,0xAC,0x00,0x2C,0x54,0x7C,0xF0,0x8C,0xA8,0x04,
	0x28,0x50,0x78,0xF1,0x88,0x97,0x26,0xA8,0x59,0x98,0xAC,0x8C,0x02,0x26,0x46,0x66,
	0xF0,0x89,0x9C,0xA8,0x29,0x51,0x79,0x24,0x70,0x59,0x44,0x69,0x38,0x64,0x48,0x31,
	0xA9,0x88,0x09,0x20,0x59,0x70,0xAB,0x11,0x38,0x40,0x69,0xA8,0x19,0x31,0x48,0x60,
	0x8C,0xA8,0x3C,0x41,0x5C,0x20,0x7C,0x00,0xF1,0x87,0x98,0x19,0x86,0xA8,0x6E,0x76,
	0x7E,0xA9,0x99,0x88,0x2D,0x55,0x7D,0x9E,0xB9,0xA3,0x8A,0x22,0x8A,0x6E,0x8A,0x56,
	0x8A,0x5E,0x9F,0xB1,0x83,0x06,0x26,0x46,0x66,0x0E,0x2E,0x4E,0x6E,0x9D,0xB8,0xAD,
	0x00,0x2C,0x54,0x7C,0xF2,0xB1,0x8C,0xB4,0x99,0xB9,0xA3,0x2D,0x55,0x7D,0x81,0x91,
	0xAC,0x38,0xAD,0x3A,0xB5,0x83,0x91,0xAC,0x2D,0xD9,0x28,0xD8,0x4D,0xD9,0x48,0xD8,
	0x6D,0xD9,0x68,0xD8,0x8C,0x9D,0xAE,0x29,0xD9,0x04,0xAE,0xD8,0x51,0xD9,0x04,0xAE,
	0xD8,0x79,0xD9,0x04,0xD8,0x81,0xF3,0x9D,0xAD,0x00,0x8D,0xAE,0x19,0x81,0xAD,0xD9,
	0x01,0xD8,0xF2,0xAE,0xDA,0x26,0xD8,0x8E,0x91,0x29,0x83,0xA7,0xD9,0xAD,0xAD,0xAD,
	0xAD,0xF3,0x2A,0xD8,0xD8,0xF1,0xB0,0xAC,0x89,0x91,0x3E,0x5E,0x76,0xF3,0xAC,0x2E,
	0x2E,0xF1,0xB1,0x8C,0x5A,0x9C,0xAC,0x2C,0x28,0x28,0x28,0x9C,0xAC,0x30,0x18,0xA8,
	0x98,0x81,0x28,0x34,0x3C,0x97,0x24,0xA7,0x28,0x34,0x3C,0x9C,0x24,0xF2,0xB0,0x89,
	0xAC,0x91,0x2C,0x4C,0x6C,0x8A,0x9B,0x2D,0xD9,0xD8,0xD8,0x51,0xD9,0xD8,0xD8,0x79,
	// bank 7, 138 bytes (remainder)
	0xD9,0xD8,0xD8,0xF1,0x9E,0x88,0xA3,0x31,0xDA,0xD8,0xD8,0x91,0x2D,0xD9,0x28,0xD8,
	0x4D,0xD9,0x48,0xD8,0x6D,0xD9,0x68,0xD8,0xB1,0x83,0x93,0x35,0x3D,0x80,0x25,0xDA,
	0xD8,0xD8,0x85,0x69,0xDA,0xD8,0xD8,0xB4,0x93,0x81,0xA3,0x28,0x34,0x3C,0xF3,0xAB,
	0x8B,0xF8,0xA3,0x91,0xB6,0x09,0xB4,0xD9,0xAB,0xDE,0xFA,0xB0,0x87,0x9C,0xB9,0xA3,
	0xDD,0xF1,0xA3,0xA3,0xA3,0xA3,0x95,0xF1,0xA3,0xA3,0xA3,0x9D,0xF1,0xA3,0xA3,0xA3,
	0xA3,0xF2,0xA3,0xB4,0x90,0x80,0xF2,0xA3,0xA3,0xA3,0xA3,0xA3,0xA3,0xA3,0xA3,0xA3,
	0xA3,0xB2,0xA3,0xA3,0xA3,0xA3,0xA3,0xA3,0xB0,0x87,0xB5,0x99,0xF1,0xA3,0xA3,0xA3,
	0x98,0xF1,0xA3,0xA3,0xA3,0xA3,0x97,0xA3,0xA3,0xA3,0xA3,0xF3,0x9B,0xA3,0xA3,0xDC,
	0xB9,0xA7,0xF1,0x26,0x26,0x26,0xD8,0xD8,0xFF
};
#pragma endregion 

#pragma region MPU6050_DMP_CONFIG
static const uint8_t DMP_CONFIG_BINARY[] PROGMEM = {
	// BANK OFFSET LENGTH [DATA]
	0x03,0x7B,0x03,0x4C,0xCD,0x6C,         // FCFG_1 inv_set_gyro_calibration
	0x03,0xAB,0x03,0x36,0x56,0x76,         // FCFG_3 inv_set_gyro_calibration
	0x00,0x68,0x04,0x02,0xCB,0x47,0xA2,   // D_0_104 inv_set_gyro_calibration
	0x02,0x18,0x04,0x00,0x05,0x8B,0xC1,   // D_0_24 inv_set_gyro_calibration
	0x01,0x0C,0x04,0x00,0x00,0x00,0x00,   // D_1_152 inv_set_accel_calibration
	0x03,0x7F,0x06,0x0C,0xC9,0x2C,0x97,0x97,0x97, // FCFG_2 inv_set_accel_calibration
	0x03,0x89,0x03,0x26,0x46,0x66,         // FCFG_7 inv_set_accel_calibration
	0x00,0x6C,0x02,0x20,0x00,               // D_0_108 inv_set_accel_calibration
	0x02,0x40,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_00 inv_set_compass_calibration
	0x02,0x44,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_01
	0x02,0x48,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_02
	0x02,0x4C,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_10
	0x02,0x50,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_11
	0x02,0x54,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_12
	0x02,0x58,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_20
	0x02,0x5C,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_21
	0x02,0xBC,0x04,0x00,0x00,0x00,0x00,   // CPASS_MTX_22
	0x01,0xEC,0x04,0x00,0x00,0x40,0x00,   // D_1_236 inv_apply_endian_accel
	0x03,0x7F,0x06,0x0C,0xC9,0x2C,0x97,0x97,0x97, // FCFG_2 inv_set_mpu_sensors
	0x04,0x02,0x03,0x0D,0x35,0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
	0x04,0x09,0x04,0x87,0x2D,0x35,0x3D,   // FCFG_5 inv_set_bias_update
	0x00,0xA3,0x01,0x00,                     // D_0_163 inv_set_dead_zone
	// SPECIAL 0x01 = enable interrupts
	0x00,0x00,0x00,0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
	0x07,0x86,0x01,0xFE,                     // CFG_6 inv_set_fifo_interupt
	0x07,0x41,0x05,0xF1,0x20,0x28,0x30,0x38, // CFG_8 inv_send_quaternion
	0x07,0x7E,0x01,0x30,                     // CFG_16 inv_set_footer
	//0x07,0x46,0x01,0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
	//0x07,0x47,0x04,0xF1,0x28,0x30,0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
	//0x07,0x6C,0x04,0xF1,0x28,0x30,0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
	0x02,0x16,0x02,0x00,MPU6050_FREQUENCY_DIV // (0x07 -> 16Mhz) D_0_22 inv_set_fifo_rate (0x06 for first 8mhz board) (0x09 for 8Mhz board from Binoy)

	// This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
	// 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
	// DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

	// It is important to make sure the host processor can keep up with reading and processing
	// the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};
#pragma endregion

#pragma region MPU6050_DMP_UPDATE
static const uint8_t DMP_UPDATE_BINARY[] PROGMEM = {
	0x01,0xB2,0x02,0xFF,0xFF,
	0x01,0x90,0x04,0x09,0x23,0xA1,0x35,
	0x01,0x6A,0x02,0x06,0x00,
	0x01,0x60,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x60,0x04,0x40,0x00,0x00,0x00,
	0x01,0x62,0x02,0x00,0x00,
	0x00,0x60,0x04,0x00,0x40,0x00,0x00
};
#pragma endregion

#pragma region MPU6050_REGISTERS
#define REG_SMPLRT_DIV				0x19	// ������� �������� ������� ������ ���������
#define REG_CONFIG					0x1A
#define REG_GYRO_CONFIG				0x1B	// ������� ������������ ���������
#define REG_ACCEL_CONFIG			0x1C	// ������� ������������ �������������
#define REG_INT_CONFIG				0x37
#define REG_INT_ENABLE				0x38	// ������� ����������
#define REG_USER_CTRL				0x6A	// ������� ����������
#define REG_PWR_MGMT_1				0x6B	// ������� ���������� ��������
#define REG_BANK_SEL				0x6D	
#define REG_MEM_START_ADDR			0x6E
#define REG_MEM_R_W					0x6F
#define REG_DMP_CFG_1				0x70	// ������� 1 ������������ DMP
#define REG_DMP_CFG_2				0x71	// ������� 2 ������������ DMP
#define REG_FIFO_COUNTH				0x72	// ������� ��������� ���� � ������
#define REG_FIFO_R_W				0x74
#define REG_WHO_AM_I				0x75	// ������� �������������� ����������
///////////////////////////////////////////////////////////
// ��������� ������� ������ ������
#define CFG_DLPF_CFG_MASK			0x07
#define DLPF_BW_256					0x00
#define DLPF_BW_188					0x01
#define DLPF_BW_98					0x02
#define DLPF_BW_42					0x03
#define DLPF_BW_20					0x04
#define DLPF_BW_10					0x05
#define DLPF_BW_5					0x06
//
///////////////////////////////////////////////////////////
// ��������� ��������� �������� ��������� �������
#define PWR1_CLKSEL_MASK			0x07
#define CLOCK_INTERNAL				0x00
#define CLOCK_PLL_XGYRO				0x01
#define CLOCK_PLL_YGYRO				0x02
#define CLOCK_PLL_ZGYRO				0x03
#define CLOCK_PLL_EXT32K			0x04
#define CLOCK_PLL_EXT19M			0x05
//
///////////////////////////////////////////////////////////
// ��������� ��������� �������������
#define ACONFIG_AFS_SEL_MASK		0x18
#define ACCEL_AFS_2					0x00
#define ACCEL_AFS_4					0x08
#define ACCEL_AFS_8					0x10
#define ACCEL_AFS_16				0x18
//
///////////////////////////////////////////////////////////
// ��������� ��������� ���������
#define GCONFIG_FS_SEL_MASK			0x18
#define GYRO_FS_250					0x00
#define GYRO_FS_500					0x08
#define GYRO_FS_1000				0x10
#define GYRO_FS_2000				0x18
//
///////////////////////////////////////////////////////////
#define USERCTRL_DMP_EN_MASK		0x80
#define USERCTRL_DMP_EN				0x80
#define USERCTRL_DMP_DIS			0x00

#define USERCTRL_FIFO_EN_MASK		0x40
#define USERCTRL_FIFO_EN			0x40
#define USERCTRL_FIFO_DIS			0x00

#define USERCTRL_FIFO_RESET_MASK	0x04
#define USERCTRL_FIFO_RESET			0x04

#define PWR1_DEVICE_RESET_MASK		0x80
#define PWR1_DEVICE_RESET			0x80

#define PWR1_SLEEP_MASK				0x40
#define PWR1_SLEEP_EN				0x40
#define PWR1_SLEEP_DIS				0x00

#pragma endregion

static void data_ready_IRQ_callback();
static void calculation_XYZ(uint8_t* data, float* X, float* Y, float* Z);
static bool writeMemoryBlock(const uint8_t* pData, int16_t DataSize, uint8_t Bank, uint8_t Addr, bool IsUseProgMem);
static bool writeDMPConfig(const uint8_t* pData, uint16_t DataSize);

static uint32_t g_status = MPU6050_DRIVER_NO_ERROR;
static volatile bool g_is_data_ready = false;
static volatile bool g_is_data_ready_timeout = false;

uint8_t MPU6050_get_FIFO_size_error_count = 0;
uint8_t MPU6050_check_FIFO_size_error_count = 0;
uint8_t MPU6050_get_data_error_count = 0;


//
// EXTERNAL INTERFACE
//
void MPU6050_initialize(uint32_t data_ready_IRQ_pin) {
	
	I2C_set_internal_address_length(1);
	g_status = MPU6050_DRIVER_ERROR;

	// Configure watch dara ready timeout timer clock
	REG_PMC_PCER0 = PMC_PCER0_PID31;
	REG_TC1_CMR1 = TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_CPCDIS;
	REG_TC1_RC1 = MPU6050_DATA_READY_TIMEOUT * (VARIANT_MCK / 8 / 1000);
	NVIC_EnableIRQ(TC4_IRQn);

	// �������� ����������� ����������
    uint8_t reg = 0;
	if (!I2C_read_byte(ADDRESS, REG_WHO_AM_I, &reg))
		return;
	
	if ( (reg >> 1) != 0x34 ) // FIXME
        return;

	// ������������ ����������
	if (!I2C_write_bits(ADDRESS, REG_PWR_MGMT_1, PWR1_DEVICE_RESET_MASK, PWR1_DEVICE_RESET))
		return;
    delay(30);

    // ���������� ������ ��������
	if (!I2C_write_bits(ADDRESS, REG_PWR_MGMT_1, PWR1_SLEEP_MASK, PWR1_SLEEP_DIS))
        return;

	// �������� DMP (�� ���������������)
	if (!writeMemoryBlock(DMP_MEMORY_BINARY, sizeof(DMP_MEMORY_BINARY), 0, 0, true))
        return;

	// �������� ������������ � DMP (�� ���������������)
	if (!writeDMPConfig(DMP_CONFIG_BINARY, sizeof(DMP_CONFIG_BINARY)))
        return;

	// ��������� ��������� �������� ��������� �������
	if (!I2C_write_bits(ADDRESS, REG_PWR_MGMT_1, PWR1_CLKSEL_MASK, CLOCK_PLL_XGYRO))
        return;

	// ��������� ����������� ������� ������ ������
	if (!I2C_write_bits(ADDRESS, REG_CONFIG, CFG_DLPF_CFG_MASK, DLPF_BW_42))
        return;

	// ��������� ��������� ��������� ������������� +/- 2g
	if (!I2C_write_bits(ADDRESS, REG_ACCEL_CONFIG, ACONFIG_AFS_SEL_MASK, ACCEL_AFS_2))
        return;

	// ��������� ��������� ��������� ��������� +/- 2000
	if (!I2C_write_bits(ADDRESS, REG_GYRO_CONFIG, GCONFIG_FS_SEL_MASK, GYRO_FS_2000))
        return;

	// ��������� ������������ DMP (�� ���������������)
	if (!I2C_write_byte(ADDRESS, REG_DMP_CFG_1, 0x03))
        return;
	if (!I2C_write_byte(ADDRESS, REG_DMP_CFG_2, 0x00))
        return;

	// �������� ���������� � DMP (�� ���������������)
	uint8_t Buffer[48] = {0};
	for (int i = 0,pos = 0; i < 7; ++i)
	{
		for (int j = 0; j < 4 || j < Buffer[2] + 3; ++j,++pos) 
			Buffer[j] = pgm_read_byte(&DMP_UPDATE_BINARY[pos]);
		
		if (!writeMemoryBlock(Buffer + 3, Buffer[2], Buffer[0], Buffer[1], false))
            return;
	}

	// IRQ pin configuration
	if (I2C_write_byte(ADDRESS, REG_INT_CONFIG, 0x00) == false)
		return;

	// Disable FIFO and DMP
	if (I2C_write_byte(ADDRESS, REG_USER_CTRL, 0x00) == false)
		return;

	// Disable all IRQ
	if (I2C_write_byte(ADDRESS, REG_INT_ENABLE, 0x00) == false)
		return;

	// Configure data ready IRQ
	pinMode(data_ready_IRQ_pin, INPUT);
	digitalWrite(data_ready_IRQ_pin, LOW);
	detachInterrupt(data_ready_IRQ_pin);
	attachInterrupt(data_ready_IRQ_pin, data_ready_IRQ_callback, RISING);

	// Initialize success
	g_status = MPU6050_DRIVER_NO_ERROR;
}

void MPU6050_DMP_start() {

	// Initialize success
	I2C_set_internal_address_length(1);
	g_status = MPU6050_DRIVER_ERROR;

	// Enable data ready timeout timer
	REG_TC1_CCR1 = TC_CCR_SWTRG | TC_CCR_CLKEN;
	REG_TC1_IER1 = TC_IER_CPCS;

	// Reset FIFO and DMP
	if (I2C_write_byte(ADDRESS, REG_USER_CTRL, 0x0C) == false)
		return;

	// Enable DMP IRQ
	if (I2C_write_byte(ADDRESS, REG_INT_ENABLE, 0x02) == false)
		return;
	
	// Enable FIFO and DMP
	if (I2C_write_byte(ADDRESS, REG_USER_CTRL, 0xC0) == false)
		return;

	g_status = MPU6050_DRIVER_NO_ERROR;
}

void MPU6050_DMP_stop() {

	I2C_set_internal_address_length(1);
	g_status = MPU6050_DRIVER_ERROR;

	// Disable data ready timeout timer
	REG_TC1_IDR1 = 0xFFFFFFFF;
	REG_TC1_CCR1 = TC_CCR_CLKDIS;

	// Disable FIFO and DMP
	if (I2C_write_byte(ADDRESS, REG_USER_CTRL, 0x00) == false)
		return;

	// Reset FIFO and DMP
	if (I2C_write_byte(ADDRESS, REG_USER_CTRL, 0x0C) == false)
		return;

	// Disable all IRQ
	if (I2C_write_byte(ADDRESS, REG_INT_ENABLE, 0x00) == false)
		return;

	g_status = MPU6050_DRIVER_NO_ERROR;
}

bool MPU6050_is_data_ready() {

	// Check IQR data ready flag
	if (g_is_data_ready == false) {
		if (g_is_data_ready_timeout == true)
			g_status = MPU6050_DRIVER_ERROR;
		return false;
	}
	g_is_data_ready_timeout = false;
	g_is_data_ready = false;


	I2C_set_internal_address_length(1);
	g_status = MPU6050_DRIVER_ERROR;

	// Get current FIFO data size
	uint8_t buffer[2] = { 0 };
	if (I2C_read_bytes(ADDRESS, REG_FIFO_COUNTH, buffer, 2) == false) {
		I2C_write_bits(ADDRESS, REG_USER_CTRL, USERCTRL_FIFO_RESET_MASK, USERCTRL_FIFO_RESET);
		++MPU6050_get_FIFO_size_error_count;
		return false;
	}
	uint32_t FIFO_bytes_count = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];

	// Check FIFO buffer size
	if (FIFO_bytes_count != MPU6050_FIFO_PACKET_SIZE) {
		I2C_write_bits(ADDRESS, REG_USER_CTRL, USERCTRL_FIFO_RESET_MASK, USERCTRL_FIFO_RESET);
		++MPU6050_check_FIFO_size_error_count;
		return false;
	}

	g_status = MPU6050_DRIVER_NO_ERROR;
	return true;
}

void MPU6050_get_data(float* X, float* Y, float* Z) {

	static bool is_start_communication = false;
	if (is_start_communication == false) { // Start communication

		I2C_set_internal_address_length(1);
		if (I2C_async_read_bytes(ADDRESS, REG_FIFO_R_W, MPU6050_FIFO_PACKET_SIZE) == false) {
			++MPU6050_get_data_error_count;
			g_status = MPU6050_DRIVER_ERROR;
			return;
		}
		is_start_communication = true;
		g_status = MPU6050_DRIVER_BUSY;
	}
	else { // Communication started. Wait complite

		// Check I2C driver status
		uint32_t status = I2C_get_status();
		if (status == I2C_DRIVER_NO_ERROR) {

			uint8_t* data = I2C_async_get_rx_buffer_address();
			calculation_XYZ(data, X, Y, Z);

			g_status = MPU6050_DRIVER_NO_ERROR;
			is_start_communication = false;
		}
		else if (status == I2C_DRIVER_ERROR) {
			g_status = MPU6050_DRIVER_ERROR;
			is_start_communication = false;
			++MPU6050_get_data_error_count;
		}
		else if (status == I2C_DRIVER_BUSY) {
			g_status = MPU6050_DRIVER_BUSY;
		}
	}
}

uint32_t MPU6050_get_status() {
	return g_status;
}

void MPU6050_reset_status() {
	g_status = MPU6050_DRIVER_NO_ERROR;
}

//
// INTERNAL INTERFACE
//
/**************************************************************************
* @brief	Function for calculation XYZ
* @param	data: FIFO data
* @param	X: angle on axis X
* @param	Y: angle on axis Y
* @param	Z: angle on axis Z
**************************************************************************/
static void calculation_XYZ(uint8_t* data, float* X, float* Y, float* Z) {

	// ��������� ������������ �� ������
	int16_t RawQ[4] = { 0 };
	RawQ[0] = (static_cast<uint16_t>(data[0]) << 8) | data[1];
	RawQ[1] = (static_cast<uint16_t>(data[4]) << 8) | data[5];
	RawQ[2] = (static_cast<uint16_t>(data[8]) << 8) | data[9];
	RawQ[3] = (static_cast<uint16_t>(data[12]) << 8) | data[13];

	// ��������� ��������
	float Q[4] = { 0 };	// WXYZ
	Q[0] = RawQ[0] / 16384.0f;
	Q[1] = RawQ[1] / 16384.0f;
	Q[2] = RawQ[2] / 16384.0f;
	Q[3] = RawQ[3] / 16384.0f;

	// Euler angles
	float tmp_x = atan2(2.0 * (Q[0] * Q[1] + Q[2] * Q[3]), 1.0 - 2.0 * (Q[1] * Q[1] + Q[2] * Q[2]));
	float tmp_y = asin(2.0 * (Q[0] * Q[2] - Q[3] * Q[1]));
	float tmp_z = atan2(2.0 * (Q[0] * Q[3] + Q[1] * Q[2]), 1.0 - 2.0 * (Q[2] * Q[2] + Q[3] * Q[3]));

	tmp_x *= 180.0 / M_PI;
	tmp_y *= 180.0 / M_PI;
	tmp_z *= 180.0 / M_PI;

	*X = tmp_x;
	*Y = tmp_y;
	*Z = tmp_z;
}

static bool writeMemoryBlock(const uint8_t* pData, int16_t DataSize, uint8_t Bank, uint8_t Addr, bool IsUseProgMem)  {
	const int MemoryChunkSize = 16;
	///////////////////////////////////////////////////////////
	// ��������� ������
	uint8_t* pVerifyBuffer = (uint8_t*)malloc(MemoryChunkSize);
	uint8_t* pProgBuffer = nullptr;
    if (IsUseProgMem == true) 
		pProgBuffer = (uint8_t*)malloc(MemoryChunkSize);
	//
	///////////////////////////////////////////////////////////
    if (I2C_write_byte(ADDRESS, REG_BANK_SEL,Bank & 0x1F) == false)	// ��������� ��������� ������
		return false;
	if (I2C_write_byte(ADDRESS, REG_MEM_START_ADDR, Addr) == false)	// ��������� ���������� ������
		return false;

	for (uint16_t i = 0; i < DataSize; /* NONE */) {

        uint8_t ChunkSize = MemoryChunkSize;
        if (i + ChunkSize > DataSize) 
			ChunkSize = DataSize - i;
		
		if (ChunkSize > 256 - Addr) 
			ChunkSize = 256 - Addr;
        
        if (IsUseProgMem == true) {
            for (uint8_t j = 0; j < ChunkSize; ++j) 
				pProgBuffer[j] = pgm_read_byte(pData + i + j);
        } 
		else 
            pProgBuffer = (uint8_t*)pData + i;
		
        if (I2C_write_bytes(ADDRESS, REG_MEM_R_W,pProgBuffer,ChunkSize) == false)
			return false;
		///////////////////////////////////////////////////////////
		// �������� ���������� ������
		if (I2C_write_byte(ADDRESS, REG_BANK_SEL,Bank & 0x1F) == false)	// ��������� ��������� ������
			return false;
		if (I2C_write_byte(ADDRESS, REG_MEM_START_ADDR, Addr) == false)	// ��������� ���������� ������
			return false;
		//
		I2C_read_bytes(ADDRESS,REG_MEM_R_W,pVerifyBuffer,ChunkSize);
		if (memcmp(pProgBuffer, pVerifyBuffer, ChunkSize) != 0) 
		{
			free(pVerifyBuffer);
			if (IsUseProgMem == true) 
				free(pProgBuffer);
			return false;
		}
		//
		///////////////////////////////////////////////////////////
        i += ChunkSize;
        Addr += ChunkSize;
		if (i < DataSize) 
		{
			if (Addr == 0) 
				Bank++;
			if (I2C_write_byte(ADDRESS,REG_BANK_SEL,Bank & 0x1F) == false)	// ��������� ��������� ������
				return false;
			if (I2C_write_byte(ADDRESS,REG_MEM_START_ADDR,Addr) == false)	// ��������� ���������� ������
				return false;
		}
    }
	free(pVerifyBuffer);
    if (IsUseProgMem == true) 
		free(pProgBuffer);
    return true;
}

static bool writeDMPConfig(const uint8_t* pData, uint16_t DataSize)  {
	uint8_t BufferSize = 64;
    uint8_t* pProgBuffer = new uint8_t[BufferSize];
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    for (int i = 0; i < DataSize; /* NONE */) 
	{
		uint8_t Bank = pgm_read_byte(pData + i++);
		uint8_t Offset = pgm_read_byte(pData + i++);
		uint8_t Len = pgm_read_byte(pData + i++);
        // ������ ������ � DMP ��� ���������� ����������� ��������
        if (Len > 0) 
		{
			if (BufferSize < Len) 
			{
				BufferSize = Len;
				delete[] pProgBuffer;
				pProgBuffer = new uint8_t[BufferSize];
			}
			for (int j = 0; j < Len; ++j) 
				pProgBuffer[j] = pgm_read_byte(pData + i + j);
			//
			if (writeMemoryBlock(pProgBuffer,Len,Bank,Offset,false) == false)
			{
				free(pProgBuffer);
				return false;
			}
            i += Len;
        } 
		else 
		{
			if (pgm_read_byte(pData + i++) == 0x01) 
			{
				if (!I2C_write_byte(ADDRESS,REG_INT_ENABLE,0x32))
				{
					free(pProgBuffer);
					return false;
				}
            } 
			else 
			{
				free(pProgBuffer);
				return false;
			}
        }
	}
	free(pProgBuffer);
	return true;
}


//
// IRQ handlers
//
static void data_ready_IRQ_callback() {

	g_is_data_ready = true;

	// Reset data ready timeout timer
	REG_TC1_CCR1 = TC_CCR_SWTRG | TC_CCR_CLKEN;
}

void TC4_Handler() {

	uint32_t status = REG_TC1_SR1;
	uint32_t irq_mask = REG_TC1_IMR1;

	if ((irq_mask & TC_IMR_CPCS) && (status & TC_SR_CPCS)) {
		g_is_data_ready_timeout = true;
	}
}