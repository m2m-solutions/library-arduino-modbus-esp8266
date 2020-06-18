/*
    ModbusRTU Library for ESP8266/ESP32
    Copyright (C) 2019-2020 Alexander Emelianov (a.m.emelianov@gmail.com)
	https://github.com/emelianov/modbus-esp8266
	This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/
#pragma once
#include "Modbus.h"
#include <HardwareSerial.h>
#if defined(ESP8266)
 #include <SoftwareSerial.h>
#endif

//#define MODBUSRTU_DEBUG
#define MODBUSRTU_BROADCAST 0
#define MODBUSRTU_RESERVE 248
#define MODBUSRTU_SERIAL_BUFFER 256
#define MODBUSRTU_MAX_TIME 10
#define MODBUSRTU_TIMEOUT 1000
#define MODBUSRTU_ADD_REG
//#define MODBUS_STATIC_FRAME 1

class ModbusRTU : public Modbus {
    protected:
        Stream* _port;
        int16_t   _txPin = -1;
		unsigned int _t;	// inter-frame delay in mS
		uint32_t t = 0;		// time sience last data byte arrived
		bool isMaster = false;
		uint8_t  _slaveId;
		uint32_t _timestamp = 0;
		cbTransaction _cb = nullptr;
		void* _data = nullptr;
		uint8_t* _sentFrame = nullptr;
		TAddress _sentReg = COIL(0);
		uint16_t maxRegs = 0x007D;
		#ifdef ESP32
		portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
		#endif
		bool send(uint8_t slaveId, TAddress startreg, cbTransaction cb, void* data = nullptr, bool waitResponse = true);
		// Prepare and send ModbusRTU frame. _frame buffer and _len should be filled with Modbus data
		// slaveId - slave id
		// startreg - first local register to save returned data to (miningless for write to slave operations)
		// cb - transaction callback function
		// data - if not null use buffer to save returned data instead of local registers
		bool rawSend(uint8_t slaveId, uint8_t* frame, uint8_t len);
		bool cleanup(); 	// Free clients if not connected and remove timedout transactions and transaction with forced events
		uint16_t crc16(uint8_t address, uint8_t* frame, uint8_t pdulen);
    public:
		void setBaudrate(uint32_t baud = -1);
	 #if defined(ESP8266)
	 	bool begin(SoftwareSerial* port, int16_t txPin=-1);
	 #endif
		bool begin(HardwareSerial* port, int16_t txPin=-1);
		bool begin(Stream* port);
        void task();
		void master() { isMaster = true; };
		void slave(uint8_t slaveId) {_slaveId = slaveId;};
		uint8_t slave() { return _slaveId; }
		uint32_t eventSource() override {return _slaveId;}
		uint16_t writeHreg(uint8_t slaveId, uint16_t offset, uint16_t value, cbTransaction cb = nullptr);
		uint16_t writeCoil(uint8_t slaveId, uint16_t offset, bool value, cbTransaction cb = nullptr);
		uint16_t readCoil(uint8_t slaveId, uint16_t offset, bool* value, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t writeCoil(uint8_t slaveId, uint16_t offset, bool* value, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t writeHreg(uint8_t slaveId, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t readIsts(uint8_t slaveId, uint16_t offset, bool* value, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t readHreg(uint8_t slaveId, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t readIreg(uint8_t slaveId, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr);

		uint16_t pushCoil(uint8_t slaveId, uint16_t to, uint16_t from, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pullCoil(uint8_t slaveId, uint16_t from, uint16_t to, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pullIsts(uint8_t slaveId, uint16_t from, uint16_t to, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pushHreg(uint8_t slaveId, uint16_t to, uint16_t from, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pullHreg(uint8_t slaveId, uint16_t from, uint16_t to, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pullIreg(uint8_t slaveId, uint16_t from, uint16_t to, uint16_t numregs = 1, cbTransaction cb = nullptr);

		uint16_t pullHregToIreg(uint8_t slaveId, uint16_t offset, uint16_t startreg, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pullCoilToIsts(uint8_t slaveId, uint16_t offset, uint16_t startreg, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pushIstsToCoil(uint8_t slaveId, uint16_t to, uint16_t from, uint16_t numregs = 1, cbTransaction cb = nullptr);
		uint16_t pushIregToHreg(uint8_t slaveId, uint16_t to, uint16_t from, uint16_t numregs = 1, cbTransaction cb = nullptr);

		uint16_t readFileRec(uint8_t slaveId, uint16_t fileNum, uint16_t startRec, uint16_t len, uint8_t* data, cbTransaction cb = nullptr) {
			if (startRec > 0x270F) return 0;
			if (!readSlaveFile(&fileNum, &startRec, &len, 1, FC_READ_FILE_REC)) return 0;
			return send(slaveId, FILE(0), cb, data);
		}
		uint16_t writeFileRec(uint8_t slaveId, uint16_t fileNum, uint16_t startRec, uint16_t len, uint8_t* data, cbTransaction cb = nullptr) {
			if (startRec > 0x270F) return 0;
			if (!writeSlaveFile(&fileNum, &startRec, &len, 1, FC_WRITE_FILE_REC, data)) return 0;
			return send(slaveId, FILE(0), cb);
		}
		uint16_t maskHreg(uint8_t slaveId, uint16_t offset, uint16_t andMask, uint16_t orMask, cbTransaction cb = nullptr) {
			free(_frame);
			_len = 7;
			_frame = (uint8_t*) malloc(_len);
			_frame[0] = FC_MASKWRITE_REG;
			_frame[1] = offset >> 8;
			_frame[2] = offset & 0x00FF;
			_frame[3] = andMask >> 8;
			_frame[4] = andMask & 0x00FF;
			_frame[5] = orMask >> 8;
			_frame[6] = orMask & 0x00FF;
			return send(slaveId, HREG(offset), cb, nullptr, cb);	
		}
/*
		uint16_t readWriteHreg(uint8_t slaveId,
			uint16_t readOffset, uint16_t* value, uint16_t numregs,
			uint16_t writeOffset, uint16_t* value, uint16_t numregs,
			cbTransaction cb = nullptr, uint8_t unit = MODBUSIP_UNIT);
*/
};