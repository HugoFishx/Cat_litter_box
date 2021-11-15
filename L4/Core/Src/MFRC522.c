#include "MFRC522.h"

#define _chipSelectPin  GPIO_PIN_6		//chipSelectPin
#define _resetPowerDownPin  GPIO_PIN_7	//resetPowerDownPin

GPIO_InitTypeDef _powerDownPinInput = (GPIO_InitTypeDef){	.Pin = _resetPowerDownPin,
															.Mode = GPIO_MODE_INPUT,
															.Pull = GPIO_NOPULL,
															.Speed = GPIO_SPEED_FREQ_LOW  };
GPIO_InitTypeDef _powerDownPinOutput = (GPIO_InitTypeDef){	.Pin = _resetPowerDownPin,
															.Mode = GPIO_MODE_OUTPUT_PP,
															.Pull = GPIO_NOPULL,
															.Speed = GPIO_SPEED_FREQ_LOW  };
SPI_HandleTypeDef hspi3;


static uint8_t UNUSED_PIN = UINT8_MAX;
/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */

void PCD_WriteRegisterOneByte(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
									uint8_t value			///< The value to write.
								) {
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&reg, 1, -1) ;
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&value, 1, -1) ;
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_SET);
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
									uint8_t count,			///< The number of bytes to write to the register
									uint8_t *values		///< The values to write. Byte array.
								) {
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&reg, 1, -1);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)values, count, -1);
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_SET);
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t PCD_ReadRegisterOneByte(	PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
								) {
	uint8_t value[2];
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_RESET);
	uint8_t address = 0x80 | reg;					// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&address, (uint8_t*)&value, 2, -1);
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_SET);
	return value[1];
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
// TODO: review this function, receive multiple values from the same address. could be simplified
void PCD_ReadRegister(	PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
								uint8_t count,			///< The number of bytes to read
								uint8_t *values,		///< Byte array to store the values in.
								uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	if (count == 0) {
		return;
	}
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_RESET);
	uint8_t address = 0x80 | reg;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		uint8_t value;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &address, (uint8_t*) &value, 2, -1);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}

	uint8_t address_array[count - index];
	for (int i = 0; i < count - index; i++)
		address_array[i] = address;

	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) address_array, (uint8_t*) &values[index], count - index + 1, -1);
	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_SET);
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask			///< The bits to set.
									) {
	uint8_t tmp;

	HAL_Delay(1);
	tmp = PCD_ReadRegisterOneByte(reg);
	PCD_WriteRegisterOneByte(reg,tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask			///< The bits to clear.
									  ) {
	uint8_t tmp;

	HAL_Delay(1);
	tmp = PCD_ReadRegisterOneByte(reg);
	PCD_WriteRegisterOneByte(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CalculateCRC(uint8_t*data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												uint8_t length,	///< In: The number of bytes to transfer.
												uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	PCD_WriteRegisterOneByte(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegisterOneByte(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegisterOneByte(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegisterOneByte(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n = PCD_ReadRegisterOneByte(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegisterOneByte(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadRegisterOneByte(CRCResultRegL);
			result[1] = PCD_ReadRegisterOneByte(CRCResultRegH);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init() {
	bool hardReset = false;

	HAL_GPIO_WritePin(GPIOB, _chipSelectPin, GPIO_PIN_SET);
	// If a valid pin number has been set, pull device out of power down / reset state.
	if (_resetPowerDownPin != UNUSED_PIN) {
		// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
		HAL_GPIO_Init(GPIOB, &_powerDownPinInput);

		if (HAL_GPIO_ReadPin(GPIOB, _resetPowerDownPin) == GPIO_PIN_RESET) {	// The MFRC522 chip is in power down mode.
			HAL_GPIO_Init(GPIOB, &_powerDownPinOutput);
			HAL_GPIO_WritePin(GPIOB, _resetPowerDownPin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOB, _resetPowerDownPin, GPIO_PIN_SET);
			HAL_Delay(50);
			hardReset = true;
		}
	}

	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		PCD_Reset();
	}

	// Reset baud rates
	PCD_WriteRegisterOneByte(TxModeReg, 0x00);
	PCD_WriteRegisterOneByte(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegisterOneByte(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegisterOneByte(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegisterOneByte(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	PCD_WriteRegisterOneByte(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegisterOneByte(TReloadRegL, 0xE8);

	PCD_WriteRegisterOneByte(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegisterOneByte(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Initializes the MFRC522 chip.
 */
/*void PCD_Init(	byte resetPowerDownPin	///< Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
					) {
	PCD_Init(SS, resetPowerDownPin); // SS is defined in pins_arduino.h
} // End PCD_Init()

/**
 * Initializes the MFRC522 chip.
 */
/*void PCD_Init(	byte chipSelectPin,		///< Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
						byte resetPowerDownPin	///< Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
					) {
	_chipSelectPin = chipSelectPin;
	_resetPowerDownPin = resetPowerDownPin;
	// Set the chipSelectPin as digital output, do not select the slave yet
	PCD_Init();
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset() {
	PCD_WriteRegisterOneByte(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	//uint8_t
	int count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		HAL_Delay(50);
	} while ((PCD_ReadRegisterOneByte(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegisterOneByte(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegisterOneByte(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff() {
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_TransceiveData(uint8_t*sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
													uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CommunicateWithPICC(uint8_t command,		///< The command to execute. One of the PCD_Command enums.
														uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
														uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
														uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteRegisterOneByte(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegisterOneByte(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteRegisterOneByte(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegisterOneByte(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegisterOneByte(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	for (i = 2000; i > 0; i--) {
		uint8_t n = PCD_ReadRegisterOneByte(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return STATUS_TIMEOUT;
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegisterOneByte(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	uint8_t _validBits = 0;
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = PCD_ReadRegisterOneByte(FIFOLevelReg) & 0x7F;	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegisterOneByte(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()


StatusCode PICC_REQA_or_WUPA(uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
												uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	uint8_t validBits;
	StatusCode status;
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t*responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522

	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegisterOneByte(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);

			// SELF_MODIFIED CODE START
			if (result == STATUS_OK)  {
				memcpy(&(uid->uidByte), responseBuffer, responseLength);
				return STATUS_OK;
			}
			// SELF_MODIFIED CODE END

			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = PCD_ReadRegisterOneByte(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return bool
 */
bool PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	PCD_WriteRegisterOneByte(TxModeReg, 0x00);
	PCD_WriteRegisterOneByte(RxModeReg, 0x00);
	// Reset ModWidthReg

	PCD_WriteRegisterOneByte(ModWidthReg, 0x26);
	StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return bool
 */
bool PICC_ReadCardSerial(Uid* uid) {
	StatusCode result = PICC_Select(uid,0);
	return (result == STATUS_OK);
} // End
