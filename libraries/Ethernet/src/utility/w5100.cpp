/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <string.h>

#include "utility/w5100.h"

// W5x00 controller instance, do not access before calling initialise_wiznet_instance()
WiznetModule *WiznetInstance;

void initialise_wiznet_instance() {
  WiznetInstance = WiznetModule::autodetect();
}

const uint8_t W5200_WRITE_FLAG = 0x80;
const uint8_t W5200_READ_FLAG = 0x00;

const uint8_t W5100_WRITE_FLAG = 0xF0;
const uint8_t W5100_READ_FLAG = 0x0F;


#define TX_RX_MAX_BUF_SIZE 2048
#define TX_BUF 0x1100
#define RX_BUF (TX_BUF + TX_RX_MAX_BUF_SIZE)


WiznetModule *WiznetModule::autodetect()
{
  /* Method
   *
   * Start by assuming we have a W5100, set the reset bit in the mode register and then read
   * back the mode register to check it's zeroed (ie reset complete.)
   *
   * If we don't get a zero back, assume we have a W5200 in which case
   * we're in the middle of a register read. Finish the request that
   * we've accidentally given it.
   */

#if defined(ARDUINO_ARCH_AVR)
  initSS();
  resetSS();
  SPI.begin();
#else
	delay(300); // small delay to leave the Wiznet chip time to initialize
  SPI.begin(SPI_CS);
  // Set clock to 4Mhz (W5100 should support up to about 14Mhz)
  SPI.setClockDivider(SPI_CS, 21);
  SPI.setDataMode(SPI_CS, SPI_MODE0);
#endif

  // To check for W5100, send a sequence of mode bits and check we read
  // back consistent mode bit values in all cases (for W5200 this will just
  // be reading the same memory address over and over, so result shouldn't change.)
  bool is_w5100 = exploratory_modewrite(RST) == 0
    && exploratory_modewrite(PINGBLOCK) == PINGBLOCK
    && exploratory_modewrite(PPOE) == PPOE
    && exploratory_modewrite(RST|PPOE) == 0;

  WiznetModule *result;
  if(is_w5100) {
    return new W5100Module();
	}
  else {
    return new W5200Module();
	}
}

uint8_t WiznetModule::exploratory_modewrite(uint8_t mode_value)
{
  /* Writes and then reads back the mode register on W5100
     However, assumes device type is unknown and may be W5200,
     so the device also reads back additional bytes as if the same
     command had been sent to W5200 (to ensure a consistent device state)
  */

  // W5100 mode write sequence, on W5200 this is interpreted as a request
  // to read 'mode_value' bytes from address 0xF000
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  #if defined(ARDUINO_ARCH_AVR)
  setSS();
  SPI.transfer(W5100_WRITE_FLAG);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(mode_value);
  resetSS();
  #else
  SPI.transfer(SPI_CS, W5100_WRITE_FLAG, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0x00, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0x00, SPI_CONTINUE);
  SPI.transfer(SPI_CS, mode_value, SPI_LAST);
  #endif
  SPI.endTransaction();

  // Read back the value, on W5200 this is interpreted as the first 4
  // reads from the sequence.
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  #if defined(ARDUINO_ARCH_AVR)
  setSS();
  SPI.transfer(W5100_READ_FLAG);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  uint8_t result = SPI.transfer(0x00);
  resetSS();
  #else
  SPI.transfer(SPI_CS, W5100_READ_FLAG, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0x00, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0x00, SPI_CONTINUE);
  uint8_t result = SPI.transfer(SPI_CS, 0x00, SPI_LAST);
  #endif
  SPI.endTransaction();

  // In case this is a W5200, complete the read cycles for the
  // device (NOPs on W5100 as no command flag will be sent)
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  #if defined(ARDUINO_ARCH_AVR)
  setSS();
  for(uint8_t i = 4; i < mode_value; i++)
    SPI.transfer(0x00);
  resetSS();
  #else 
  for(uint8_t i = 4; i < mode_value; i++)
    SPI.transfer(SPI_CS, 0x00, SPI_LAST);
	#endif
  SPI.endTransaction();

  return result;
}

void WiznetModule::init(void)
{
#if defined(ARDUINO_ARCH_AVR)
  SPI.begin();
  initSS();
#else
  SPI.begin(SPI_CS);
  // Set clock to 4Mhz (W5100 should support up to about 14Mhz)
  SPI.setClockDivider(SPI_CS, 21);
  SPI.setDataMode(SPI_CS, SPI_MODE0);
#endif
  SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  writeMR(WiznetModule::RST);
  delay(300); // small delay to leave the Wiznet chip time to initialize after a reset 
  SPI.endTransaction();
}


void W5100Module::init()
{
  WiznetModule::init();
  writeTMSR(0x55);
  writeRMSR(0x55);
}

void W5200Module::init()
{
  WiznetModule::init();
  for (uint8_t i=0; i< get_max_sockets() ; i++) {
    write(get_chbase() + i * 0x100 + 0x001F, 2);
    write(get_chbase() + i * 0x100 + 0x001E, 2);
  }
}

uint16_t WiznetModule::getTXFreeSize(SOCKET s)
{
  uint16_t val=0, val1=0;
  do {
    val1 = readSnTX_FSR(s);
    if (val1 != 0)
      val = readSnTX_FSR(s);
  } 
  while (val != val1);
  return val;
}

uint16_t WiznetModule::getRXReceivedSize(SOCKET s)
{
  uint16_t val=0,val1=0;
  do {
    val1 = readSnRX_RSR(s);
    if (val1 != 0)
      val = readSnRX_RSR(s);
  } 
  while (val != val1);
  return val;
}


void WiznetModule::send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
  // This is same as having no offset in a call to send_data_processing_offset
  send_data_processing_offset(s, 0, data, len);
}

void WiznetModule::send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
  uint16_t ptr = readSnTX_WR(s);
  ptr += data_offset;
  uint16_t offset = ptr & SMASK;
  uint16_t dstAddr = offset + get_sock_tx_addr(s);

  if (offset + len > SSIZE) 
  {
    // Wrap around circular buffer
    uint16_t size = SSIZE - offset;
    write(dstAddr, data, size);
    write(get_sock_tx_addr(s), data + size, len - size);
  } 
  else {
    write(dstAddr, data, len);
  }

  ptr += len;
  writeSnTX_WR(s, ptr);
}


void WiznetModule::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
  uint16_t ptr;
  ptr = readSnRX_RD(s);
  read_data(s, ptr, data, len);
  if (!peek)
  {
    ptr += len;
    writeSnRX_RD(s, ptr);
  }
}

void WiznetModule::read_data(SOCKET s, volatile uint16_t src, volatile uint8_t *dst, uint16_t len)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = src & RMASK;
  src_ptr = get_sock_rx_addr(s) + src_mask;

  if( (src_mask + len) > RSIZE ) 
  {
    size = RSIZE - src_mask;
    read(src_ptr, (uint8_t *)dst, size);
    dst += size;
    read(get_sock_rx_addr(s), (uint8_t *) dst, len - size);
  } 
  else
    read(src_ptr, (uint8_t *) dst, len);
}


uint8_t W5100Module::write(uint16_t _addr, uint8_t _data)
{
#if defined(ARDUINO_ARCH_AVR)
  setSS();  
  SPI.transfer(W5100_WRITE_FLAG);
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(_data);
  resetSS();
#else
  SPI.transfer(SPI_CS, W5100_WRITE_FLAG, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _data);
#endif
  return 1;
}

uint8_t W5200Module::write(uint16_t _addr, uint8_t _data)
{
#if defined(ARDUINO_ARCH_AVR)
  setSS();  
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(W5200_WRITE_FLAG);
  SPI.transfer(0x01);
  SPI.transfer(_data);
  resetSS();
#else
  SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
  SPI.transfer(SPI_CS, W5200_WRITE_FLAG, SPI_CONTINUE);
  SPI.transfer(0x01, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _data);
#endif
  return 1;
}

uint16_t W5100Module::write(uint16_t _addr, const uint8_t *_buf, uint16_t _len)
{
  for (uint16_t i=0; i<_len; i++)
  {
#if defined(ARDUINO_ARCH_AVR)
    setSS();    
    SPI.transfer(W5100_WRITE_FLAG);
    SPI.transfer(_addr >> 8);
    SPI.transfer(_addr & 0xFF);
    _addr++;
    SPI.transfer(_buf[i]);
    resetSS();
#else
	  SPI.transfer(SPI_CS, W5100_WRITE_FLAG, SPI_CONTINUE);
	  SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
	  SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
    _addr++;
	  SPI.transfer(SPI_CS, _buf[i], SPI_LAST);
#endif
  }
  return _len;
}

uint16_t W5200Module::write(uint16_t _addr, const uint8_t *_buf, uint16_t _len)
{
#if defined(ARDUINO_ARCH_AVR)
  setSS();    
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(W5200_WRITE_FLAG | ((_len & 0x7F00) >> 8));
  SPI.transfer(_len & 0xFF);
  for (uint16_t i=0; i<_len; i++)
    SPI.transfer(_buf[i]);
  resetSS();
#else
	SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
	SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
	SPI.transfer(SPI_CS, W5200_WRITE_FLAG | ((_len & 0x7F00) >> 8), SPI_CONTINUE);
  SPI.transfer(SPI_CS, _len & 0xFF, SPI_CONTINUE);
  for (uint16_t i=0; i<_len; i++)
  	SPI.transfer(SPI_CS, _buf[i], i==_len-1 ? SPI_LAST : SPI_CONTINUE);
#endif  
  return _len;
}

uint8_t W5100Module::read(uint16_t _addr)
{
#if defined(ARDUINO_ARCH_AVR)
  setSS();  
  SPI.transfer(W5100_READ_FLAG);
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  uint8_t _data = SPI.transfer(0);
  resetSS();
#else
  SPI.transfer(SPI_CS, W5100_READ_FLAG, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
  uint8_t _data = SPI.transfer(SPI_CS, 0, SPI_LAST);
#endif
  return _data;
}

uint8_t W5200Module::read(uint16_t _addr)
{
#if defined(ARDUINO_ARCH_AVR)
  setSS();  
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(W5200_READ_FLAG);
  SPI.transfer(0x01);
  uint8_t _data = SPI.transfer(0);
  resetSS();
#else
  SPI.transfer(SPI_CS, W5200_READ_FLAG, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
  SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
  SPI.transfer(0x01, SPI_CONTINUE);
  uint8_t _data = SPI.transfer(SPI_CS, 0, SPI_LAST);
#endif
  return _data;
}

uint16_t W5100Module::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  for (uint16_t i=0; i<_len; i++)
  {
#if defined(ARDUINO_ARCH_AVR)
    setSS();
    SPI.transfer(W5100_READ_FLAG);
    SPI.transfer(_addr >> 8);
    SPI.transfer(_addr & 0xFF);
    _addr++;
    _buf[i] = SPI.transfer(0);
    resetSS();
#else
	SPI.transfer(SPI_CS, W5100_READ_FLAG, SPI_CONTINUE);
	SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
	SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
    _buf[i] = SPI.transfer(SPI_CS, 0, SPI_LAST);
    _addr++;
#endif
  }
  return _len;
}

uint16_t W5200Module::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{

#if defined(ARDUINO_ARCH_AVR)
  setSS();
  SPI.transfer(_addr >> 8);
  SPI.transfer(_addr & 0xFF);
  SPI.transfer(W5200_READ_FLAG);
  for (uint16_t i=0; i<_len; i++)
    _buf[i] = SPI.transfer(0);
  resetSS();
#else
	SPI.transfer(SPI_CS, _addr >> 8, SPI_CONTINUE);
	SPI.transfer(SPI_CS, _addr & 0xFF, SPI_CONTINUE);
	SPI.transfer(SPI_CS, W5200_READ_FLAG | ((_len & 0x7F00) >> 8), SPI_CONTINUE);
	SPI.transfer(SPI_CS, _len & 0xFF, SPI_CONTINUE);
  for (uint16_t i=0; i<_len; i++)
    _buf[i] = SPI.transfer(SPI_CS, i==_len-1 ? SPI_LAST : SPI_CONTINUE);
#endif
  return _len;
}

void WiznetModule::execCmdSn(SOCKET s, SockCMD _cmd) {
  // Send command to socket
  writeSnCR(s, _cmd);
  // Wait for command to complete
  while (readSnCR(s))
    ;
}
