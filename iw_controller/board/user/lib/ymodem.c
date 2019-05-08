#include "ymodem.h"



#define CRC16_F       /* activate the CRC16 integrity */

uint8_t aPacketData[PACKET_1K_SIZE + PACKET_DATA_INDEX + PACKET_TRAILER_SIZE];


static void PrepareIntialPacket(uint8_t *p_data, const uint8_t *p_file_name, uint32_t length);
static void PreparePacket(uint8_t *p_source, uint8_t *p_packet, uint8_t pkt_nr, uint32_t size_blk);
static int ReceivePacket(serial_handle_t *handle,uint8_t *p_data, uint32_t *p_length, uint32_t timeout);
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);
uint8_t CalcChecksum(const uint8_t *p_data, uint32_t size);


/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int port_ymodem_recv(serial_handle_t *handle,uint8_t *dst,uint32_t size,uint32_t timeout)
{
    int rc;
    int read_total = 0,read_left = size;
    utils_timer_t timer;
    
    utils_timer_init(&timer,timeout,false);

    while (utils_timer_value(&timer) > 0 && read_left > 0) {
        rc = serial_select(handle,utils_timer_value(&timer));
        if (rc == 0) {
            log_error("ymodem select timeout.\r\n");
            return -1;
        }
        if (rc < 0) {
            log_error("ymodem select err.\r\n");
            return -1;
        }

        rc = serial_read(handle,(char*)dst,read_left);
        if (rc < 0) {
            log_error("ymodem read err.\r\n");
            return -1;
        }
        read_total += rc;
        read_left -= rc;
    }

    if (read_left > 0) {
        return -1;
    }

    return 0;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int port_ymodem_send(serial_handle_t *handle,uint8_t *src,uint32_t size,uint32_t timeout)
{
    int rc;

    rc = serial_write(handle,(char*)src,size);
    if (rc != size) {
        log_error("ymodem write err.\r\n");
        return -1;
    }
    rc = serial_complete(handle,timeout);
    if (rc != 0) {
        log_error("ymodem complete err.\r\n");
        return -1;
    }

    return 0;
}
/*
* @brief 发送单个字节
* @param src 发送的字节
* @param 
* @return 0：成功 -1：失败
* @note
*/
static int Serial_PutByte(serial_handle_t *handle,char src)
{
    uint8_t send = src;

    return port_ymodem_send(handle,&send,1,100);
}
/**
  * @brief  Receive a packet from sender
  * @param  data
  * @param  length
  *     0: end of transmission
  *     2: abort by sender
  *    >0: packet length
  * @param  timeout
  * @retval 0: normally return
  *         -1: abort by user
  */
static int ReceivePacket(serial_handle_t *handle,uint8_t *p_data, uint32_t *p_length, uint32_t timeout)
{
  uint32_t crc;
  uint32_t packet_size = 0;
  int rc;
  uint8_t char1;

  *p_length = 0;

  rc = port_ymodem_recv(handle, &char1, 1, timeout);

  if (rc == 0)
  {
    switch (char1)
    {
      case SOH:
        packet_size = PACKET_SIZE;
        break;
      case STX:
        packet_size = PACKET_1K_SIZE;
        break;
      case EOT:
        break;
      case CA:
        if ((port_ymodem_recv(handle, &char1, 1, timeout) == 0) && (char1 == CA))
        {
          packet_size = 2;
        }
        else
        {
          rc = -1;
        }
        break;
      case ABORT1:
      case ABORT2:
        rc = -2;
        break;
      default:
        rc = -1;
        break;
    }
    *p_data = char1;

    if (packet_size >= PACKET_SIZE )
    {
      rc = port_ymodem_recv(handle, &p_data[PACKET_NUMBER_INDEX], packet_size + PACKET_OVERHEAD_SIZE, timeout);

      /* Simple packet sanity check */
      if (rc == 0 )
      {
        if (p_data[PACKET_NUMBER_INDEX] != ((p_data[PACKET_CNUMBER_INDEX]) ^ NEGATIVE_BYTE))
        {
          packet_size = 0;
          rc = -1;
        }
        else
        {
          /* Check packet CRC */
          crc = p_data[ packet_size + PACKET_DATA_INDEX ] << 8;
          crc += p_data[ packet_size + PACKET_DATA_INDEX + 1 ];
          if (Cal_CRC16(&p_data[PACKET_DATA_INDEX], packet_size) != crc )
          {
            packet_size = 0;
            rc = -1;
          }
        }
      }
      else
      {
        packet_size = 0;
      }
    }
  }
  *p_length = packet_size;
  return rc;
}

/**
  * @brief  Prepare the first block
  * @param  p_data:  output buffer
  * @param  p_file_name: name of the file to be sent
  * @param  length: length of the file to be sent in bytes
  * @retval None
  */
static void PrepareIntialPacket(uint8_t *p_data, const uint8_t *p_file_name, uint32_t length)
{
  uint32_t i, j = 0;
  uint8_t astring[10];

  /* first 3 bytes are constant */
  p_data[PACKET_START_INDEX] = SOH;
  p_data[PACKET_NUMBER_INDEX] = 0x00;
  p_data[PACKET_CNUMBER_INDEX] = 0xff;

  /* Filename written */
  for (i = 0; (p_file_name[i] != '\0') && (i < FILE_NAME_LENGTH); i++)
  {
    p_data[i + PACKET_DATA_INDEX] = p_file_name[i];
  }

  p_data[i + PACKET_DATA_INDEX] = 0x00;

  /* file size written */
  Int2Str (astring, length);
  i = i + PACKET_DATA_INDEX + 1;
  while (astring[j] != '\0')
  {
    p_data[i++] = astring[j++];
  }

  /* padding with zeros */
  for (j = i; j < PACKET_SIZE + PACKET_DATA_INDEX; j++)
  {
    p_data[j] = 0;
  }
}

/**
  * @brief  Prepare the data packet
  * @param  p_source: pointer to the data to be sent
  * @param  p_packet: pointer to the output buffer
  * @param  pkt_nr: number of the packet
  * @param  size_blk: length of the block to be sent in bytes
  * @retval None
  */
static void PreparePacket(uint8_t *p_source, uint8_t *p_packet, uint8_t pkt_nr, uint32_t size_blk)
{
  uint8_t *p_record;
  uint32_t i, size, packet_size;

  /* Make first three packet */
  packet_size = size_blk >= PACKET_1K_SIZE ? PACKET_1K_SIZE : PACKET_SIZE;
  size = size_blk < packet_size ? size_blk : packet_size;
  if (packet_size == PACKET_1K_SIZE)
  {
    p_packet[PACKET_START_INDEX] = STX;
  }
  else
  {
    p_packet[PACKET_START_INDEX] = SOH;
  }
  p_packet[PACKET_NUMBER_INDEX] = pkt_nr;
  p_packet[PACKET_CNUMBER_INDEX] = (~pkt_nr);
  p_record = p_source;

  /* Filename packet has valid data */
  for (i = PACKET_DATA_INDEX; i < size + PACKET_DATA_INDEX;i++)
  {
    p_packet[i] = *p_record++;
  }
  if ( size  <= packet_size)
  {
    for (i = size + PACKET_DATA_INDEX; i < packet_size + PACKET_DATA_INDEX; i++)
    {
      p_packet[i] = 0x1A; /* EOF (0x1A) or 0x00 */
    }
  }
}

/**
  * @brief  Update CRC16 for input byte
  * @param  crc_in input value 
  * @param  input byte
  * @retval None
  */
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
  uint32_t crc = crc_in;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;
    if(in & 0x100)
      ++crc;
    if(crc & 0x10000)
      crc ^= 0x1021;
  }
  
  while(!(in & 0x10000));

  return crc & 0xffffu;
}

/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = p_data+size;

  while(p_data < dataEnd)
    crc = UpdateCRC16(crc, *p_data++);
 
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);

  return crc&0xffffu;
}

/**
  * @brief  Calculate Check sum for YModem Packet
  * @param  p_data Pointer to input data
  * @param  size length of input data
  * @retval uint8_t checksum value
  */
uint8_t CalcChecksum(const uint8_t *p_data, uint32_t size)
{
  uint32_t sum = 0;
  const uint8_t *p_data_end = p_data + size;

  while (p_data < p_data_end )
  {
    sum += *p_data++;
  }

  return (sum & 0xffu);
}


/**
  * @brief  Receive a file using the ymodem protocol with CRC16.
  * @param  p_size The size of the file.
  * @retval COM_StatusTypeDef result of reception/programming
  */
COM_StatusTypeDef Ymodem_Receive (serial_handle_t *handle,uint32_t flash_addr,uint32_t flash_size,char *file_name, uint32_t *p_size )
{
  uint32_t i, packet_length, session_done = 0, file_done, errors = 0, session_begin = 0;
  uint32_t flashdestination, ramsource, filesize;
  uint8_t *file_ptr;
  uint8_t file_size[FILE_SIZE_LENGTH], tmp, packets_received;
  COM_StatusTypeDef result = COM_OK;

  /* Initialize flashdestination variable */
  flashdestination = flash_addr;

  while ((session_done == 0) && (result == COM_OK))
  {
    packets_received = 0;
    file_done = 0;
    while ((file_done == 0) && (result == COM_OK))
    {
      switch (ReceivePacket(handle,aPacketData, &packet_length, DOWNLOAD_TIMEOUT))
      {
        case 0:
          errors = 0;
          switch (packet_length)
          {
            case 2:
              /* Abort by sender */
              Serial_PutByte(handle,ACK);
              result = COM_ABORT;
              break;
            case 0:
              /* End of transmission */
              Serial_PutByte(handle,ACK);
              file_done = 1;
              break;
            default:
              /* Normal packet */
              if (aPacketData[PACKET_NUMBER_INDEX] != packets_received)
              {
                Serial_PutByte(handle,NAK);
              }
              else
              {
                if (packets_received == 0)
                {
                  /* File name packet */
                  if (aPacketData[PACKET_DATA_INDEX] != 0)
                  {
                    /* File name extraction */
                    i = 0;
                    file_ptr = aPacketData + PACKET_DATA_INDEX;
                    while ( (*file_ptr != 0) && (i < FILE_NAME_LENGTH))
                    {
                      file_name[i++] = *file_ptr++;
                    }

                    /* File size extraction */
                    file_name[i++] = '\0';
                    i = 0;
                    file_ptr ++;
                    while ( (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH))
                    {
                      file_size[i++] = *file_ptr++;
                    }
                    file_size[i++] = '\0';
                    Str2Int(file_size, &filesize);

                    /* Test the size of the image to be sent */
                    /* Image size is greater than Flash size */
                    if (*p_size > (flash_size + 1))
                    {
                      /* End session */
                      tmp = CA;
                      port_ymodem_send(handle, &tmp, 1, NAK_TIMEOUT);
                      port_ymodem_send(handle, &tmp, 1, NAK_TIMEOUT);
                      result = COM_LIMIT;
                    }
                    /* erase user application area */
                    flash_if_erase(flash_addr,flash_size);
                    *p_size = filesize;

                    Serial_PutByte(handle,ACK);
                    Serial_PutByte(handle,CRC16);
                  }
                  /* File header packet is empty, end session */
                  else
                  {
                    Serial_PutByte(handle,ACK);
                    file_done = 1;
                    session_done = 1;
                    break;
                  }
                }
                else /* Data packet */
                {
                  ramsource = (uint32_t) & aPacketData[PACKET_DATA_INDEX];

                  /* Write received data in Flash */
                  if (flash_if_write(flashdestination, (uint8_t*) ramsource, packet_length) == 0)                   
                  {
                    flashdestination += packet_length;
                    Serial_PutByte(handle,ACK);
                  }
                  else /* An error occurred while writing to Flash memory */
                  {
                    /* End session */
                    Serial_PutByte(handle,CA);
                    Serial_PutByte(handle,CA);
                    result = COM_DATA;
                  }
                }
                packets_received ++;
                session_begin = 1;
              }
              break;
          }
          break;
        case -2: /* Abort actually */
          Serial_PutByte(handle,CA);
          Serial_PutByte(handle,CA);
          result = COM_ABORT;
          break;
        default:
          if (session_begin > 0)
          {
            errors ++;
          }
          if (errors > MAX_ERRORS)
          {
            /* Abort communication */
            Serial_PutByte(handle,CA);
            Serial_PutByte(handle,CA);
          }
          else
          {
            Serial_PutByte(handle,CRC16); /* Ask for a packet */
          }
          break;
      }
    }
  }
  return result;
}

/**
  * @brief  Transmit a file using the ymodem protocol
  * @param  p_buf: Address of the first byte
  * @param  p_file_name: Name of the file sent
  * @param  file_size: Size of the transmission
  * @retval COM_StatusTypeDef result of the communication
  */
COM_StatusTypeDef Ymodem_Transmit (serial_handle_t *handle,uint8_t *p_buf, const uint8_t *p_file_name, uint32_t file_size,uint32_t max_size)
{
  uint32_t errors = 0, ack_recpt = 0, size = 0, pkt_size;
  uint8_t *p_buf_int;
  COM_StatusTypeDef result = COM_OK;
  uint32_t blk_number = 1;
  uint8_t a_rx_ctrl[2];
  uint8_t i;
#ifdef CRC16_F    
  uint32_t temp_crc;
#else /* CRC16_F */   
  uint8_t temp_chksum;
#endif /* CRC16_F */  

  /* Prepare first block - header */
  PrepareIntialPacket(aPacketData, p_file_name, file_size);

  while (( !ack_recpt ) && ( result == COM_OK ))
  {
    /* Send Packet */
    port_ymodem_send(handle, &aPacketData[PACKET_START_INDEX], PACKET_SIZE + PACKET_HEADER_SIZE, NAK_TIMEOUT);

    /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F    
    temp_crc = Cal_CRC16(&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
    Serial_PutByte(handle,temp_crc >> 8);
    Serial_PutByte(handle,temp_crc & 0xFF);
#else /* CRC16_F */   
    temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
    Serial_PutByte(handle,temp_chksum);
#endif /* CRC16_F */

    /* Wait for Ack and 'C' */
    if (port_ymodem_recv(handle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == 0)
    {
      if (a_rx_ctrl[0] == ACK)
      {
        ack_recpt = 1;
      }
      else if (a_rx_ctrl[0] == CA)
      {
        if ((port_ymodem_recv(handle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == 0) && (a_rx_ctrl[0] == CA))
        {
          osDelay( 2 );
          serial_flush(handle);
          result = COM_ABORT;
        }
      }
    }
    else
    {
      errors++;
    }
    if (errors >= MAX_ERRORS)
    {
      result = COM_ERROR;
    }
  }

  p_buf_int = p_buf;
  size = file_size;

  /* Here 1024 bytes length is used to send the packets */
  while ((size) && (result == COM_OK ))
  {
    /* Prepare next packet */
    PreparePacket(p_buf_int, aPacketData, blk_number, size);
    ack_recpt = 0;
    a_rx_ctrl[0] = 0;
    errors = 0;

    /* Resend packet if NAK for few times else end of communication */
    while (( !ack_recpt ) && ( result == COM_OK ))
    {
      /* Send next packet */
      if (size >= PACKET_1K_SIZE)
      {
        pkt_size = PACKET_1K_SIZE;
      }
      else
      {
        pkt_size = PACKET_SIZE;
      }

      port_ymodem_send(handle, &aPacketData[PACKET_START_INDEX], pkt_size + PACKET_HEADER_SIZE, NAK_TIMEOUT);
      
      /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F    
      temp_crc = Cal_CRC16(&aPacketData[PACKET_DATA_INDEX], pkt_size);
      Serial_PutByte(handle,temp_crc >> 8);
      Serial_PutByte(handle,temp_crc & 0xFF);
#else /* CRC16_F */   
      temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX], pkt_size);
      Serial_PutByte(handle,temp_chksum);
#endif /* CRC16_F */
      
      /* Wait for Ack */
      if ((port_ymodem_recv(handle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == 0) && (a_rx_ctrl[0] == ACK))
      {
        ack_recpt = 1;
        if (size > pkt_size)
        {
          p_buf_int += pkt_size;
          size -= pkt_size;
          if (blk_number == (max_size / PACKET_1K_SIZE))
          {
            result = COM_LIMIT; /* boundary error */
          }
          else
          {
            blk_number++;
          }
        }
        else
        {
          p_buf_int += pkt_size;
          size = 0;
        }
      }
      else
      {
        errors++;
      }

      /* Resend packet if NAK  for a count of 10 else end of communication */
      if (errors >= MAX_ERRORS)
      {
        result = COM_ERROR;
      }
    }
  }

  /* Sending End Of Transmission char */
  ack_recpt = 0;
  a_rx_ctrl[0] = 0x00;
  errors = 0;
  while (( !ack_recpt ) && ( result == COM_OK ))
  {
    Serial_PutByte(handle,EOT);

    /* Wait for Ack */
    if (port_ymodem_recv(handle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == 0)
    {
      if (a_rx_ctrl[0] == ACK)
      {
        ack_recpt = 1;
      }
      else if (a_rx_ctrl[0] == CA)
      {
        if ((port_ymodem_recv(handle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == 0) && (a_rx_ctrl[0] == CA))
        {
          osDelay( 2 );
          serial_flush(handle);
          result = COM_ABORT;
        }
      }
    }
    else
    {
      errors++;
    }

    if (errors >=  MAX_ERRORS)
    {
      result = COM_ERROR;
    }
  }

  /* Empty packet sent - some terminal emulators need this to close session */
  if ( result == COM_OK )
  {
    /* Preparing an empty packet */
    aPacketData[PACKET_START_INDEX] = SOH;
    aPacketData[PACKET_NUMBER_INDEX] = 0;
    aPacketData[PACKET_CNUMBER_INDEX] = 0xFF;
    for (i = PACKET_DATA_INDEX; i < (PACKET_SIZE + PACKET_DATA_INDEX); i++)
    {
      aPacketData [i] = 0x00;
    }

    /* Send Packet */
    port_ymodem_send(handle, &aPacketData[PACKET_START_INDEX], PACKET_SIZE + PACKET_HEADER_SIZE, NAK_TIMEOUT);

    /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F    
    temp_crc = Cal_CRC16(&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
    Serial_PutByte(handle,temp_crc >> 8);
    Serial_PutByte(handle,temp_crc & 0xFF);
#else /* CRC16_F */   
    temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
    Serial_PutByte(handle,temp_chksum);
#endif /* CRC16_F */

    /* Wait for Ack and 'C' */
    if (port_ymodem_recv(handle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == 0)
    {
      if (a_rx_ctrl[0] == CA)
      {
          osDelay( 2 );
          serial_flush(handle);
          result = COM_ABORT;
      }
    }
  }

  return result; /* file transmitted successfully */
}