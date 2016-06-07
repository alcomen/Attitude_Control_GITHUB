/** \fn read_from_ima
 * \brief Reads 18 bytes from serial and fills mot_data with them.
 * This is overridden by SPI since it receives and sends simultaneously.
 */
 
void send_to_ima(void)
{
  byte i;
  byte * l_ptr_o,         /**< Pointer to data to be sent (o - output). */
  *l_ptr_i,         /**< Pointer to store data received (i - input). */
  l_flag = 0,      /**< Flag to indicate that data is being received. */
  l_in_cnt = 0,    /**< Counter for incoming data. */
  l_inbyte;        /**< Byte to store incoming bytes. */
  
  //copy_sensor_data(oringem, destino)
  copy_sensor_data( sen_data_local, &sen_data_to_ima );    // Converter de Littleendian para Bigendian
  
  l_ptr_o = (byte*) &sen_data_to_ima;  // Estrutura após a função de passagem Endian 
  
  Udp.beginPacket(ipRemote, portRemote);
  
  for( i = 0; i < 120; i++ )
    {
      Udp.write(*l_ptr_o);
      l_ptr_o++;
    }
    
  Udp.endPacket();
}
  

void read_from_ima_ethernet( uint8_t *index )
{
    uint8_t *l_ptr, *i_ptr;
    uint8_t i;
    s_motor ethernet_received;
    
    l_ptr = (uint8_t *) &ethernet_received;
    
    // Primeiro passa todos os 12 bytes recebidos do IMA para O Buffer (ethernet_received) do tipo s_motor
    for( i = 0; i < 12; i++ )
    {
        *l_ptr = *index;
        l_ptr++;
        index++;
    }
    
    // Converte de BigEndian (ethernet_received) para LittleEndian e salva em "ethernet_received"
    copy_motor_data( ethernet_received, &mot_data_received );
}
