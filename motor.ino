/** 
 * \brief Initialize motors.
 */
void Init_Motors(void)  // Menor número, rampa mais suave
{
  Serial1.println("1AC100");
  delay(200);
  Serial1.println("1DEC100");
  delay(200);
  Serial1.println("2AC100");
  delay(200);
  Serial1.println("2DEC100");
  delay(200);
  Serial1.println("3AC100");
  delay(200);
  Serial1.println("3DEC100");
  delay(200);
  /*
    char l_str[] = "EN";
    uint8_t l_bcmd[] = { 200, 0, 202, 255 };
    int l_node;
  */  
    sen_data_local.motor_x_st = 1;
    sen_data_local.motor_y_st = 1;
    sen_data_local.motor_z_st = 1;
  /*  
    // MOTOR 1
    l_node = 1;
    if( enviarComando( l_node, l_str ) != 0 )
    {
        sen_data.motor_x_st = 0;
    }
    if( enviarComandoBinario( l_node, l_bcmd, 4 ) != 0 )
    {
        sen_data.motor_x_st = 0;
    }
    
    // MOTOR 2
    l_node = 2;
    if( enviarComando( l_node, l_str ) != 0 )
    {
        sen_data.motor_y_st = 0;
    }
    if( enviarComandoBinario( l_node, l_bcmd, 4 ) != 0 )
    {
        sen_data.motor_y_st = 0;
    }
    
    // MOTOR 3
    l_node = 3;
    if( enviarComando( l_node, l_str ) != 0 )
    {
        sen_data.motor_z_st = 0;
    }
    if( enviarComandoBinario( l_node, l_bcmd, 4 ) != 0 )
    {
        sen_data.motor_z_st = 0;
    }
    */
}

void meu_delay(int tempo)
{
  int i;
  
  for(i=0; i<tempo; i++)
  {
    delay(1);
    if(Serial1.available())
    {
      sen_data_local.motor_x_st = 1;
      sen_data_local.motor_y_st = 1;
      sen_data_local.motor_z_st = 1;
      break;
    }
  }
  sen_data_local.motor_x_st = 1;
  sen_data_local.motor_y_st = 1;
  sen_data_local.motor_z_st = 1;
}

String request_motor_speed(uint8_t motor_id)
{
  uint8_t a;
  
  String feedback = "";
  
  //Serial.println("---------------------------");
  //Serial.print("Motor ");
  //Serial.print(motor_id);
  //Serial.print(" : ");
  
   while (Serial1.available())
     Serial1.read();
     
  
  Motors_Vel_Index = motor_id-1;
  Serial1.print(motor_id);
  Serial1.println("GV");
  
  delay(50);
  
  while (Serial1.available()) {
    
    char inChar = (char)Serial1.read();
    if(inChar!= '\r' && inChar!= '\n')
      feedback += inChar;
    //Serial.print(inChar);
    
    meu_delay(10);
  }
  
  
    
    
  
  //while(stringComplete == false)
  //{
    //serialEvent1();
  //}
  
  //while(inputString != (String)'\n')
  //{
    //Serial.print(inputString);
    //inputString++;
  //}
  //Serial.println(" ");
  //{
  //  serialEvent1();
  //}

  
  print_var = 0;
  /*
    while (Serial1.available() > 0) {
      
      aux = Serial1.read();
      
      //Serial.print(aux);
      
      if(aux != 0x0A && 
          aux != 0x0D &&
            aux != 0x4F && 
              aux != 0x4B)
      {
        rx_Motor[rx_Motor_Index] = aux;
        rx_Motor_Index++;
        rx_Motor_Available = 0xFF;
        //Serial.println(rx_Motor_Index);
      }else
      {
        rx_Motor_Index = 0;
        if(rx_Motor_Available)
        {
          for(a=0; a<4; a++)
          {
            Serial.print(rx_Motor[a]);
          }
          
          for(a=0; a<4; a++)
          {
            rx_Motor[a]=0;
          }
        rx_Motor_Available = 0;
        Serial.println("");
        }
      }
  }
  */
  
  return feedback;
}

void send_2_motors_acm(uint8_t motor_id, int16_t motor_speed)
{
  uint8_t a;
  int16_t motor_speed_to_send;
  
  motor_speed_to_send = motor_speed_default + motor_speed;
  
  //rx_ptr = (uint8_t *) &rx_buff;
  
  if(motor_speed_to_send < (motor_speed_default-motor_speed_limit)) motor_speed_to_send = 0;//-motor_speed_limit;
  if(motor_speed_to_send > motor_speed_limit) motor_speed_to_send = motor_speed_limit;
  
  if((motor_id == 3 || motor_id == 1)) // Deixar motores 1 e 2 em 5000 RPM para trabalhar somente o eixo de YAW
  {
    Serial1.print(motor_id);
    Serial1.print("V");
    Serial1.println(motor_speed_default);
  }
  else
  {
    Serial1.print(motor_id);
    Serial1.print("V");
    Serial1.println(motor_speed_to_send);
  }
  
  //index = receive_frame_acm((char *)rx_ptr, 4);
  
  //Serial1.println(index);
  
  //Serial.println("");
  
  //Serial.println("Sending Motor");
  
  //receive_array( rx_buff, 2 );
  
  //Serial1.println("Motor Feedback: ");
  
  //Serial.print(Serial1.read());
  
  //for(index=0; index<sizeof rx_buff; index++)
  //{
  //  Serial1.print(rx_buff[index]);
  //}
  
  //Serial1.println("");
  
  //Serial.print(motor_id);
  //Serial.print("V");
  //Serial.println(motor_speed);
  
}
