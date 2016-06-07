
#define override_control 1

int override_ref_angle = 50;
int override_control_error = 0;
unsigned int override_control_gain = 1000;
int override_send_motor_Z = 0;
int override_control_sat = 5000;


//#if (override_control)

void yaw_control_over_ride(void)
{
  override_control_error = override_ref_angle - (TO_DEG(yaw));
  
  // Escalonamento do ganho x erro 
  /*
  if(override_control_error > 0 && override_control_error < 4) override_control_gain = override_control_gain;
  if(override_control_error > 4 && override_control_error < 8) override_control_gain = override_control_gain*2;
  if(override_control_error > 8) override_control_gain = override_control_gain*4;
  
  if(override_control_error < 0 && override_control_error > -4) override_control_gain = override_control_gain;
  if(override_control_error < -4 && override_control_error > -8) override_control_gain = override_control_gain*2;
  if(override_control_error < -8) override_control_gain = override_control_gain*4;
*/
  
  
  override_send_motor_Z = (override_control_error * override_control_gain);
  // Saturador
  
  if(override_send_motor_Z > motor_speed_default) override_send_motor_Z = motor_speed_default;else
  if(override_send_motor_Z < -(override_control_sat)) override_send_motor_Z = -motor_speed_default;
  
  // Se erro maior que 0, compensar (-)
  if(override_control_error >= 0)
  {
    send_2_motors_acm(2, (override_send_motor_Z));
  }else
  // Se erro menor que 0, compensar (+)
  {
    send_2_motors_acm(2, -(override_send_motor_Z));
  }
  
  mot_data_received.motor_z_sp = override_send_motor_Z;
  Serial.println(override_control_error);
  Serial.println(override_send_motor_Z);
}

//#endif
