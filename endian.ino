void copy_sensor_data( struct s_ima_data sen_data_src, struct s_ima_data *sen_data_dst )
{
    //sen_data_dst -> msg_hdr        = sen_data_src.msg_hdr;
    sen_data_dst -> msg_id         = sen_data_src.msg_id;
    
    change_endian_32( ( uint8_t * ) &sen_data_src.accel_x,      ( uint8_t * ) &sen_data_dst -> accel_x );
    change_endian_32( ( uint8_t * ) &sen_data_src.accel_x_s,    ( uint8_t * ) &sen_data_dst -> accel_x_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.accel_y,      ( uint8_t * ) &sen_data_dst -> accel_y );
    change_endian_32( ( uint8_t * ) &sen_data_src.accel_y_s,    ( uint8_t * ) &sen_data_dst -> accel_y_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.accel_z,      ( uint8_t * ) &sen_data_dst -> accel_z );
    change_endian_32( ( uint8_t * ) &sen_data_src.accel_z_s,    ( uint8_t * ) &sen_data_dst -> accel_z_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.magnetom_x,   ( uint8_t * ) &sen_data_dst -> magnetom_x );
    change_endian_32( ( uint8_t * ) &sen_data_src.magnetom_x_s, ( uint8_t * ) &sen_data_dst -> magnetom_x_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.magnetom_y,   ( uint8_t * ) &sen_data_dst -> magnetom_y );
    change_endian_32( ( uint8_t * ) &sen_data_src.magnetom_y_s, ( uint8_t * ) &sen_data_dst -> magnetom_y_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.magnetom_z,   ( uint8_t * ) &sen_data_dst -> magnetom_z );
    change_endian_32( ( uint8_t * ) &sen_data_src.magnetom_z_s, ( uint8_t * ) &sen_data_dst -> magnetom_z_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.gyro_x,       ( uint8_t * ) &sen_data_dst -> gyro_x );
    change_endian_32( ( uint8_t * ) &sen_data_src.gyro_x_s,     ( uint8_t * ) &sen_data_dst -> gyro_x_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.gyro_y,       ( uint8_t * ) &sen_data_dst -> gyro_y );
    change_endian_32( ( uint8_t * ) &sen_data_src.gyro_y_s,     ( uint8_t * ) &sen_data_dst -> gyro_y_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.gyro_z,       ( uint8_t * ) &sen_data_dst -> gyro_z );
    change_endian_32( ( uint8_t * ) &sen_data_src.gyro_z_s,     ( uint8_t * ) &sen_data_dst -> gyro_z_s );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.roll,         ( uint8_t * ) &sen_data_dst -> roll );
    change_endian_32( ( uint8_t * ) &sen_data_src.pitch,        ( uint8_t * ) &sen_data_dst -> pitch );
    change_endian_32( ( uint8_t * ) &sen_data_src.yaw,          ( uint8_t * ) &sen_data_dst -> yaw );
    
    change_endian_32( ( uint8_t * ) &sen_data_src.quat[ 0 ],    ( uint8_t * ) &sen_data_dst -> quat[ 0 ] );
    change_endian_32( ( uint8_t * ) &sen_data_src.quat[ 1 ],    ( uint8_t * ) &sen_data_dst -> quat[ 1 ] );
    change_endian_32( ( uint8_t * ) &sen_data_src.quat[ 2 ],    ( uint8_t * ) &sen_data_dst -> quat[ 2 ] );
    change_endian_32( ( uint8_t * ) &sen_data_src.quat[ 3 ],    ( uint8_t * ) &sen_data_dst -> quat[ 3 ] );
    
    change_endian_16( ( uint8_t * ) &sen_data_src.motor_x_sp,    ( uint8_t * ) &sen_data_dst -> motor_x_sp );
    change_endian_32( ( uint8_t * ) &sen_data_src.motor_x_st,    ( uint8_t * ) &sen_data_dst -> motor_x_st );
    
    change_endian_16( ( uint8_t * ) &sen_data_src.motor_y_sp,    ( uint8_t * ) &sen_data_dst -> motor_y_sp );
    change_endian_32( ( uint8_t * ) &sen_data_src.motor_y_st,    ( uint8_t * ) &sen_data_dst -> motor_y_st );
    
    change_endian_16( ( uint8_t * ) &sen_data_src.motor_z_sp,    ( uint8_t * ) &sen_data_dst -> motor_z_sp );
    change_endian_32( ( uint8_t * ) &sen_data_src.motor_z_st,    ( uint8_t * ) &sen_data_dst -> motor_z_st );
    
    sen_data_dst -> optional       = sen_data_src.optional;
}

void copy_motor_data( struct s_motor mot_data_src, struct s_motor * mot_data_dst )
{
    mot_data_dst -> msg_id        = mot_data_src.msg_id;
    mot_data_dst -> act_en        = mot_data_src.act_en;
    
    change_endian_16( ( uint8_t * ) &mot_data_src.motor_x_sp,   ( uint8_t * ) &mot_data_dst -> motor_x_sp );
    mot_data_dst -> motor_x_w     = mot_data_src.motor_x_w;
    
    change_endian_16( ( uint8_t * ) &mot_data_src.motor_y_sp,   ( uint8_t * ) &mot_data_dst -> motor_y_sp );
    mot_data_dst -> motor_y_w     = mot_data_src.motor_y_w;
    
    change_endian_16( ( uint8_t * ) &mot_data_src.motor_z_sp,   ( uint8_t * ) &mot_data_dst -> motor_z_sp );
    mot_data_dst -> motor_z_w     = mot_data_src.motor_z_w;
    
    mot_data_dst -> optional      = mot_data_src.optional;
}

void change_endian_16( uint8_t * v_src, uint8_t * v_dst )
{
    v_dst[ 0 ] = v_src[ 1 ];
    v_dst[ 1 ] = v_src[ 0 ];
}

void change_endian_32( uint8_t * v_src, uint8_t * v_dst )
{
    v_dst[ 0 ] = v_src[ 3 ];
    v_dst[ 1 ] = v_src[ 2 ];
    v_dst[ 2 ] = v_src[ 1 ];
    v_dst[ 3 ] = v_src[ 0 ];
}
