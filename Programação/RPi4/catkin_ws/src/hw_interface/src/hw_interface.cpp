#include "hw_interface.hpp"

RobotHWInterface::RobotHWInterface(ros::NodeHandle& nh, i2c_config_t& i2c_configuration, i2c_slave_device_config_t& slave_configuration) : 
    nh(nh), 
    command_timeout_(nh.createTimer(ros::Duration(CMD_VEL_TIMEOUT_TIME), 
        &RobotHWInterface::commandTimeoutCallback, this, true, false)) 
{
    
    init_i2c(i2c_configuration,slave_configuration);

    // Subscreve ao tópico de comandos de movimento
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RobotHWInterface::cmdVelCallback, this);

    // Publishes odometry
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    // Carregar parâmetros do arquivo .yaml
    nh.getParam("robot_geometry/wheel_radius", wheel_radius);
    nh.getParam("robot_geometry/wheel_separation_width", wheel_separation_width);
    nh.getParam("robot_geometry/wheel_separation_length", wheel_separation_length);
    nh.getParam("robot_geometry/front_four_bar_separation", front_four_bar_separation);
    nh.getParam("robot_geometry/deceleration_rate", deceleration_rate);
    nh.getParam("robot_geometry/vcenter_max_speed", vcenter_max_speed);
    nh.getParam("robot_geometry/vcenter_min_speed", vcenter_min_speed);

    nh.getParam("fourbar_geometry/link_a2", link_a2);
    nh.getParam("fourbar_geometry/link_b2", link_b2);
    nh.getParam("fourbar_geometry/link_c", link_c);
    nh.getParam("fourbar_geometry/link_d2", link_d2);
    nh.getParam("fourbar_geometry/phi_2_o", phi_2_o);
    nh.getParam("fourbar_geometry/phi_tangent_lower_limit", phi_tangent_lower_limit);
    nh.getParam("fourbar_geometry/phi_tangent_upper_limit", phi_tangent_upper_limit);
    //nh.getParam("fourbar_geometry/q2_lower_limit", q2_lower_limit);
    //nh.getParam("fourbar_geometry/q2_upper_limit", q2_upper_limit);


    current_time = ros::Time::now();
}

void RobotHWInterface::init_i2c(i2c_config_t& i2c_configuration, i2c_slave_device_config_t& slave_configuration){

    i2c_master.i2c_param_config(i2c_configuration);
    i2c_master.i2c_add_slave_device("espFeliz", slave_configuration);
    i2c_master.i2c_start_bsc();

    while(true){
        
        //ROS_INFO("xxxx controller");
        if(i2c_master.i2c_blocking_read("espFeliz")){
            memcpy(&total_x_displacement_intermed, i2c_master.slave_devices["espFeliz"].rx_buffer,4);
            memcpy(&total_y_displacement_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+4,4);
            memcpy(&total_theta_displacement_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+8,4);
            memcpy(&time_stamp_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+12,4);
            ROS_INFO("Tentando carregar primeiro dado");
            if(time_stamp_intermed == 0x7fffffff){
                throw std::runtime_error("Esp32 is in Error state!");
            }
            else{
                total_x_displacement = (static_cast<double>(total_x_displacement_intermed))/1000000;
                total_y_displacement = (static_cast<double>(total_y_displacement_intermed))/1000000;
                total_theta_displacement = (static_cast<double>(total_theta_displacement_intermed))/1000;
                time_stamp = time_stamp_intermed;

                if(i2c_master.i2c_blocking_read("espFeliz")){
                    memcpy(&total_x_displacement_intermed, i2c_master.slave_devices["espFeliz"].rx_buffer,4);
                    memcpy(&total_y_displacement_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+4,4);
                    memcpy(&total_theta_displacement_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+8,4);
                    memcpy(&time_stamp_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+12,4);

                    if(time_stamp_intermed == 0x7fffffff){
                        throw std::runtime_error("Esp32 is in Error state!");
                    }
                    else{
                        total_x_displacement_old = total_x_displacement;
                        total_y_displacement_old = total_y_displacement;
                        total_theta_displacement_old = total_theta_displacement;
                        time_stamp_old = time_stamp;

                        if(time_stamp_intermed != time_stamp_old){
                            total_x_displacement = (static_cast<double>(total_x_displacement_intermed))/1000000;
                            total_y_displacement = (static_cast<double>(total_y_displacement_intermed))/1000000;
                            total_theta_displacement = (static_cast<double>(total_theta_displacement_intermed))/1000;
                            time_stamp = time_stamp_intermed;

                            vel_linear_x = (total_x_displacement - total_x_displacement_old)/(time_stamp-time_stamp_old);
                            vel_linear_y = (total_y_displacement - total_y_displacement_old)/(time_stamp-time_stamp_old);
                            vel_angular_z = (total_theta_displacement - total_theta_displacement_old)/(time_stamp-time_stamp_old);
                            
                            base_vel_linear_x = cos(total_theta_displacement)*vel_linear_x + sin(total_theta_displacement)*vel_linear_y;
                            base_vel_linear_y = -sin(total_theta_displacement)*vel_linear_x + cos(total_theta_displacement)*vel_linear_y;
                            base_vel_angular_z = vel_angular_z;

                            //std::cout<<total_x_displacement<<" ";
                            //std::cout<<total_y_displacement<<" ";
                            //std::cout<<total_theta_displacement<<" ";
                            //std::cout<<time_stamp<<std::endl;

                            //ROS_INFO("%F ", total_x_displacement);
                            //ROS_INFO("%F ", total_y_displacement);
                            //ROS_INFO("%F ", total_theta_displacement);
                            //ROS_INFO("%d ", time_stamp);
                            


                            i2c_master.set_write(); //preparing for using non blocking functions
                            break;
                        }



                    }
                    



                }

            }
        

        }



    }

}

void RobotHWInterface::calculate_speed(){
    
    //i2c_master.wait_bsc_transfer();
    memcpy(&total_x_displacement_intermed, i2c_master.slave_devices["espFeliz"].rx_buffer,4);
    memcpy(&total_y_displacement_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+4,4);
    memcpy(&total_theta_displacement_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+8,4);
    memcpy(&time_stamp_intermed, (i2c_master.slave_devices["espFeliz"].rx_buffer)+12,4);

    if(time_stamp_intermed == 0x7fffffff){
        throw std::runtime_error("Esp32 is in Error state!");
    }
    else{

        if(time_stamp_intermed != time_stamp){

            total_x_displacement_old = total_x_displacement;
            total_y_displacement_old = total_y_displacement;
            total_theta_displacement_old = total_theta_displacement;
            time_stamp_old = time_stamp;

            total_x_displacement = (static_cast<double>(total_x_displacement_intermed))/1000000;
            total_y_displacement = (static_cast<double>(total_y_displacement_intermed))/1000000;
            total_theta_displacement = (static_cast<double>(total_theta_displacement_intermed))/1000;
            time_stamp = time_stamp_intermed;

            vel_linear_x = (total_x_displacement - total_x_displacement_old)/(time_stamp-time_stamp_old);
            vel_linear_y = (total_y_displacement - total_y_displacement_old)/(time_stamp-time_stamp_old);
            vel_angular_z = (total_theta_displacement - total_theta_displacement_old)/(time_stamp-time_stamp_old);

            base_vel_linear_x = cos(total_theta_displacement)*vel_linear_x + sin(total_theta_displacement)*vel_linear_y;
            base_vel_linear_y = -sin(total_theta_displacement)*vel_linear_x + cos(total_theta_displacement)*vel_linear_y;
            base_vel_angular_z = vel_angular_z;

            //ROS_INFO("%F ", total_x_displacement);
            //ROS_INFO("%F ", total_y_displacement);
            //ROS_INFO("%F ", total_theta_displacement);
            //ROS_INFO("%d ", time_stamp);

            }
        //else continue

        

    }

    //i2c_master.wait_bsc_transfer();

}

void RobotHWInterface::read_data(){
    i2c_master.wait_bsc_transfer();
    if(!(i2c_master.i2c_read("espFeliz"))); //continue;
}

void RobotHWInterface::write_data(){
    
    memcpy(i2c_master.slave_devices["espFeliz"].tx_buffer,&rear_left_wheel_speed,4);
    memcpy((i2c_master.slave_devices["espFeliz"].tx_buffer)+4,&rear_right_wheel_speed,4);
    memcpy((i2c_master.slave_devices["espFeliz"].tx_buffer)+8,&servo_angle,4);
    
    i2c_master.wait_bsc_transfer();
    //carregar dado aqui ou em calculate_speed
    if(!(i2c_master.i2c_write("espFeliz"))); //continue;
}

void RobotHWInterface::cmdVelCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {

    //vel_linear_x_to_esp  =  cos(total_theta_displacement)*msg->linear.x -sin(total_theta_displacement)*msg->linear.y;
    //vel_linear_y_to_esp  =  sin(total_theta_displacement)*msg->linear.x + cos(total_theta_displacement)*msg->linear.y;
    //vel_angular_z_to_esp =  msg->angular.z;

    acker_steering_angle  =  msg->steering_angle;
    acker_central_v_speed  =  msg->speed;

    //ou simplismente
    //vel_linear_x_to_esp = msg->linear.x;
    //vel_linear_y_to_esp = msg->linear.y;
    //vel_angular_z_to_esp =  msg->angular.z;

    // Reinicia o temporizador cada vez que um comando é recebido
    command_timeout_.stop();
    command_timeout_.setPeriod(ros::Duration(CMD_VEL_TIMEOUT_TIME), true); // Reset com auto-restart
    command_timeout_.start();
}

double RobotHWInterface::mapSpeed(double v_input) {
    return std::min(std::max(v_input, vcenter_min_speed), vcenter_max_speed);
}

double RobotHWInterface::mapTangent(double tangent_input) {
    return std::min(std::max(tangent_input, phi_tangent_lower_limit), phi_tangent_upper_limit);
}

void RobotHWInterface::commandTimeoutCallback(const ros::TimerEvent&) {
    updateWheelSpeedForDeceleration();
}

void RobotHWInterface::updateWheelSpeedForDeceleration() {
    // Desacelera cada roda gradualmente até zero

    if (std::abs(rear_left_wheel_speed) > deceleration_rate) rear_left_wheel_speed -= deceleration_rate * (rear_left_wheel_speed > 0 ? 1 : -1);
    else rear_left_wheel_speed = 0;

    if (std::abs(rear_right_wheel_speed) > deceleration_rate) rear_right_wheel_speed -= deceleration_rate * (rear_right_wheel_speed > 0 ? 1 : -1);
    else rear_right_wheel_speed = 0;

    // Verifica se todas as velocidades chegaram a zero, se não, continua desacelerando
    if (rear_left_wheel_speed != 0 || rear_right_wheel_speed != 0) {
        command_timeout_.stop();
        command_timeout_.setPeriod(ros::Duration(CMD_VEL_TIMEOUT_DEACELERATION_PERIOD), true); // Use um intervalo mais curto para desaceleração suave
        command_timeout_.start();
    }
}


void RobotHWInterface::updateOdometry() {

    current_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(total_theta_displacement);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = total_x_displacement;
    odom_trans.transform.translation.y = total_y_displacement;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);
    //ROS_INFO("Odom tf sent");

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = total_x_displacement;
    odom.pose.pose.position.y = total_y_displacement;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = base_vel_linear_x;
    odom.twist.twist.linear.y = base_vel_linear_y;
    odom.twist.twist.angular.z = base_vel_angular_z;

    odom_pub.publish(odom);
    //ROS_INFO("Odom published");

}

void RobotHWInterface::ackermann_inverse(){
    static double tan_phi;
    static double sin_phi;
    static double cos_phi;
    static double phi;

    static double phi_right;

    static double phi_2;

    static double K_2;
    static double Z_2;
    static double W_2;

    static double eq_a;
    static double eq_b;
    static double eq_c;

    static double sin_q2_sol1;
    static double sin_q2_sol2;

    static double cos_q2_sol1;
    static double cos_q2_sol2;

    static double q2_sol1;
    static double q2_sol2;

    static double q2_final;

    static double left_vel;
    static double right_vel;
    static double v_center;
    
    //tan_phi = mapTangent((base_vel_angular_z_to_esp*wheel_separation_length*cos(total_theta_displacement))/(vel_linear_x_to_esp));

    //v_center = mapSpeed(vel_linear_x_to_esp/cos(total_theta_displacement));

    //ou simplesmente
    v_center = acker_central_v_speed;//base_vel_linear_y_to_esp;
    //if(base_vel_linear_y_to_esp < 1.7*)
    //tan_phi = (vel_angular_z_to_esp*wheel_separation_length)/v_center;

    //left_vel = (wheel_separation_length)*((wheel_separation_length*v_center) - (((wheel_separation_width*wheel_separation_length)*wheel_separation_width)/2));
    //right_vel = (wheel_separation_length)*((wheel_separation_length*v_center) + (((base_vel_angular_z_to_esp*wheel_separation_length)*wheel_separation_width)/2));

    //left_vel = ((vel_linear_x_to_esp*tan_phi)/(cos(total_theta_displacement)*wheel_separation_length))*((wheel_separation_length/tan_phi) - (wheel_separation_width/2));
    //right_vel = ((vel_linear_x_to_esp*tan_phi)/(cos(total_theta_displacement)*wheel_separation_length))*((wheel_separation_length/tan_phi) + (wheel_separation_width/2));
    
    phi = acker_steering_angle;

    tan_phi = tan(phi);
    sin_phi  = sin(phi);
    cos_phi  = cos(phi);

    left_vel = v_center - ((((v_center*tan_phi)/wheel_separation_length)*wheel_separation_width)/2);
    right_vel = v_center + ((((v_center*tan_phi)/wheel_separation_length)*wheel_separation_width)/2);

    phi_right = atan((2*wheel_separation_length*sin_phi)/(2*wheel_separation_length*cos_phi + wheel_separation_width*sin_phi));

    phi_2 = phi_2_o-phi_right;

    K_2 = 2*link_a2*(link_c*cos(phi_2) + link_d2);
    Z_2 = 2*link_a2*link_c*sin(phi_2);
    W_2 = 2*link_c*link_d2*cos(phi_2) - link_b2*link_b2 + link_a2*link_a2 + link_c*link_c + link_d2*link_d2;

    eq_a = Z_2*Z_2 + K_2*K_2;
    eq_b = -2*Z_2*W_2;
    eq_c = W_2*W_2 - K_2*K_2;

    sin_q2_sol1 = (-eq_b + sqrt(eq_b*eq_b - 4*eq_a*eq_c))/(2*eq_a);
    sin_q2_sol2 = (-eq_b - sqrt(eq_b*eq_b - 4*eq_a*eq_c))/(2*eq_a);

    //ROS_INFO("%F ",wheel_separation_length);
    //ROS_INFO("%F ",wheel_separation_width );
    //ROS_INFO("%F ",sin_phi);
    //ROS_INFO("%F ",cos_phi);

   //ROS_INFO("%F ",phi_2_o);

    cos_q2_sol1 = (W_2/K_2) - (Z_2/K_2)*sin_q2_sol1;
    cos_q2_sol2 = (W_2/K_2) - (Z_2/K_2)*sin_q2_sol2;

    q2_sol1 = atan2(sin_q2_sol1,cos_q2_sol1);
    q2_sol2 = atan2(sin_q2_sol2,cos_q2_sol2);
    //1.222 70 - 120 //130 max 2.269
    if(1.000 <= q2_sol1 && q2_sol1 <= 3) q2_final = q2_sol1; 

    if(1.000 <= q2_sol2 && q2_sol2 <= 3) q2_final = q2_sol2; 

    //if(q2_lower_limit >= q2_sol1 && q2_sol1 <= q2_upper_limit) q2_final = q2_sol1; 

    //if(q2_lower_limit >= q2_sol2 && q2_sol2 <= q2_upper_limit) q2_final = q2_sol2; 


    rear_left_wheel_speed = static_cast<float>(left_vel/wheel_radius);
    rear_right_wheel_speed = static_cast<float>(right_vel/wheel_radius);
    servo_angle = static_cast<float>(q2_final*(180.0/M_PI));
    
    ROS_INFO("Recebendo");
    ROS_INFO("%F ", total_x_displacement);
    ROS_INFO("%F ", total_y_displacement);
    ROS_INFO("%F ", total_theta_displacement);
    ROS_INFO("%d ", time_stamp);

    //ROS_INFO("Enviando");
    //ROS_INFO("%f ", rear_left_wheel_speed);
    //ROS_INFO("%f ", rear_right_wheel_speed);
    //ROS_INFO("%f ", servo_angle);
    //ROS_INFO("%F ", q2_final);
    //ROS_INFO("%F ", q2_sol1);
    //ROS_INFO("%F ", q2_sol2);
    //ROS_INFO("%F ", phi_2);

    //ROS_INFO("%F ",link_c );

}

