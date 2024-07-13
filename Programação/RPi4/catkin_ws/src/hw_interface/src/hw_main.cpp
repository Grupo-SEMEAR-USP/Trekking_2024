#include "hw_interface.hpp"



int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh; // Cria um NodeHandle

    //setting i2c
    i2c_config_t i2c_config = i2c_config_info(BSC_UNIT_1,I2C_SLC_FREQUENCY,GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE);

    i2c_slave_device_config_t slave_config;
        slave_config.tx_buffer_size = I2C_TX_BUFFER_SIZE;
        slave_config.rx_buffer_size = I2C_RX_BUFFER_SIZE;
        slave_config.i2c_slave_adress = I2C_SLAVE_ADRESS;

    ROS_INFO("controlle");
    ROS_INFO("Pós controller");
    RobotHWInterface controller(nh,i2c_config,slave_config); // Passa o NodeHandle como argumento
    ROS_INFO("Pós contro");

    ros::Rate rate(HW_IF_UPDATE_FREQ);

    //ros::AsyncSpinner spinner(4);
    std::cout<<"Pós spinner";

    //spinner.start();

    std::cout<<"Pós spinner start";

    while (ros::ok())
    {  
        controller.write_data();
        controller.updateOdometry();
        //wait
        controller.read_data();
        controller.ackermann_inverse();
        //wait
        controller.calculate_speed();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}