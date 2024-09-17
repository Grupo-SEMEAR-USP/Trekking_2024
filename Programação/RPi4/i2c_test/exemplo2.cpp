#include <bsc_i2c_interface.hpp>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <cmath>



int main(){

    i2c_config_t i2c_config = {};
        i2c_config.i2c_port = 1; 
        i2c_config.slc_frequency = 400000; 
        i2c_config.wait_port_time = 200;
        i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE; 
        i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE; 
        i2c_config.falling_edge_delay = 320;
        i2c_config.rising_edge_delay = 320;
        i2c_config.wait_slave_timeout = 640; 
        i2c_config.interrupt_mode = DISABLE_INTERRUPT;
        i2c_config.use_10bit_adress = false;

    i2c_slave_device_config_t slave_config;
        slave_config.tx_buffer_size = 12;
        slave_config.rx_buffer_size = 16;
        slave_config.i2c_slave_adress = 0x69;

    bsc_i2c_handle i2c_master;
    i2c_master.i2c_param_config(i2c_config);
    i2c_master.i2c_add_slave_device("espFeliz", slave_config);
    i2c_master.i2c_start_bsc();

    double x = 0;
    double y = 0;
    double th = 0;
    uint32_t time_stamp = 0;

    int xi = 0;
    int yi = 0;
    int thi = 0;
    uint32_t time_stampi = 0;
    
    usleep(2000000);

    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    while (1){
        float velleft = 10;
        float velright  =10;
        float serv = 80;
        
        //std::cout<<"voc1"<<std::endl;
        //std::cout<<"voc0"<<std::endl;
        //uint8_t 
        memcpy(i2c_master.slave_devices["espFeliz"].tx_buffer,&velleft,4);
        memcpy((i2c_master.slave_devices["espFeliz"].tx_buffer)+4,&velright,4);
        memcpy((i2c_master.slave_devices["espFeliz"].tx_buffer)+8,&serv,4);
        //std::cout<<"voc";
        //i2c_master.wait_bsc_transfer();
    //carregar dado aqui ou em calculate_speed
        i2c_master.i2c_blocking_write("espFeliz"); //continue;
        //std::cout<<"voc2"<<std::endl;
        
        usleep(1500);

        if(i2c_master.i2c_blocking_read("espFeliz")){
        


            memcpy(&xi, i2c_master.slave_devices["espFeliz"].rx_buffer,4);
            memcpy(&yi, (i2c_master.slave_devices["espFeliz"].rx_buffer)+4,4);
            memcpy(&thi, (i2c_master.slave_devices["espFeliz"].rx_buffer)+8,4);
            memcpy(&time_stampi, (i2c_master.slave_devices["espFeliz"].rx_buffer)+12,4);


            x = (static_cast<double>(xi))/1000000;
            y = (static_cast<double>(yi))/1000000;
            th = (static_cast<double>(thi))/1000;
            time_stamp = time_stampi;
        
        //for (int i = 0; i < 8 ;i++){
        //    std::cout<<(int)i2c_master.slave_devices["espFeliz"].rx_buffer[i];
        //}
        //std::cout<<std::endl;
        std::cout<<"x: "<<x<<" y: "<<y<<" th: "<<th<<" time: "<<time_stamp;
        std::cout<<std::endl;
        }
        else{
            std::cout<<"failed"<<std::endl;
        }
        
        usleep(1500);
        
    }

    return 0;
}