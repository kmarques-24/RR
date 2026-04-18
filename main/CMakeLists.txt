# use idf.py set-target esp32s3 to relink files if recently renamed. idf.py menuconfig also helps

idf_component_register(SRCS "main.cpp"                   
                    "rr_os_service.cpp"
                    "uros_service.cpp"   
                    "estimator.cpp" 
                    "controller.cpp"  
                    "utils.cpp" 
                    "imu_service.cpp"
                    "tof_service.cpp"
                    "twai_test_main.c" 
                    "twai_service.cpp" 
                    "hardware_motors.cpp" 
                    "hardware_encoders.cpp"
                    "led.cpp"   
                    "radio_service.cpp"     
                    INCLUDE_DIRS "include"
                    )
#spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT) # not using rn