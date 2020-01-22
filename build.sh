#!/bin/sh

: '
 BUILD SCRIPT FOR ESP32
 Author: Vikram vel
 Date: 10 Sept 2019

 Inputs:
 1 - Release Build
 2 - Debug Build

 Notes:
 Debug Build:   Debug builds are basically to track the developer builds, the version is incremented
                only if the make command was successfull. The verison number is tracked from the
                counter.h file in the main source code repository.
                Also after the successfull build the script tries to upload the firmware, so ensure
                the ESP32 is in "Waiting for download mode", also ensure if right COM port number is
                set below in the python script ( --port).

 Release Build: Release builds do not have auto inrement of the version values, hence the right 
                release verison needs to be updated before the build in version.h file in the source.

'
 


build_type=0

version=`sed -n 's/^.*BUILD_VERSION //p' main/counter.h`

 function debug_build {
        echo " "
        echo " "
        echo "************************************************************************************"
        echo "********                        ESP 32 Debug BUILD                         *********"
        echo " "
        echo "Starting Debug build... "
        make
        if [ $? -eq 0 ] ; then
            clear
            echo ""
            echo ""
            echo "Debug Build Successfull"
            #increment debug build only if build is success
            ((version++))
            echo "Build Number :" $version
            printf '%s\n' '#ifndef __COUNTER_H__' '#define __COUNTER_H__' "#define BUILD_VERSION $version" '#define DEBUG_BUILD 1' '#endif' > main/counter.h
            cp build/app-template.bin build/ESP32_1_0_1_$version.bin
            echo " "
            echo "File Name: ESP32_1_0_1_"$version".bin"
            echo ""
            echo ""
            echo "Writting Image file to the ESP32 ... "
            # Check the COM port ESP is connected and update the below --port COM<portnumber>
            python /home/v.vel/esp/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port COM19 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0xd000 /c/firmware/eos-wifi/build/ota_data_initial.bin 0x1000 /c/firmware/eos-wifi/build/bootloader/bootloader.bin 0x10000 /c/firmware/eos-wifi/build/app-template.bin 0x8000 /c/firmware/eos-wifi/build/partitions_two_ota.bin

        else 
            echo ""
            echo ""
            echo "***********************************************************"
            echo "                     Build Failure !"
            echo "***********************************************************"
            return -1
        fi
 }
 function release_build {
     echo " "
     echo " "
     echo "*******************************"
     echo "******** ESP 32 BUILD *********"
     echo " "
     echo "Starting build... "
     
     read -s -p "Kindly check if version number has been updated in version.h has been updated! (y/n) " release_check
     echo $release_check
     if [ $release_check == 'y' ] || [ $release_check == 'Y' ]
     then
        echo "Starting build... "
        make -w
     else
        echo "Build not started. Update version.h and start again."
     fi
 }



read -s -p "Enter build type (1 - Debug, 2 - Release): " build_type
echo $build_type

 case $build_type in
       1) debug_build;;
       2) release_build;;
  esac

