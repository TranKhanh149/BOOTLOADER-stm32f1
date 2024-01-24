# BOOTLOADER-stm32f1
Description: After Reset, PC jump to 0x0800000 then check condition in while(1) to jump to function bootloader(). (In my example, I do not set condition, so after reset mcu will jump to function bootloader()).
In blink project, I set delay(3000) before blink. So after resetting, chip will delay 3s before blink (0x08008000)
