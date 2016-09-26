SOURCES=main.c touch.c dynamixel.c adc.c dxl_hal.c timers.c serial.c
avr:
	avr-gcc -mmcu=atmega2561 -std=gnu99 -Wall $(SOURCES) -o main.elf -I../include
	avr-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature main.elf main.hex
clean:
	rm -f *.o *.out *.gch *.a *.hex *.elf *.exe