build:
	pio run

# кошкомальчик на arch	
sim_max: build
	simavr -m atmega328p -f 16000000 .pio/build/nanoatmega328/firmware.elf

sim_raw_max:
	simavr -m atmega328p -f 16000000 .pio/build/nanoatmega328/firmware.elfma

sim_andrey: build
	C:\Users\Asus\.platformio\packages\tool-simavr\bin\simavr.exe -m atmega328p -f 16000000 .pio/build/nanoatmega328/firmware.elf

sim_raw_andrey:
	C:\Users\Asus\.platformio\packages\tool-simavr\bin\simavr.exe -m atmega328p -f 16000000 .pio/build/nanoatmega328/firmware.elf