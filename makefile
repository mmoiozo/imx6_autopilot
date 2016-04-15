CC=gcc
CFLAGS= $(shell pkg-config gstreamer-1.0 --cflags) 
DEPS = read_sensors.h serial_com.h AHRS.h PWM.h spi_uart.h gst_video.h stabilisation.h
OBJ = main_ap_loop.o read_sensors.o serial_com.o AHRS.o PWM.o spi_uart.o gst_video.o stabilisation.o
LIBS= $(shell pkg-config gstreamer-1.0 --libs) -lm

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

auto_pilot: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)
