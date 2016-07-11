CC=gcc
CFLAGS= $(shell pkg-config gstreamer-1.0 --cflags) 
DEPS = gst_pipeline.h 
OBJ =  gst_pipeline.o 
LIBS= $(shell pkg-config gstreamer-1.0 --libs) -lm

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

gst_pipeline: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)
