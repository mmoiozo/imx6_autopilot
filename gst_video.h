#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <gst/gst.h>
#include <glib.h>
#include <fcntl.h>		//Used for UART
#include <termios.h>		//Used for UART
#include <errno.h>
#include <sys/types.h>
#include <stdint.h>

void initialize_pipeline(int   argc, char **argv[]);
void initialize_240p(int   argc, char **argv[]);

gboolean bus_call (GstBus *bus, GstMessage *msg);
GstBus *g_bus;
GstMessage *msg;
GstElement *pipeline, *videosrc, *srcq, *videoenc, *encq, *parse, *rtp, *sink, *bayerq, *bayer, *videotransform;

 GstCaps *caps_bayer, *caps_720, *caps_240;