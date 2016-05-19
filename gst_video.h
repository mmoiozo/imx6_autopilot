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
void initialize_720p(int   argc, char **argv[]);
void start_1080p_record(int   argc, char **argv[]);
void start_720p_record(int   argc, char **argv[]);
void start_720x960_record(int   argc, char **argv[]);
void check_pipeline_status();

gboolean bus_call (GstBus *bus, GstMessage *msg);
GstBus *g_bus;
GstMessage *msg;
GstElement *pipeline, *videosrc, *srcq, *videoenc, *encq, *parse, *rtp, *sink, *bayerq, *bayerq1, *bayerq2, *bayer, *bayer1, *bayer2, *videotransform, *rawq, *mpegmux, *flip, *flipq;

 GstCaps *caps_bayer, *caps_720, *caps_240, *caps_raw, *caps_ipu;