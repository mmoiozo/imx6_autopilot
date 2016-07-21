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
#include "gst_pipeline.h"


#define MAX_BUF 4


int pipeline_status = -1;//
int wait_for_state_change = 0;//
int state_wait_count = 0;
GstState old_state, new_state;
int rec_com = 0;

FILE *fp;

int fd_tx;
int fd_rx;
int res = 0;
char * myfifo = "/tmp/myfifo";
char * tx_fifo = "/tmp/tx_fifo";
char * rx_fifo = "/tmp/rx_fifo";
   
char data_tx[4];
char data_rx[4] = {0,0,0,0};

int loop_status = 1;


int main (int argc, char **argv)//[]

{
  
  
     //START GSTREAMER PIPELINE//
     //start_720p_record(&argc,&argv);
  //start_720p_flip(&argc,&argv);
 //start_720p_mpeg4(&argc,&argv);
 //start_720p(&argc,&argv);
  start_1080p(&argc,&argv);
    
    mkfifo(rx_fifo, 0777);

    /* open, read, and display the message from the FIFO */
    fd_tx = open(tx_fifo, O_RDONLY | O_NONBLOCK);
    printf("open tx at gst_pipeline: %d\n", fd_tx);
    
    fp = fopen("/home/alarm/log/gst_log.txt", "a");
	//get system time
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	//printf("%s\n", asctime(tm));
        fprintf(fp, "STARTING NEW GSTREAMER pipeline at: %s\n", asctime(tm));
        fclose(fp);
  
  while(loop_status == 1)
   {
    fd_rx = open(rx_fifo, O_WRONLY | O_NONBLOCK);
    //printf("open rx: %d\n", fd_rx);
    res = write(fd_rx, data_rx, sizeof(data_rx));
    //printf("write:%d\n",res);
    close(fd_rx);
    //fd = open(myfifo, O_RDONLY | O_NONBLOCK);
    res = read(fd_tx, data_tx, MAX_BUF);
    //printf("read:%d\n",res);
    if (res < 0)fd_tx = open(tx_fifo, O_RDONLY | O_NONBLOCK);//probably needs to be opened
    
    rec_com = data_tx[0];
    data_rx[0] = data_tx[0];
    data_rx[1] = (char)pipeline_status;
    //printf("Received: %d\n", data_tx[0]);
     
   check_pipeline_status();//check if pipeline has to be stopped or started
   usleep(250000);//4HZ
   }
   return 0;
}



void check_pipeline_status()
{
        GstStateChangeReturn state_ret;
  
     // See if we have pending messages on the bus and handle them
        while ((msg = gst_bus_pop (g_bus))) 
	{
          // Call your bus message handler
          bus_call (g_bus, msg);
          gst_message_unref (msg); 
	}
	
	if(rec_com == 1 && new_state != GST_STATE_PLAYING && wait_for_state_change == 0)
        {
            gst_element_set_state (pipeline, GST_STATE_PLAYING);
            wait_for_state_change = 1;// wat for getting to playing state
        }
        else if(rec_com == 2 && new_state != GST_STATE_NULL && wait_for_state_change == 0)
        {
	  //printf("state change attempt : %s\n", new_state);
            state_ret = gst_element_set_state (pipeline, GST_STATE_NULL);
	    wait_for_state_change = 1;//wait for getting to null state
	    if(state_ret == GST_STATE_CHANGE_SUCCESS)
	    {
	      new_state = GST_STATE_NULL;
	      pipeline_status = 0;
	      wait_for_state_change = 0;//
	      printf("Stopped recording\n");
	    }
        }
	
	//wait 4 sec serial_com
	if(wait_for_state_change != 0 && state_wait_count < 8)
	{
	  state_wait_count++;
	}
	else if(wait_for_state_change != 0)
	{
	  wait_for_state_change = 0;
	  state_wait_count = 0;
	} 
	if(new_state == GST_STATE_NULL)pipeline_status = 0;
        else if(new_state == GST_STATE_PLAYING)pipeline_status = 1;
	
}

gboolean bus_call (GstBus *bus, GstMessage *msg)//static
{
  switch (GST_MESSAGE_TYPE (msg)) {
 
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      
      fp = fopen("/home/alarm/log/gst_log.txt", "a");
      fprintf(fp,"GST End of stream\n");
      fclose(fp);
      //loop_status = 0;
      break;
 
    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;
 
      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);
 
      g_printerr ("Error: %s\n", error->message);
      fp = fopen("/home/alarm/log/gst_log.txt", "a");
      fprintf(fp,"GST Error: %s\n", error->message);
      fclose(fp);
      g_error_free (error);
 
      //loop_status = 0;
      break;
    }
    case GST_MESSAGE_STATE_CHANGED:
          /* We are only interested in state-changed messages from the pipeline */
    
          gst_message_parse_state_changed (msg, &old_state, &new_state, NULL);
          g_print ("Element %s changed state from %s to %s.\n",
          GST_OBJECT_NAME (msg->src),
          gst_element_state_get_name (old_state),
          gst_element_state_get_name (new_state));
	  
	  fp = fopen("/home/alarm/log/gst_log.txt", "a");
          fprintf(fp,"GST Element %s changed state from %s to %s.\n",
          GST_OBJECT_NAME (msg->src),
          gst_element_state_get_name (old_state),
          gst_element_state_get_name (new_state));
	  fclose(fp);
	  
	  wait_for_state_change = 0;//state is changed may now order a new state
	  
	  
          break;
    default:
         g_printerr ("Unexpected message received.\n");
	 fp = fopen("/home/alarm/log/gst_log.txt", "a");
	 fprintf(fp,"Uknown GST message: %s\n",GST_MESSAGE_TYPE_NAME(msg));
	 fclose(fp);
      break;
  }
 
  return TRUE;
}

gboolean start_720p_mpeg4(int *argc, char ***argv)
{

 const gchar* nano_str;

 guint major, minor, micro, nano;
 
 
//int *ptr2 = &argv;
 gst_init (argc, argv);

 gst_version (&major, &minor, &micro, &nano);

 if (nano == 1)
   nano_str = "(CVS)";
 else if (nano == 2)
   nano_str = "(Prerelease)";
 else
   nano_str = "";
   printf ("This program is linked against GStreamer %d.%d.%d %s\n",major, minor, micro, nano_str);

  // Create gstreamer elements /
  pipeline = gst_pipeline_new ("video_testsrc");
  //videosrc = gst_element_factory_make ("videotestsrc", "videosrc");
  videosrc = gst_element_factory_make ("imxv4l2src", "videosrc");
  srcq = gst_element_factory_make ("queue", "srcq");
  //bayer = gst_element_factory_make ("imxbayer", "bayer");
  bayer1 = gst_element_factory_make ("imxbayer1sthalf", "bayer1");
  bayer2 = gst_element_factory_make ("imxbayer2ndhalf", "bayer2");
  bayerq1 = gst_element_factory_make ("queue", "bayerq1");
  bayerq2 = gst_element_factory_make ("queue", "bayerq2");
  flip = gst_element_factory_make ("imxipuvideotransform", "flip");
  flipq = gst_element_factory_make ("queue", "flipq");
  videoenc = gst_element_factory_make ("imxvpuenc_mpeg4", "videoenc");
  encq = gst_element_factory_make ("queue", "encq");
  mpegmux = gst_element_factory_make ("mpegtsmux", "mpegmux");
  sink = gst_element_factory_make ("filesink", "sink");
  
  
 
  if (!pipeline || !videosrc || !srcq || !bayer1 || !bayer2 || !bayerq1 || !bayerq2 || !flip || !flipq || !videoenc
    || !encq || !mpegmux || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return FALSE;
  }

  g_object_set (G_OBJECT (sink),"location","/home/alarm/media/clip_720p_1.mp4","sync",FALSE, NULL);
  g_object_set (G_OBJECT (videosrc),"capture-mode",1, "capture-format", 1,"fps-n",30, NULL);
  g_object_set (G_OBJECT (flip),"output-rotation", 2, NULL);
  g_object_set (G_OBJECT (videoenc),"bitrate", 10000 , NULL);
  g_object_set (G_OBJECT (bayer1),"fbnum",2, "fbset", 1,"extbuf",1,"red",1.15,"green", 1.0,"blue",1.25,"chrom",140, NULL);
  g_object_set (G_OBJECT (bayer2),"fbnum",2, NULL);


 g_bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));

  // we add all elements into the pipeline /
  gst_bin_add_many (GST_BIN (pipeline), videosrc, srcq, bayer1, bayerq1, bayer2, bayerq2, flip, flipq, videoenc, encq, mpegmux, sink, NULL);
  
  caps_bayer = gst_caps_new_simple ("video/x-bayer","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  caps_raw = gst_caps_new_simple ("video/x-raw","format",G_TYPE_STRING, "I420","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  caps_ipu = gst_caps_new_simple ("video/x-raw","format",G_TYPE_STRING, "I420","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  
  
  
  
  if(!gst_element_link_filtered(videosrc,srcq, caps_bayer))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
 
 gst_element_link_many (srcq,bayer1,bayerq1, bayer2, NULL);
 
 if(!gst_element_link_filtered(bayer2,bayerq2, caps_raw))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 }  
 
 gst_element_link_many (bayerq2,flip, NULL);
 
 if(!gst_element_link_filtered(flip,flipq, caps_ipu))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
  
  //gst_element_link_many (srcq, videoenc, encq, parse, rtp, sink, NULL);
  gst_element_link_many (flipq, videoenc, encq, mpegmux, sink, NULL);
 // Set the pipeline to "playing" state/
  g_print ("Streaming to port: %s\n", argv);
  
  //gst_element_set_state (pipeline, GST_STATE_PLAYING);
  return TRUE;
}


gboolean start_720p(int *argc, char ***argv)
{

 const gchar* nano_str;

 guint major, minor, micro, nano;
 
 
//int *ptr2 = &argv;
 gst_init (argc, argv);

 gst_version (&major, &minor, &micro, &nano);

 if (nano == 1)
   nano_str = "(CVS)";
 else if (nano == 2)
   nano_str = "(Prerelease)";
 else
   nano_str = "";
   printf ("This program is linked against GStreamer %d.%d.%d %s\n",major, minor, micro, nano_str);

  // Create gstreamer elements /
  pipeline = gst_pipeline_new ("video_testsrc");
  //videosrc = gst_element_factory_make ("videotestsrc", "videosrc");
  videosrc = gst_element_factory_make ("imxv4l2src", "videosrc");
  srcq = gst_element_factory_make ("queue", "srcq");
  //bayer = gst_element_factory_make ("imxbayer", "bayer");
  bayer1 = gst_element_factory_make ("imxbayer1sthalf", "bayer1");
  bayer2 = gst_element_factory_make ("imxbayer2ndhalf", "bayer2");
  bayerq1 = gst_element_factory_make ("queue", "bayerq1");
  bayerq2 = gst_element_factory_make ("queue", "bayerq2");
  videoenc = gst_element_factory_make ("imxvpuenc_mpeg4", "videoenc");
  encq = gst_element_factory_make ("queue", "encq");
  mpegmux = gst_element_factory_make ("mpegtsmux", "mpegmux");
  sink = gst_element_factory_make ("filesink", "sink");
  
  
 
  if (!pipeline || !videosrc || !srcq || !bayer1 || !bayer2 || !bayerq1 || !bayerq2 || !videoenc
    || !encq || !mpegmux || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return FALSE;
  }

  g_object_set (G_OBJECT (sink),"location","/home/alarm/media/clip_720p_1.mp4","sync",FALSE, NULL);
  g_object_set (G_OBJECT (videosrc),"capture-mode",1, "capture-format", 1,"fps-n",30, NULL);
  g_object_set (G_OBJECT (videoenc),"bitrate", 10000 , NULL);
  g_object_set (G_OBJECT (bayer1),"fbnum",2, "fbset", 1,"extbuf",1,"red",1.15,"green", 1.0,"blue",1.25,"chrom",140, NULL);
  g_object_set (G_OBJECT (bayer2),"fbnum",2, NULL);


 g_bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));

  // we add all elements into the pipeline /
  gst_bin_add_many (GST_BIN (pipeline), videosrc, srcq, bayer1, bayerq1, bayer2, bayerq2, videoenc, encq, mpegmux, sink, NULL);
  
  caps_bayer = gst_caps_new_simple ("video/x-bayer","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  caps_raw = gst_caps_new_simple ("video/x-raw","format",G_TYPE_STRING, "I420","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  
  
  
  
  if(!gst_element_link_filtered(videosrc,srcq, caps_bayer))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
 
 gst_element_link_many (srcq,bayer1,bayerq1, bayer2, NULL);
 
 if(!gst_element_link_filtered(bayer2,bayerq2, caps_raw))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 }  
  
  //gst_element_link_many (srcq, videoenc, encq, parse, rtp, sink, NULL);
  gst_element_link_many (bayerq2, videoenc, encq, mpegmux, sink, NULL);
 // Set the pipeline to "playing" state/
  g_print ("Streaming to port: %s\n", argv);
  
  //gst_element_set_state (pipeline, GST_STATE_PLAYING);
  return TRUE;
}

gboolean start_1080p(int *argc, char ***argv)
{

 const gchar* nano_str;

 guint major, minor, micro, nano;
 
 
//int *ptr2 = &argv;
 gst_init (argc, argv);

 gst_version (&major, &minor, &micro, &nano);

 if (nano == 1)
   nano_str = "(CVS)";
 else if (nano == 2)
   nano_str = "(Prerelease)";
 else
   nano_str = "";
   printf ("This program is linked against GStreamer %d.%d.%d %s\n",major, minor, micro, nano_str);

  // Create gstreamer elements /
  pipeline = gst_pipeline_new ("video_testsrc");
  //videosrc = gst_element_factory_make ("videotestsrc", "videosrc");
  videosrc = gst_element_factory_make ("imxv4l2src", "videosrc");
  srcq = gst_element_factory_make ("queue", "srcq");
  //bayer = gst_element_factory_make ("imxbayer", "bayer");
  bayer1 = gst_element_factory_make ("imxbayer1sthalf", "bayer1");
  bayer2 = gst_element_factory_make ("imxbayer2ndhalf", "bayer2");
  bayerq1 = gst_element_factory_make ("queue", "bayerq1");
  bayerq2 = gst_element_factory_make ("queue", "bayerq2");
  videoenc = gst_element_factory_make ("imxvpuenc_mpeg4", "videoenc");
  encq = gst_element_factory_make ("queue", "encq");
  mpegmux = gst_element_factory_make ("mpegtsmux", "mpegmux");
  sink = gst_element_factory_make ("filesink", "sink");
  
  
 
  if (!pipeline || !videosrc || !srcq || !bayer1 || !bayer2 || !bayerq1 || !bayerq2 || !videoenc
    || !encq || !mpegmux || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return FALSE;
  }

  g_object_set (G_OBJECT (sink),"location","/home/alarm/media/clip_1080p_1.mp4","sync",FALSE, NULL);
  g_object_set (G_OBJECT (videosrc),"capture-mode",2, "capture-format", 1,"fps-n",30,"queue-size",10, NULL);
  g_object_set (G_OBJECT (videoenc),"bitrate", 10000 , NULL);
  g_object_set (G_OBJECT (bayer1),"fbnum",2, "fbset", 1,"extbuf",1,"red",1.15,"green", 1.0,"blue",1.25,"chrom",140, NULL);
  g_object_set (G_OBJECT (bayer2),"fbnum",2, NULL);


 g_bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));

  // we add all elements into the pipeline /
  gst_bin_add_many (GST_BIN (pipeline), videosrc, srcq, bayer1, bayerq1, bayer2, bayerq2, videoenc, encq, mpegmux, sink, NULL);
  
  caps_bayer = gst_caps_new_simple ("video/x-bayer","width", G_TYPE_INT, 1920,"height", G_TYPE_INT, 1080,NULL);
  caps_raw = gst_caps_new_simple ("video/x-raw","format",G_TYPE_STRING, "I420","width", G_TYPE_INT, 1920,"height", G_TYPE_INT, 1080,NULL);
  
  
  
  
  if(!gst_element_link_filtered(videosrc,srcq, caps_bayer))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
 
 gst_element_link_many (srcq,bayer1,bayerq1, bayer2, NULL);
 
 if(!gst_element_link_filtered(bayer2,bayerq2, caps_raw))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 }  
  
  //gst_element_link_many (srcq, videoenc, encq, parse, rtp, sink, NULL);
  gst_element_link_many (bayerq2, videoenc, encq, mpegmux, sink, NULL);
 // Set the pipeline to "playing" state/
  g_print ("Streaming to port: %s\n", argv);
  
  //gst_element_set_state (pipeline, GST_STATE_PLAYING);
  return TRUE;
}



 static gboolean link_elements_with_filter (GstElement*element1, GstElement*element2)
  {
 gboolean link_ok;
 GstCaps*caps;
 caps = gst_caps_new_simple ("video/x-raw",
 "format", G_TYPE_STRING, "I420",
 "width", G_TYPE_INT, 960,
 "height", G_TYPE_INT, 720,
 "framerate", GST_TYPE_FRACTION, 30, 1,NULL);

 link_ok = gst_element_link_filtered (element1, element2, caps);
 gst_caps_unref (caps);
 if (!link_ok) {
  g_warning ("Failed to link element1 and element2!");
  }
 return link_ok;
}

//check with h264 

gboolean start_720p_flip(int *argc, char ***argv)
{

 const gchar* nano_str;

 guint major, minor, micro, nano;
 
 
//int *ptr2 = &argv;
 gst_init (argc, argv);

 gst_version (&major, &minor, &micro, &nano);

 if (nano == 1)
   nano_str = "(CVS)";
 else if (nano == 2)
   nano_str = "(Prerelease)";
 else
   nano_str = "";
   printf ("This program is linked against GStreamer %d.%d.%d %s\n",major, minor, micro, nano_str);

  /* Create gstreamer elements */
  pipeline = gst_pipeline_new ("video_testsrc");
  //videosrc = gst_element_factory_make ("videotestsrc", "videosrc");
  videosrc = gst_element_factory_make ("imxv4l2src", "videosrc");
  srcq = gst_element_factory_make ("queue", "srcq");
  //bayer = gst_element_factory_make ("imxbayer", "bayer");
  bayer1 = gst_element_factory_make ("imxbayer1sthalf", "bayer1");
  bayer2 = gst_element_factory_make ("imxbayer2ndhalf", "bayer2");
  bayerq1 = gst_element_factory_make ("queue", "bayerq1");
  bayerq2 = gst_element_factory_make ("queue", "bayerq2");
  flip = gst_element_factory_make ("imxipuvideotransform", "flip");
  flipq = gst_element_factory_make ("queue", "flipq");
  videoenc = gst_element_factory_make ("imxvpuenc_h264", "videoenc");
  encq = gst_element_factory_make ("queue", "encq");
  parse = gst_element_factory_make ("h264parse", "parse");
  mpegmux = gst_element_factory_make ("mpegtsmux", "mpegmux");
  sink = gst_element_factory_make ("filesink", "sink");
  
  
 
  if (!pipeline || !videosrc || !srcq || !bayer1 || !bayer2 || !bayerq1 || !bayerq2 || !flip || !flipq || !videoenc
    || !encq || !parse || !mpegmux || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return FALSE;
  }

  g_object_set (G_OBJECT (sink),"location","/home/alarm/media/clip_720p_1.mts","sync",FALSE, NULL);
  g_object_set (G_OBJECT (videosrc),"capture-mode",1, "capture-format", 1,"fps-n",30, NULL);
  g_object_set (G_OBJECT (flip),"output-rotation", 2, NULL);
  g_object_set (G_OBJECT (videoenc),"idr-interval", 16 ,"quant-param" ,20 , NULL);
  g_object_set (G_OBJECT (bayer1),"fbnum",2, "fbset", 1,"extbuf",1,"red",1.15,"green", 1.0,"blue",1.25,"chrom",140, NULL);
  g_object_set (G_OBJECT (bayer2),"fbnum",2, NULL);


 g_bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));

  /* we add all elements into the pipeline */
  gst_bin_add_many (GST_BIN (pipeline), videosrc, srcq, bayer1, bayerq1, bayer2, bayerq2, flip, flipq, videoenc, encq, parse, mpegmux, sink, NULL);
  
  caps_bayer = gst_caps_new_simple ("video/x-bayer","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  caps_raw = gst_caps_new_simple ("video/x-raw","format",G_TYPE_STRING, "I420","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  caps_ipu = gst_caps_new_simple ("video/x-raw","format",G_TYPE_STRING, "I420","width", G_TYPE_INT, 1280,"height", G_TYPE_INT, 720,NULL);
  
  
  
  if(!gst_element_link_filtered(videosrc,srcq, caps_bayer))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
 
 gst_element_link_many (srcq,bayer1,bayerq1, bayer2, NULL);
 
 if(!gst_element_link_filtered(bayer2,bayerq2, caps_raw))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
 
 gst_element_link_many (bayerq2,flip, NULL);
 
 if(!gst_element_link_filtered(flip,flipq, caps_ipu))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return FALSE;
 } 
 
  
  
  //gst_element_link_many (srcq, videoenc, encq, parse, rtp, sink, NULL);
  gst_element_link_many (flipq, videoenc, encq, parse, mpegmux, sink, NULL);
 /* Set the pipeline to "playing" state*/
  g_print ("Streaming to port: %s\n", argv);
  
  //gst_element_set_state (pipeline, GST_STATE_PLAYING);
  return TRUE;
}








