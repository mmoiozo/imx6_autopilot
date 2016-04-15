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
#include "gst_video.h"


gboolean bus_call (GstBus *bus, GstMessage *msg)//static
{
  switch (GST_MESSAGE_TYPE (msg)) {
 
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      //loop_status = 0;
      break;
 
    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;
 
      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);
 
      g_printerr ("Error: %s\n", error->message);
      g_error_free (error);
 
      //loop_status = 0;
      break;
    }
    default:
      break;
  }
 
  return TRUE;
}

 static gboolean
 link_elements_with_filter (GstElement*element1, GstElement*element2)
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

void initialize_pipeline(int   argc, char **argv[])
{

 const gchar* nano_str;

 guint major, minor, micro, nano;
 
 
int *ptr2 = &argv;
 gst_init (&argc, &ptr2);

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
  videosrc = gst_element_factory_make ("videotestsrc", "videosrc");
  srcq = gst_element_factory_make ("queue", "srcq");
  videoenc = gst_element_factory_make ("imxvpuenc_h264", "videoenc");
  encq = gst_element_factory_make ("queue", "encq");
  parse = gst_element_factory_make ("h264parse", "parse");
  rtp = gst_element_factory_make ("rtph264pay", "rtp");
  sink = gst_element_factory_make ("udpsink", "sink");
 
  if (!pipeline || !videosrc || !srcq || !videoenc
    || !encq || !parse || !rtp || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  g_object_set (G_OBJECT (sink), "port", 5000,"host", "192.168.2.7", NULL);
  g_object_set (G_OBJECT (videosrc),"pattern", "smpte", "horizontal-speed", 1, NULL);
  g_object_set (G_OBJECT (videoenc),"idr-interval", 16 ,"quant-param" ,20 , NULL);


 g_bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));

  /* we add all elements into the pipeline */
  gst_bin_add_many (GST_BIN (pipeline), videosrc, srcq, videoenc, encq, parse, rtp, sink, NULL);
 
  /* we link the elements together */
  if(!link_elements_with_filter(videosrc, srcq))
	{
        g_printerr ("Videosrc and srcq could not be linked.\n");
        return -1;
	}
  gst_element_link_many (srcq, videoenc, encq, parse, rtp, sink, NULL);
 /* Set the pipeline to "playing" state*/
  g_print ("Streaming to port: %s\n", argv[1]);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);
}

void initialize_240p(int   argc, char **argv[])
{

 const gchar* nano_str;

 guint major, minor, micro, nano;
 
 
int *ptr2 = &argv;
 gst_init (&argc, &ptr2);

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
  bayer = gst_element_factory_make ("imxbayer", "bayer");
  bayerq = gst_element_factory_make ("queue", "bayerq");
  videotransform = gst_element_factory_make ("imxipuvideotransform", "videotransform");
  videoenc = gst_element_factory_make ("imxvpuenc_h264", "videoenc");
  encq = gst_element_factory_make ("queue", "encq");
  parse = gst_element_factory_make ("h264parse", "parse");
  rtp = gst_element_factory_make ("rtph264pay", "rtp");
  sink = gst_element_factory_make ("udpsink", "sink");
  
  
 
  if (!pipeline || !videosrc || !srcq || !videoenc
    || !encq || !parse || !rtp || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  g_object_set (G_OBJECT (sink), "port", 5000,"host", "192.168.2.7", NULL);
  g_object_set (G_OBJECT (videosrc),"pattern", "smpte", "horizontal-speed", 1, NULL);
  g_object_set (G_OBJECT (videoenc),"idr-interval", 16 ,"quant-param" ,20 , NULL);


 g_bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));

  /* we add all elements into the pipeline */
  gst_bin_add_many (GST_BIN (pipeline), videosrc, srcq, bayer, bayerq, videoenc, encq, parse, rtp, sink, NULL);
  
  caps_bayer = gst_caps_new_simple ("video/x-bayer",
 "width", G_TYPE_INT, 1280,
 "height", G_TYPE_INT, 720,NULL);
  
  
  if(!gst_element_link_filtered(videosrc,srcq, caps_bayer))
 {
  gst_object_unref (pipeline);
  g_critical ("Unable to link csp to tee. check your caps.");
  return 0;
 } 
 
 
 
  /* we link the elements together */
  /*
  if(!link_elements_with_filter(videosrc, srcq))
	{
        g_printerr ("Videosrc and srcq could not be linked.\n");
        return -1;
	}
	*/
  
  
  gst_element_link_many (srcq, videoenc, encq, parse, rtp, sink, NULL);
 /* Set the pipeline to "playing" state*/
  g_print ("Streaming to port: %s\n", argv[1]);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);
}
