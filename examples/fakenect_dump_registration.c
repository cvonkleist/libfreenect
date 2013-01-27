/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 *
 */

// (cvk) this was stolen from
// https://github.com/mankoff/libfreenect.git
// more info: http://kenmankoff.com/2012/05/01/offline-registration-for-the-kinect

#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/time.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <libfreenect-registration.h>

#define DEPTH_MAX_METRIC_VALUE 10000
#define DEPTH_MAX_RAW_VALUE 2048
#define DEPTH_NO_MM_VALUE 0

#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480

#define REG_X_VAL_SCALE 256 // "fixed-point" precision for double -> int32_t conversion


// (cvk) this was stolen from
// https://github.com/mankoff/libfreenect.git
// more info: http://kenmankoff.com/2012/05/01/offline-registration-for-the-kinect
void dump_registration(char* registration_filename) {
  printf("Dumping Kinect registration to %s\n", registration_filename);

  freenect_context *f_ctx;
  if (freenect_init(&f_ctx, NULL) < 0) {
	printf("freenect_init() failed\n");
	exit(0);
  }

  freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(f_ctx, 
          (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

  freenect_device *f_dev;
  int user_device_number = 0;
  if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
	printf("Could not open device\n");
	exit(0);
  }

  freenect_registration reg;
  reg = freenect_copy_registration( f_dev );

  FILE* fp = fopen(registration_filename, "w");
  if (!fp) {
	printf("Error: Cannot open file '%s'\n", registration_filename);
	exit(1);
  }

  // from freenect_copy_registration in registration.c
  /*
  reg.reg_info = reg->registration.reg_info;
  reg.reg_pad_info = reg->registration.reg_pad_info;
  reg.zero_plane_info = reg->registration.zero_plane_info;
  reg.const_shift = reg->registration.const_shift;
  reg.raw_to_mm_shift    = (uint16_t*)malloc( sizeof(uint16_t) * DEPTH_MAX_RAW_VALUE );
  reg.depth_to_rgb_shift = (int32_t*)malloc( sizeof( int32_t) * DEPTH_MAX_METRIC_VALUE );
  reg.registration_table = (int32_t (*)[2])malloc( sizeof( int32_t) * DEPTH_X_RES * DEPTH_Y_RES * 2 );
  */

  /* Should be ~2.4 MB
  printf( "\n\n%d\n\n", sizeof(reg)+
		  sizeof(uint16_t)*DEPTH_MAX_RAW_VALUE + 
		  sizeof(int32_t)*DEPTH_MAX_METRIC_VALUE + 
		  sizeof(int32_t)*DEPTH_X_RES*DEPTH_Y_RES*2 );
  */

  // write out first four fields
  fwrite( &reg.reg_info, sizeof(reg.reg_info), 1, fp );
  fwrite( &reg.reg_pad_info, sizeof(reg.reg_pad_info), 1, fp);
  fwrite( &reg.zero_plane_info, sizeof(reg.zero_plane_info), 1, fp);
  fwrite( &reg.const_shift, sizeof(reg.const_shift), 1, fp);

  //printf( "\n\n%d\n\n", sizeof(uint16_t)*DEPTH_MAX_RAW_VALUE ); // 4096
  fwrite( (&reg)->raw_to_mm_shift, sizeof(uint16_t), DEPTH_MAX_RAW_VALUE, fp );

  //printf( "\n\n%d\n\n", sizeof(int32_t)*DEPTH_MAX_METRIC_VALUE ); // 4000
  fwrite( (&reg)->depth_to_rgb_shift, sizeof(int32_t), DEPTH_MAX_METRIC_VALUE, fp );

  //printf( "\n\n%d\n\n", sizeof(int32_t)*DEPTH_X_RES*DEPTH_Y_RES*2 ); // 2457600
  fwrite( (&reg)->registration_table, sizeof(int32_t), DEPTH_X_RES*DEPTH_Y_RES*2, fp );

  fclose(fp);
}

void usage()
{
  printf("\n"
		 "fakenect registration dumper for kinect\n"
		 "\n"
		 "usage:\n"
		 "  fakenect_dump_registration <regfile>\n"
		 "\n"
		 "dumps the connected kinect device's registration paramters into <regfile>.\n"
		 );
  exit(0);
}

int main(int argc, char **argv)
{
  char *registration_filename;
  if (argc != 2)
	usage();

  registration_filename = argv[1];

  dump_registration(registration_filename);

  return 0;
}
