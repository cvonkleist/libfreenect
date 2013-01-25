/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 Brandyn White (bwhite@dappervision.com)
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
 */

#include <libfreenect.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include "../src/freenect_internal.h"
#include <libfreenect-registration.h>

#define GRAVITY 9.80665

// (cvk) this is added stuff to support playback of raw depth frames in
// REGISTERED mode.  globals are copied from registration.c.
#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480
#define DEPTH_MAX_METRIC_VALUE FREENECT_DEPTH_MM_MAX_VALUE
#define DEPTH_NO_MM_VALUE      FREENECT_DEPTH_MM_NO_VALUE
#define DEPTH_MAX_RAW_VALUE    FREENECT_DEPTH_RAW_MAX_VALUE
#define DEPTH_NO_RAW_VALUE     FREENECT_DEPTH_RAW_NO_VALUE
#define DEPTH_MIRROR_X 0
#define REG_X_VAL_SCALE 256 // "fixed-point" precision for double -> int32_t conversion
static void *depth_buffer_registered = NULL;
freenect_registration registration;
freenect_depth_format chosen_fmt;

// The dev and ctx are just faked with these numbers

static freenect_device *fake_dev = (freenect_device *)1234;
static freenect_context *fake_ctx = (freenect_context *)5678;
static freenect_depth_cb cur_depth_cb = NULL;
static freenect_video_cb cur_rgb_cb = NULL;
static char *input_path = NULL;
static FILE *index_fp = NULL;
static freenect_raw_tilt_state state = {};
static int already_warned = 0;
static double playback_prev_time = 0.;
static double record_prev_time = 0.;
static void *depth_buffer = NULL;
static void *rgb_buffer = NULL;
static int depth_running = 0;
static int rgb_running = 0;
static void *user_ptr = NULL;

static void sleep_highres(double tm)
{
#ifdef _WIN32
	int msec = floor(tm * 1000);
	if (msec > 0) {
		Sleep(msec);
	}
#else
	int sec = floor(tm);
	int usec = (tm - sec) * 1000000;
	if (tm > 0) {
		sleep(sec);
		usleep(usec);
	}
#endif
}

static double get_time()
{
#ifdef _WIN32
	SYSTEMTIME st;
	GetSystemTime(&st);
	FILETIME ft;
	SystemTimeToFileTime(&st, &ft);
	ULARGE_INTEGER li;
	li.LowPart = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;
	// FILETIME is given as a 64-bit value for the number of 100-nanosecond
	// intervals that have passed since Jan 1, 1601 (UTC).  The difference between that
	// epoch and the POSIX epoch (Jan 1, 1970) is 116444736000000000 such ticks.
	uint64_t total_usecs = (li.QuadPart - 116444736000000000L) / 10L;
	return (total_usecs / 1000000.);
#else
	struct timeval cur;
	gettimeofday(&cur, NULL);
	return cur.tv_sec + cur.tv_usec / 1000000.;
#endif
}

static char *one_line(FILE *fp)
{
	int pos = 0;
	char *out = NULL;
	char c;
	while ((c = fgetc(fp))) {
		if (c == '\n' || c == EOF)
			break;
		out = realloc(out, pos + 1);
		out[pos++] = c;
	}
	if (out) {
		out = realloc(out, pos + 1);
		out[pos] = '\0';
	}
	return out;
}

static int get_data_size(FILE *fp)
{
	int orig = ftell(fp);
	fseek(fp, 0L, SEEK_END);
	int out = ftell(fp);
	fseek(fp, orig, SEEK_SET);
	return out;
}

static int parse_line(char *type, double *cur_time, unsigned int *timestamp, unsigned int *data_size, char **data)
{
	char *line = one_line(index_fp);
	if (!line) {
		printf("Warning: No more lines in [%s]\n", input_path);
		return -1;
	}
	int file_path_size = strlen(input_path) + strlen(line) + 50;
	char *file_path = malloc(file_path_size);
	snprintf(file_path, file_path_size, "%s/%s", input_path, line);
	// Open file
	FILE *cur_fp = fopen(file_path, "rb");
	if (!cur_fp) {
		printf("Error: Cannot open file [%s]\n", file_path);
		exit(1);
	}
	// Parse data from file name
	*data_size = get_data_size(cur_fp);
	sscanf(line, "%c-%lf-%u-%*s", type, cur_time, timestamp);
	*data = malloc(*data_size);
	if (fread(*data, *data_size, 1, cur_fp) != 1) {
		printf("Error: Couldn't read entire file.\n");
		return -1;
	}
	fclose(cur_fp);
	free(line);
	free(file_path);
	return 0;
}

static void open_index()
{
	input_path = getenv("FAKENECT_PATH");
	if (!input_path) {
		printf("Error: Environmental variable FAKENECT_PATH is not set.  Set it to a path that was created using the 'record' utility.\n");
		exit(1);
	}
	int index_path_size = strlen(input_path) + 50;
	char *index_path = malloc(index_path_size);
	snprintf(index_path, index_path_size, "%s/INDEX.txt", input_path);
	index_fp = fopen(index_path, "rb");
	if (!index_fp) {
		printf("Error: Cannot open file [%s]\n", index_path);
		exit(1);
	}
	free(index_path);
}

static char *skip_line(char *str)
{
	char *out = strchr(str, '\n');
	if (!out) {
		printf("Error: PGM/PPM has incorrect formatting, expected a header on one line followed by a newline\n");
		exit(1);
	}
	return out + 1;
}


// (cvk) this was stolen from
// https://github.com/mankoff/libfreenect.git
// more info: http://kenmankoff.com/2012/05/01/offline-registration-for-the-kinect
freenect_registration load_registration(char* regfile) 
{

  FILE *fp = NULL;
  /* load the regdump file */
  fp = fopen(regfile, "r");
  if (!fp) {
	perror(regfile);
	exit(1);
  }
  
  freenect_registration reg;
  // allocate memory and set up pointers
  // (from freenect_copy_registration in registration.c)
  reg.raw_to_mm_shift    = (uint16_t*)malloc( sizeof(uint16_t) * DEPTH_MAX_RAW_VALUE );
  reg.depth_to_rgb_shift = (int32_t*)malloc( sizeof( int32_t) * DEPTH_MAX_METRIC_VALUE );
  reg.registration_table = (int32_t (*)[2])malloc( sizeof( int32_t) * DEPTH_X_RES * DEPTH_Y_RES * 2 );
  // load (inverse of kinect_regdump)
  fread( &reg.reg_info, sizeof(reg.reg_info), 1, fp );
  fread( &reg.reg_pad_info, sizeof(reg.reg_pad_info), 1, fp);
  fread( &reg.zero_plane_info, sizeof(reg.zero_plane_info), 1, fp);
  fread( &reg.const_shift, sizeof(reg.const_shift), 1, fp);
  fread( reg.raw_to_mm_shift, sizeof(uint16_t), DEPTH_MAX_RAW_VALUE, fp );
  fread( reg.depth_to_rgb_shift, sizeof(int32_t), DEPTH_MAX_METRIC_VALUE, fp );
  fread( reg.registration_table, sizeof(int32_t), DEPTH_X_RES*DEPTH_Y_RES*2, fp );
  fclose(fp);

  return reg;
}

// (cvk) this was stolen from registration.c, but i modified it to work on
// normal depth data instead of packed depth data.
//
// apply registration data to a single packed frame
int freenect_apply_registration(freenect_device* dev, uint16_t* input, uint16_t* output_mm)
{
	// set output buffer to zero using pointer-sized memory access (~ 30-40% faster than memset)
	size_t i, *wipe = (size_t*)output_mm;
	for (i = 0; i < DEPTH_X_RES * DEPTH_Y_RES * sizeof(uint16_t) / sizeof(size_t); i++) wipe[i] = DEPTH_NO_MM_VALUE;

	freenect_registration *reg = &registration;

	uint32_t target_offset = DEPTH_Y_RES * reg->reg_pad_info.start_lines;
	uint32_t x,y;

	for (y = 0; y < DEPTH_Y_RES; y++) {
		for (x = 0; x < DEPTH_X_RES; x++) {

			/*// get 8 pixels from the packed frame*/
			/*if (source_index == 8) {*/
				/*// (cvk) unpack_8_pixels( input_packed, unpack );*/
				/*source_index = 0;*/
			/*}*/

			// get the value at the current depth pixel, convert to millimeters
			uint16_t metric_depth = reg->raw_to_mm_shift[ input[y * DEPTH_X_RES + x] ];
			if(x == 320) {
			//	printf("x = %d / y = %d / input[%d] = %d / metric_depth = %d\n", x, y, y * DEPTH_X_RES + x, input[y * DEPTH_X_RES + x], metric_depth);
			}


			// so long as the current pixel has a depth value
			if (metric_depth == DEPTH_NO_MM_VALUE) continue;
			if (metric_depth >= DEPTH_MAX_METRIC_VALUE) continue;

			// calculate the new x and y location for that pixel
			// using registration_table for the basic rectification
			// and depth_to_rgb_shift for determining the x shift
			uint32_t reg_index = DEPTH_MIRROR_X ? ((y + 1) * DEPTH_X_RES - x - 1) : (y * DEPTH_X_RES + x);
			uint32_t nx = (reg->registration_table[reg_index][0] + reg->depth_to_rgb_shift[metric_depth]) / REG_X_VAL_SCALE;
			uint32_t ny =  reg->registration_table[reg_index][1];

			// ignore anything outside the image bounds
			if (nx >= DEPTH_X_RES) continue;
			// convert nx, ny to an index in the depth image array
			uint32_t target_index = (DEPTH_MIRROR_X ? ((ny + 1) * DEPTH_X_RES - nx - 1) : (ny * DEPTH_X_RES + nx)) - target_offset;

			// get the current value at the new location
			uint16_t current_depth = output_mm[target_index];

			// make sure the new location is empty, or the new value is closer
			if ((current_depth == DEPTH_NO_MM_VALUE) || (current_depth > metric_depth)) {
				output_mm[target_index] = metric_depth; // always save depth at current location


				#ifdef DENSE_REGISTRATION
					// if we're not on the first row, or the first column
					if ((nx > 0) && (ny > 0)) {
						output_mm[target_index - DEPTH_X_RES    ] = metric_depth; // save depth at (x,y-1)
						output_mm[target_index - DEPTH_X_RES - 1] = metric_depth; // save depth at (x-1,y-1)
						output_mm[target_index               - 1] = metric_depth; // save depth at (x-1,y)
					} else if (ny > 0) {
						output_mm[target_index - DEPTH_X_RES] = metric_depth; // save depth at (x,y-1)
					} else if (nx > 0) {
						output_mm[target_index - 1] = metric_depth; // save depth at (x-1,y)
					}
				#endif
			}
		}
	}
	return 0;
}

int freenect_process_events(freenect_context *ctx)
{
	/* This is where the magic happens. We read 1 update from the index
	   per call, so this needs to be called in a loop like usual.  If the
	   index line is a Depth/RGB image the provided callback is called.  If
	   the index line is accelerometer data, then it is used to update our
	   internal state.  If you query for the accelerometer data you get the
	   last sensor reading that we have.  The time delays are compensated as
	   best as we can to match those from the original data and current run
	   conditions (e.g., if it takes longer to run this code then we wait less).
	 */
	if (!index_fp)
		open_index();
	char type;
	double record_cur_time;
	unsigned int timestamp, data_size;
	char *data = NULL;
	if (parse_line(&type, &record_cur_time, &timestamp, &data_size, &data))
		return -1;
	// Sleep an amount that compensates for the original and current delays
	// playback_ is w.r.t. the current time
	// record_ is w.r.t. the original time period during the recording
	if (record_prev_time != 0. && playback_prev_time != 0.)
		sleep_highres((record_cur_time - record_prev_time) - (get_time() - playback_prev_time));
	record_prev_time = record_cur_time;
	switch (type) {
		case 'd':
			if (cur_depth_cb && depth_running) {
				void *cur_depth = skip_line(data);
				if (depth_buffer) {
					memcpy(depth_buffer, cur_depth, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT).bytes);
					cur_depth = depth_buffer;
				}

				// (cvk) originally, fakenect could only play
				// back captured kinect depth data in
				// FREENECT_DEPTH_11BIT mode, which is set
				// using a call like this:
				//
				//	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
				//
				// in this modified version of fakenect, a call
				// to our modified freenect_find_depth_mode()
				// stores the requested video mode in
				// chosen_fmt. then, when we are reading a
				// depth frame from an input file, we call
				// freenect_apply_registration() to align the
				// depth image to the rgb image if chosen_fmt
				// is FREENECT_DEPTH_REGISTERED.
				if(chosen_fmt == FREENECT_DEPTH_REGISTERED) {
					freenect_apply_registration(fake_dev, cur_depth, depth_buffer_registered);
					cur_depth = depth_buffer_registered;
				}
				cur_depth_cb(fake_dev, cur_depth, timestamp);
			}
			break;
		case 'r':
			if (cur_rgb_cb && rgb_running) {
				void *cur_rgb = skip_line(data);
				if (rgb_buffer) {
					memcpy(rgb_buffer, cur_rgb, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes);
					cur_rgb = rgb_buffer;
				}
				cur_rgb_cb(fake_dev, cur_rgb, timestamp);
			}
			break;
		case 'a':
			if (data_size == sizeof(state)) {
				memcpy(&state, data, sizeof(state));
			} else if (!already_warned) {
				already_warned = 1;
				printf("\n\nWarning: Accelerometer data has an unexpected"
				       " size [%u] instead of [%u].  The acceleration "
				       "and tilt data will be substituted for dummy "
				       "values.  This data was probably made with an "
				       "older version of record (the upstream interface "
				       "changed).\n\n",
				       data_size, (unsigned int)sizeof state);
			}
			break;
	}
	free(data);
	playback_prev_time = get_time();
	return 0;
}

double freenect_get_tilt_degs(freenect_raw_tilt_state *state)
{
	// NOTE: This is duped from tilt.c, this is the only function we need from there
	return ((double)state->tilt_angle) / 2.;
}

freenect_raw_tilt_state* freenect_get_tilt_state(freenect_device *dev)
{
	return &state;
}

void freenect_get_mks_accel(freenect_raw_tilt_state *state, double* x, double* y, double* z)
{
	//the documentation for the accelerometer (http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf)
	//states there are 819 counts/g
	*x = (double)state->accelerometer_x/FREENECT_COUNTS_PER_G*GRAVITY;
	*y = (double)state->accelerometer_y/FREENECT_COUNTS_PER_G*GRAVITY;
	*z = (double)state->accelerometer_z/FREENECT_COUNTS_PER_G*GRAVITY;
}

void freenect_set_depth_callback(freenect_device *dev, freenect_depth_cb cb)
{
	cur_depth_cb = cb;
}

void freenect_set_video_callback(freenect_device *dev, freenect_video_cb cb)
{
	cur_rgb_cb = cb;
}

int freenect_set_video_mode(freenect_device* dev, const freenect_frame_mode mode)
{
        // Always say it was successful but continue to pass through the
        // underlying data.  Would be better to check for conflict.
        return 0;
}

int freenect_set_depth_mode(freenect_device* dev, const freenect_frame_mode mode)
{
        // Always say it was successful but continue to pass through the
        // underlying data.  Would be better to check for conflict.
        return 0;
}

freenect_frame_mode freenect_find_video_mode(freenect_resolution res, freenect_video_format fmt) {
    assert(FREENECT_RESOLUTION_MEDIUM == res);
    assert(FREENECT_VIDEO_RGB == fmt);
    // NOTE: This will leave uninitialized values if new fields are added.
    // To update this line run the "record" program, look at the top output
    freenect_frame_mode out = {256, 1, {0}, 921600, 640, 480, 24, 0, 30, 1};
    return out;
}

freenect_frame_mode freenect_find_depth_mode(freenect_resolution res, freenect_depth_format fmt) {
    assert(FREENECT_RESOLUTION_MEDIUM == res);

    // (cvk) originally fakenect only allowed playback in 11BIT mode, but now
    // it will allow the recorded 11BIT frames to be played back in REGISTERED
    // mode, so this assertion was changed to allow either one.
    assert(FREENECT_DEPTH_11BIT == fmt || FREENECT_DEPTH_REGISTERED == fmt);
    chosen_fmt = fmt;

    if(chosen_fmt == FREENECT_DEPTH_REGISTERED)
	    printf("* * * * * fakenect: playing back 11BIT data in REGISTERED mode * * * * *\n");

    // NOTE: This will leave uninitialized values if new fields are added.
    // To update this line run the "record" program, look at the top output
    freenect_frame_mode out = {256, 1, {0}, 614400, 640, 480, 11, 5, 30, 1};
    return out;
}

int freenect_num_devices(freenect_context *ctx)
{
	// Always 1 device
	return 1;
}

int freenect_open_device(freenect_context *ctx, freenect_device **dev, int index)
{
	// Set it to some number to allow for NULL checks
	*dev = fake_dev;
	return 0;
}

int freenect_init(freenect_context **ctx, freenect_usb_context *usb_ctx)
{
	*ctx = fake_ctx;
	// (cvk) initialize space for the raw depth image to be rebuilt into a
	// new buffer, where it will be aligned to the rgb image.
	//
	// TODO: free this memory on exit, lol
	depth_buffer_registered = malloc(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT).bytes);

	// (cvk) load up the saved kinect registration data from a dump file. for now, this must be dumped with the kinect_register tool.
	//
	// more info:
	//
	// http://kenmankoff.com/2012/05/01/offline-registration-for-the-kinect
	registration = load_registration("/tmp/fakenect_registration_data");
	return 0;
}

void freenect_select_subdevices(freenect_context *ctx, freenect_device_flags subdevs) {
	// Ideally, we'd actually check for MOTOR and CAMERA and AUDIO, but for now
	// we just ignore them and provide all captured data.
}

int freenect_set_depth_buffer(freenect_device *dev, void *buf)
{
	depth_buffer = buf;
	return 0;
}

int freenect_set_video_buffer(freenect_device *dev, void *buf)
{
	rgb_buffer = buf;
	return 0;
}

void freenect_set_user(freenect_device *dev, void *user)
{
	user_ptr = user;
}

void *freenect_get_user(freenect_device *dev)
{
	return user_ptr;
}

int freenect_start_depth(freenect_device *dev)
{
	depth_running = 1;
	return 0;
}

int freenect_start_video(freenect_device *dev)
{
	rgb_running = 1;
	return 0;
}

int freenect_stop_depth(freenect_device *dev)
{
	depth_running = 0;
	return 0;
}

int freenect_stop_video(freenect_device *dev)
{
	rgb_running = 0;
	return 0;
}

int freenect_set_video_format(freenect_device *dev, freenect_video_format fmt)
{
	assert(fmt == FREENECT_VIDEO_RGB);
	return 0;
}
int freenect_set_depth_format(freenect_device *dev, freenect_depth_format fmt)
{
	assert(fmt == FREENECT_DEPTH_11BIT);
	return 0;
}

void freenect_set_log_callback(freenect_context *ctx, freenect_log_cb cb) {}
void freenect_set_log_level(freenect_context *ctx, freenect_loglevel level) {}
int freenect_shutdown(freenect_context *ctx)
{
	return 0;
}
int freenect_close_device(freenect_device *dev)
{
	return 0;
}
int freenect_set_tilt_degs(freenect_device *dev, double angle)
{
	return 0;
}
int freenect_set_led(freenect_device *dev, freenect_led_options option)
{
	return 0;
}
int freenect_update_tilt_state(freenect_device *dev)
{
	return 0;
}
