#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/v4l2-subdev.h>
#include "hailo_stitch.hpp"

#define MAX_NUM_OF_PLANES 3
#define DEFAULT_HEF_PATH "/usr/bin/hdr_4k_3_exposures.hef"
#define HEF_PATH_NAME_LENGTH PATH_MAX
#define VIDEO_WAIT_FOR_STREAM_START _IO('D', BASE_VIDIOC_PRIVATE + 3)
#define ARG_NUM 7
#define DOL_NUM_EXP_RAW 3
#define DOL_NUM_EXP_STITCHED 1
#define INPUT_WIDTH_4K 3840
#define INPUT_HEIGHT_4K 2160
#define VIDEO_RAW_CAPTURE_BUF_COUNT 3
#define VIDEO_ISP_IN_BUF_COUNT 2
#define VIDEO_YUV_PATH "/dev/video0"

struct buffer {
	int num_planes;
	int sizes[MAX_NUM_OF_PLANES];
	void *planes[MAX_NUM_OF_PLANES];
	struct v4l2_buffer v4l2_buf;
	int first_use;
};

struct buffer *buffers[2];
static unsigned int n_buffers[2];
static volatile int async_finished = 0;
static HailortAsyncStitching* stitcher;
static int raw_capture_fd = -1;
static int isp_in_fd = -1;
static int isp_in_buffers_all_used = 0;

enum {
	VIDEO_RAW_CAPTURE,
	VIDEO_ISP_IN,
};

static int xioctl(int fh, uint32_t request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static int open_device(const char *dev_name)
{
	struct stat st;
	static int fd = 0;
	int mode = O_RDWR;
	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name,
			errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		exit(EXIT_FAILURE);
	}
	
	fd = open(dev_name, mode, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,
			strerror(errno));
		exit(EXIT_FAILURE);
	}
	return fd;
}

int path_to_fd(int path){
	if(path == VIDEO_RAW_CAPTURE)
		return raw_capture_fd;
	if(path == VIDEO_ISP_IN)
		return isp_in_fd;
	return -1;
}

v4l2_buf_type path_to_type(int path){
	return path==VIDEO_RAW_CAPTURE ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
}

unsigned int path_to_buf_count(int path){
	if(path == VIDEO_RAW_CAPTURE)
		return VIDEO_RAW_CAPTURE_BUF_COUNT;
	return VIDEO_ISP_IN_BUF_COUNT;
}

int set_format(int path, int width, int height, int pix_fmt, int num_planes)
{
	struct v4l2_format fmt;
	fmt.type = path_to_type(path);
	fmt.fmt.pix_mp.width = width;
	fmt.fmt.pix_mp.height = height;
	fmt.fmt.pix_mp.pixelformat = pix_fmt;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.num_planes = num_planes;
	if (-1 == xioctl(path_to_fd(path), VIDIOC_S_FMT, &fmt)) {
		return -EINVAL;
	}
	return 0;
}

int start_stream(int path)
{
	enum v4l2_buf_type type;

	type = path_to_type(path);
	if (-1 == xioctl(path_to_fd(path), VIDIOC_STREAMON, &type))
		exit(-1);

	return 0;
}

static int set_raw_capture_fps(){
	struct v4l2_streamparm parm = {0};
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = 20;

	return xioctl(path_to_fd(VIDEO_RAW_CAPTURE), VIDIOC_S_PARM, &parm);
}

static void stop_stream(int path)
{
	enum v4l2_buf_type type;

	type = path_to_type(path);
	if (-1 == xioctl(path_to_fd(path), VIDIOC_STREAMOFF, &type))
		exit(-1);
}

static int queue_buffer(int path,int index)
{
	if (path < 0 || path >= (int)n_buffers[path]) {
		return -EINVAL;
	}

	return xioctl(path_to_fd(path), VIDIOC_QBUF, &buffers[path][index].v4l2_buf);
}

static int queue_buffers(int path)
{
	unsigned int i = 0;
	int ret;
	for (i = 0; i < n_buffers[path]; ++i) {
		ret = queue_buffer(path, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int init_buffers(int path, int num_planes)
{
	struct v4l2_requestbuffers req;
	int type;
	memset(&req, 0, sizeof(struct v4l2_requestbuffers));

	req.count = path_to_buf_count(path);
	type = path_to_type(path);
	req.type = type;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(path_to_fd(path), VIDIOC_REQBUFS, &req)) {
		return -EINVAL;
	}
	if (req.count < path_to_buf_count(path)) {
		return -ENOMEM;
	}

	buffers[path] = (buffer*)calloc(req.count, sizeof(*buffers[path]));

	if (!buffers[path]) {
		printf("cant allocate memory!\n");
		return -ENOMEM;
	}

	for (n_buffers[path] = 0; n_buffers[path] < req.count; ++n_buffers[path]) {
		struct v4l2_buffer buf;
		unsigned int plane;

		memset(&buf, 0, sizeof(struct v4l2_buffer));

		buf.type = type;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers[path];
		buf.length = num_planes;
		buf.m.planes = (v4l2_plane*)malloc(num_planes *
				      sizeof(struct v4l2_plane));
		memset(buf.m.planes, 0,
		       num_planes * sizeof(struct v4l2_plane));

		if (-1 == xioctl(path_to_fd(path), VIDIOC_QUERYBUF, &buf)){
			printf("querybuf failed!\n");
			return errno;
		}

		buffers[path][n_buffers[path]].num_planes = buf.length;
		for (plane = 0; plane < buf.length; ++plane) {
			buffers[path][n_buffers[path]].sizes[plane] =
				buf.m.planes[plane].length;
			buffers[path][n_buffers[path]].planes[plane] =
				mmap(NULL /* start anywhere */,
				     buf.m.planes[plane].length,
				     PROT_READ | PROT_WRITE /* required */,
				     MAP_SHARED /* recommended */, path_to_fd(path),
				     buf.m.planes[plane].m.mem_offset);

			if (MAP_FAILED == buffers[path][n_buffers[path]].planes[plane]){
				printf("mmap failed!\n");
				return -ENOMEM;
			}
		}
		buffers[path][n_buffers[path]].first_use = 0;
		memcpy(&buffers[path][n_buffers[path]].v4l2_buf, &buf,
		       sizeof(struct v4l2_buffer));
	}
	return 0;
}

static void free_buffers(int path)
{
	unsigned int frame;
	unsigned int plane;
	for (frame = 0; frame < n_buffers[path]; ++frame) {
		free(buffers[path][frame].v4l2_buf.m.planes);
		for (plane = 0; plane < buffers[path][frame].v4l2_buf.length; ++plane)
			munmap(buffers[path][frame].planes[plane],
			       buffers[path][frame].sizes[plane]);
	}

	free(buffers[path]);
}

static int find_first_non_used_buffer(int path){
	for(unsigned int index = 0; index < n_buffers[path]; ++index){
		if(!buffers[path][index].first_use)
			return index;
	}

	return -1;
}

static int read_frame(int path, int num_planes)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[MAX_NUM_OF_PLANES];
	memset(&buf, 0, sizeof(struct v4l2_buffer));
	
	if(path == VIDEO_ISP_IN && !isp_in_buffers_all_used){
		int index = find_first_non_used_buffer(VIDEO_ISP_IN);
		if(index >= 0){
			buffers[VIDEO_ISP_IN][index].first_use = 1;
			return index;
		}
		isp_in_buffers_all_used = 1;
	}

	buf.type = path_to_type(path);
	buf.memory = V4L2_MEMORY_MMAP;
	buf.length = num_planes;
	buf.m.planes = planes;
	memset(buf.m.planes, 0, MAX_NUM_OF_PLANES * sizeof(struct v4l2_plane));
	if (-1 == xioctl(path_to_fd(path), VIDIOC_DQBUF, &buf)) {
		return -1;
	}

	if (buf.index >= n_buffers[path]) {
		return -1;
	}

	return buf.index;
}


void dequeue_all_buffers(int path){
	unsigned int index2 = 0;
	fd_set read_fds;
	struct timeval timeout;
	
	memset(&timeout, 0, sizeof(timeout));
	FD_ZERO(&read_fds);
	FD_SET(raw_capture_fd, &read_fds);

	timeout.tv_sec = 0;       // 0 seconds
	timeout.tv_usec = 1000;   // 1000 microseconds = 1 millisecond

	while(index2 >= 0){
		if(select(raw_capture_fd + 1, &read_fds, nullptr, nullptr, &timeout) == 0)
			return;

		index2 = read_frame(VIDEO_RAW_CAPTURE, DOL_NUM_EXP_RAW);
	}
}

void wait_for_yuv_stream_start(){
	int fd_video_yuv = open(VIDEO_YUV_PATH, O_RDWR);
	if(fd_video_yuv < 0){
		printf("cant open yuv output video device\n");
		return;
	}

	ioctl(fd_video_yuv, VIDEO_WAIT_FOR_STREAM_START);
	close(fd_video_yuv);
}

static void mcm_loop()
{
	int index2 = 0;
	int index3;

	wait_for_yuv_stream_start();
	dequeue_all_buffers(VIDEO_RAW_CAPTURE);
	queue_buffers(VIDEO_RAW_CAPTURE);
	while(true){
		index2 = read_frame(VIDEO_RAW_CAPTURE, DOL_NUM_EXP_RAW);
		index3 = read_frame(VIDEO_ISP_IN, DOL_NUM_EXP_STITCHED);

		stitcher->process(buffers[VIDEO_RAW_CAPTURE][index2].planes, buffers[VIDEO_ISP_IN][index3].planes[0]);

		while(!async_finished){
			continue;
		}
		queue_buffer(VIDEO_ISP_IN, index3);
		queue_buffer(VIDEO_RAW_CAPTURE, index2);

		async_finished = 0;
	}
}

void on_infer(void* ptr){
	async_finished = 1;
}

int main(int argc, char *argv[])
{
	int ret;
	ret = -EINVAL;

	printf("Starting Hailo15 HDR manager\n");
	stitcher = new HailortAsyncStitching(on_infer);
	stitcher->init(std::string(DEFAULT_HEF_PATH), std::string("0"), 1,1000, DOL_NUM_EXP_RAW);
	raw_capture_fd = open_device("/dev/video2");
	if (raw_capture_fd < 0) {
		printf("unable to open raw capture video device\n");
		exit(-1);
	}

	isp_in_fd = open_device("/dev/video3");
	if (isp_in_fd < 0) {
		ret = -1;
		printf("unable to open isp in video device\n");
		goto err_video3;
	}

	ret = set_format(VIDEO_RAW_CAPTURE, INPUT_WIDTH_4K, INPUT_HEIGHT_4K ,V4L2_PIX_FMT_SRGGB12, DOL_NUM_EXP_RAW);
	if (ret) {
		printf("unable to set format raw capture video device\n");
		goto err_set_fmt_video2;
	}

	ret = set_raw_capture_fps();
	if(ret){
		printf("unable to set fps for raw capture video device\n");
		goto err_set_fps;
	}

	ret = set_format(VIDEO_ISP_IN, INPUT_WIDTH_4K, INPUT_HEIGHT_4K, V4L2_PIX_FMT_SRGGB12, DOL_NUM_EXP_STITCHED);
	if (ret) {
		printf("unable to set format isp in video device\n");
		goto err_set_fmt_video3;
	}

	ret = init_buffers(VIDEO_RAW_CAPTURE, DOL_NUM_EXP_RAW);
	if (ret) {
		printf("unable to init buffers raw capture video device\n");
		goto err_init_buffers_video2;
	}

	ret = init_buffers(VIDEO_ISP_IN, DOL_NUM_EXP_STITCHED);
	if (ret) {
		printf("unable to init buffers isp in video device\n");
		goto err_init_buffers_video3;
	}

	ret = queue_buffers(VIDEO_RAW_CAPTURE);
	if (ret) {
		printf("unable to queue buffers video2\n");
		goto err_queue_buffers_video2;
	}

	ret = start_stream(VIDEO_RAW_CAPTURE);
	if (ret) {
		printf("unable to start stream video2\n");
		goto err_start_stream_video2;
	}
	ret = start_stream(VIDEO_ISP_IN);
	if (ret) {
		printf("unable to start stream video3\n");
		goto err_start_stream_video3;
	}
	mcm_loop();

	stop_stream(VIDEO_ISP_IN);
	ret = 0;
	printf("finished\n");

err_start_stream_video3:
	stop_stream(VIDEO_RAW_CAPTURE);
err_start_stream_video2:
err_queue_buffers_video2:
	free_buffers(VIDEO_ISP_IN);
err_init_buffers_video3:
	free_buffers(VIDEO_RAW_CAPTURE);
err_init_buffers_video2:
err_set_fmt_video3:
err_set_fps:
err_set_fmt_video2:
	delete stitcher;
	close(isp_in_fd);
err_video3:
	close(raw_capture_fd);
	return ret;
}
