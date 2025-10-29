#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <csignal>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/v4l2-subdev.h>
#include <sys/epoll.h>
#include "hailo_stitch.hpp"
#include "../../../common/ioctl_cmds.h"

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
#define LS_RATIO 16
#define VS_RATIO 4

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
static unsigned char* wb_buffer = (unsigned char*)MAP_FAILED;
static constexpr int wb_buffer_size = 12;
static constexpr float wb_compensation = 0.03143406;
std::unordered_map<std::string, struct v4l2_query_ext_ctrl> ctrl_map;
int fd_video_yuv = -1;

enum {
	VIDEO_RAW_CAPTURE,
	VIDEO_ISP_IN,
};

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
		fprintf(stderr, "Error: Invalid buffer index %d for path %d\n", index, path);
		return -EINVAL;
	}

	int ret = xioctl(path_to_fd(path), VIDIOC_QBUF, &buffers[path][index].v4l2_buf);

	if (ret == -1) {
		fprintf(stderr, "Error: Failed to queue buffer %d for path %d\n", index, path);
		return errno;
	}

	return 0;
}

static int queue_buffers(int path)
{
	unsigned int i = 0;
	int ret;
	for (i = 0; i < n_buffers[path]; ++i) {
		ret = queue_buffer(path, i);
		if (ret) {
			fprintf(stderr, "Error: Failed to queue buffer %d for path %d\n", i, path);
			return ret;
		}
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
		fprintf(stderr, "Error: VIDIOC_REQBUFS failed for path %d with error %d: %s\n", path, errno, strerror(errno));
		return -EINVAL;
	}
	if (req.count < path_to_buf_count(path)) {
		fprintf(stderr, "Error: Not enough buffers allocated for path %d\n", path);
		return -ENOMEM;
	}

	buffers[path] = (buffer*)calloc(req.count, sizeof(*buffers[path]));

	if (!buffers[path]) {
		fprintf(stderr, "Error: Failed to allocate memory for buffers on path %d\n", path);
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
		if (!buf.m.planes) {
			fprintf(stderr, "Error: Failed to allocate memory for planes in buffer %d on path %d\n", n_buffers[path], path);
			free(buffers[path]);
			return -ENOMEM;
		}
		memset(buf.m.planes, 0,
		       num_planes * sizeof(struct v4l2_plane));

		if (-1 == xioctl(path_to_fd(path), VIDIOC_QUERYBUF, &buf)) {
			fprintf(stderr, "Error: VIDIOC_QUERYBUF failed for buffer %d on path %d\n", n_buffers[path], path);
			free(buf.m.planes);
			free(buffers[path]);
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

			if (MAP_FAILED == buffers[path][n_buffers[path]].planes[plane]) {
				fprintf(stderr, "Error: mmap failed for buffer %d, plane %d on path %d\n", n_buffers[path], plane, path);
				free(buf.m.planes);
				free(buffers[path]);
				return -ENOMEM;
			}
		}
		buffers[path][n_buffers[path]].first_use = 0;
		memcpy(&buffers[path][n_buffers[path]].v4l2_buf, &buf,
		       sizeof(struct v4l2_buffer));
	}
	wb_buffer = (unsigned char*)mmap(NULL, wb_buffer_size, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
	if (MAP_FAILED == wb_buffer) {
		fprintf(stderr, "Error: mmap failed for wb_buffer\n");
		free(buffers[path]);
		return -ENOMEM;
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
	buffers[path] = nullptr;
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
	int ret = xioctl(path_to_fd(path), VIDIOC_DQBUF, &buf);
	if (ret == -1) {
		fprintf(stderr, "Error: Failed to dequeue buffer for path %d: %d\n", path, errno);
		return -1;
	}

	if (buf.index >= n_buffers[path]) {
		fprintf(stderr, "Error: Invalid buffer index %d for path %d\n", buf.index, path);
		return -1;
	}

	return buf.index;
}


void dequeue_all_buffers(int path) {
    unsigned int index2 = 0;
    struct epoll_event event;
    struct epoll_event events[1];
    int epoll_fd;
    int timeout_ms = 1;
    
    epoll_fd = epoll_create1(0);
    if (epoll_fd == -1) {
        fprintf(stderr, "epoll_create1 failed");
        return;
    }

    event.events = EPOLLIN;
    event.data.fd = raw_capture_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, raw_capture_fd, &event) == -1) {
        fprintf(stderr, "epoll_ctl failed");
        close(epoll_fd);
        return;
    }

    while (index2 >= 0) {
        int nfds = epoll_wait(epoll_fd, events, 1, timeout_ms);
        
        if (nfds == 0) {
            // Timeout expired (no events)
            close(epoll_fd);
            return;
        } else if (nfds == -1) {
            fprintf(stderr, "epoll_wait failed");
            close(epoll_fd);
            return;
        }

        // If the event is triggered for raw_capture_fd, process it
        if (events[0].data.fd == raw_capture_fd) {
            index2 = read_frame(VIDEO_RAW_CAPTURE, DOL_NUM_EXP_RAW);
        }
    }

    close(epoll_fd);
}

bool wait_for_yuv_stream_start(){
	int ret = -1;
	fd_video_yuv = open(VIDEO_YUV_PATH, O_RDWR);
	if (fd_video_yuv < 0){
		fprintf(stderr, "cant open yuv output video device\n");
		return false;
	}

	ret = ioctl(fd_video_yuv, VIDEO_WAIT_FOR_STREAM_START);
	if (ret < 0){
		fprintf(stderr, "VIDEO_WAIT_FOR_STREAM_START failed with return value %d\n", ret);
		return false;
	}
	return true;
}


bool getV4l2Qctrl(std::string name, struct v4l2_query_ext_ctrl& o_qctrl){

	if(ctrl_map.contains(name)){
		o_qctrl = ctrl_map[name];
		return true;
	}

	int ret = -1;
	const unsigned next_flag = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
	struct v4l2_query_ext_ctrl qctrl;
	memset(&qctrl, 0, sizeof(qctrl));
	qctrl.id = next_flag;
	while (true)
	{
		ret = ioctl(fd_video_yuv, VIDIOC_QUERY_EXT_CTRL, &qctrl);
		if (ret < 0)
		{
			fprintf(stderr, "VIDIOC_QUERY_EXT_CTRL failed with return value %d\n", ret);
			return false;
		}
		if (0 == strcmp(qctrl.name, name.c_str()))
		{
			ctrl_map[name] = qctrl;
			o_qctrl = ctrl_map[name];
			return true;
		}

		qctrl.id |= next_flag;
	}

	return false;
}

bool getIspCtrl(std::string name, int& o_val){

	struct v4l2_ext_control ctrl;
	struct v4l2_ext_controls ctrls;
	struct v4l2_query_ext_ctrl qctrl;
	int ret = -1;
	memset(&ctrl, 0, sizeof(ctrl));
	memset(&ctrls, 0, sizeof(ctrls));
	memset(&qctrl, 0, sizeof(qctrl));
	
	if(!getV4l2Qctrl(name, qctrl)){
		return false;
	}

	ctrl.id = qctrl.id;
	ctrl.size = qctrl.elem_size * qctrl.elems;
	ctrls.count = 1;
	ctrls.controls = &ctrl;
	ctrls.which = V4L2_CTRL_ID2WHICH(ctrl.id);
	ret = ioctl(fd_video_yuv, VIDIOC_G_EXT_CTRLS, &ctrls);
	if(ret != 0){
		fprintf(stderr, "VIDIOC_G_EXT_CTRLS failed with return value %d\n", ret);
		return ret;
	}
	o_val = ctrl.value;
	return true;
}


bool setIspCtrlPtr(std::string name, unsigned int* val){

	struct v4l2_ext_control ctrl;
	struct v4l2_ext_controls ctrls;
	struct v4l2_query_ext_ctrl qctrl;
	int ret = -1;
	memset(&ctrl, 0, sizeof(ctrl));
	memset(&ctrls, 0, sizeof(ctrls));
	memset(&qctrl, 0, sizeof(qctrl));
	if(!getV4l2Qctrl(name, qctrl))
		return false;
		
	ctrl.id = qctrl.id;
	ctrl.size = qctrl.elem_size * qctrl.elems;
	ctrl.p_u32 = val;
	ctrls.count = 1;
	ctrls.controls = &ctrl;
	ctrls.which = V4L2_CTRL_ID2WHICH(ctrl.id);
	ret = ioctl(fd_video_yuv, VIDIOC_S_EXT_CTRLS, &ctrls);
	if(ret != 0){
		fprintf(stderr, "VIDIOC_S_EXT_CTRLS failed with return value %d\n", ret);
		return false;
	}

	return true;
}

bool setRatio(float lsRatio, float vsRatio){
	unsigned int ratio[2];
	memset(ratio, 0, sizeof(ratio));
	ratio[0] = lsRatio * (1<<16);
	ratio[1] = vsRatio * (1<<16);
	return setIspCtrlPtr("isp_hdr_ratio", ratio);
}

void updateWBGains(unsigned char* wbBuffer){
	int channels_raw[4];
	float channels[4];
	memset(channels_raw, 0, sizeof(channels_raw));
	memset(channels, 0, sizeof(channels));


	if (!getIspCtrl("isp_wb_r_gain", channels_raw[0]) ||
		!getIspCtrl("isp_wb_gr_gain", channels_raw[1]) ||
		!getIspCtrl("isp_wb_gb_gain", channels_raw[2]) ||
		!getIspCtrl("isp_wb_b_gain", channels_raw[3])) {
		fprintf(stderr, "Error: Failed to retrieve ISP WB gain\n");
		return;
	}
	
	for(int channel = 0; channel < 4; ++channel){
		channels[channel] = ((float)channels_raw[channel]) / 256;
		float channel_quant = channels[channel] / wb_compensation;
		int channel_to_buffer = std::ceil(channel_quant);
		// we need to limit the value to 127 because the NN-core will not accept values greater than 127
		channel_to_buffer = std::min(channel_to_buffer, 127);
		wbBuffer[channel] = channel_to_buffer;
		wbBuffer[channel + 4] = channel_to_buffer;
		wbBuffer[channel + 8] = channel_to_buffer;
	}
}

static void mcm_loop()
{
	int index2 = 0;
	int index3;

	if(!wait_for_yuv_stream_start()) {
		return;
	}
	if(!setRatio(LS_RATIO, VS_RATIO)) {
		return;
	}
	dequeue_all_buffers(VIDEO_RAW_CAPTURE);
	queue_buffers(VIDEO_RAW_CAPTURE);
	while(true){
		index2 = read_frame(VIDEO_RAW_CAPTURE, DOL_NUM_EXP_RAW);
		index3 = read_frame(VIDEO_ISP_IN, DOL_NUM_EXP_STITCHED);
		updateWBGains(wb_buffer);

		stitcher->process(buffers[VIDEO_RAW_CAPTURE][index2].planes, wb_buffer, buffers[VIDEO_ISP_IN][index3].planes[0]);

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

static void cleanup() {
    printf("stopping stream\n");
    stop_stream(VIDEO_ISP_IN);
    stop_stream(VIDEO_RAW_CAPTURE);

    if (set_isp_mcm_mode(ISP_MCM_MODE_OFF)) {
        printf("failed to set mcm mode to ISP_MCM_MODE_OFF\n");
    } else {
        printf("finished\n");
    }

    if (fd_video_yuv >= 0) {
        close(fd_video_yuv);
    }
    
    free_buffers(VIDEO_ISP_IN);
    free_buffers(VIDEO_RAW_CAPTURE);

	if(wb_buffer != MAP_FAILED)
		munmap(wb_buffer, wb_buffer_size);

    delete stitcher;
    
    close(isp_in_fd);
    close(raw_capture_fd);
}

void signal_handler(int signal) {
    printf("got signal %d, exiting...\n", signal);
    cleanup();
    exit(0);
}

int main(int argc, char *argv[])
{
	int ret = 0;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

	printf("Starting Hailo15 HDR manager\n");
	
	stitcher = new HailortAsyncStitching();
	if (stitcher->init(std::string(DEFAULT_HEF_PATH), std::string("0"), 1, 1000, DOL_NUM_EXP_RAW) != 0) {
		printf("unable to initialize stitcher\n");
		ret = -1;
        goto cleanup_label;
	}
	stitcher->set_on_infer(on_infer);

	if (set_isp_mcm_mode(ISP_MCM_MODE_STITCHING)) {
		printf("failed to set mcm mode to ISP_MCM_MODE_STITCHING\n");
		ret = -1;
        goto cleanup_label;
	}

	raw_capture_fd = open_device("/dev/video2");
	if (raw_capture_fd < 0) {
		printf("unable to open raw capture video device\n");
		ret = -1;
        goto cleanup_label;
	}

	isp_in_fd = open_device("/dev/video10");
	if (isp_in_fd < 0) {
		printf("unable to open isp in video device\n");
		ret = -1;
        goto cleanup_label;
	}

	if (set_format(VIDEO_RAW_CAPTURE, INPUT_WIDTH_4K, INPUT_HEIGHT_4K ,V4L2_PIX_FMT_SRGGB12, DOL_NUM_EXP_RAW)) {
		printf("unable to set format raw capture video device\n");
		ret = -1;
        goto cleanup_label;
	}

	if(set_raw_capture_fps()){
		printf("unable to set fps for raw capture video device\n");
		ret = -1;
        goto cleanup_label;
	}

	if (set_format(VIDEO_ISP_IN, INPUT_WIDTH_4K, INPUT_HEIGHT_4K, V4L2_PIX_FMT_SRGGB12, DOL_NUM_EXP_STITCHED)) {
		printf("unable to set format isp in video device\n");
		ret = -1;
        goto cleanup_label;
	}

	if (init_buffers(VIDEO_RAW_CAPTURE, DOL_NUM_EXP_RAW)) {
		printf("unable to init buffers raw capture video device\n");
		ret = -1;
        goto cleanup_label;
	}

	if (init_buffers(VIDEO_ISP_IN, DOL_NUM_EXP_STITCHED)) {
		printf("unable to init buffers isp in video device\n");
		ret = -1;
        goto cleanup_label;
	}

	if (queue_buffers(VIDEO_RAW_CAPTURE)) {
		printf("unable to queue buffers video2\n");
		ret = -1;
        goto cleanup_label;
	}

	if (start_stream(VIDEO_RAW_CAPTURE)) {
		printf("unable to start stream video2\n");
		ret = -1;
        goto cleanup_label;
	}
	if (start_stream(VIDEO_ISP_IN)) {
		printf("unable to start stream video3\n");
		ret = -1;
        goto cleanup_label;
	}
	mcm_loop();

cleanup_label:
    cleanup();
	return ret;
}
