#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <math.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/v4l2-subdev.h>

#define MAX_NUM_OF_PLANES (3)
#define EXPECTED_FRAMES (30)
#define NUM_BUFFERS (5)
#define MAX_NUM_SUBDEVS (4)
#define DEV_PATH_SIZE (16)
#define NAME_FILE_LEN (39)
#define FRAME_HEIGHT (2160)
#define FRAME_WIDTH (3840)

#define VIDEO_DEVICE_NAME "/dev/video0"
#define SUBDEV_PATH "/sys/class/video4linux"
#define OUT_FILE_NAME "out"
#define VSM_FILE_NAME "out.vsm"
#define HAILO15_GET_VSM_IOC 0xc00c44c1

struct fmt {
	char *name;
	int pix_fmt;
	int num_planes;
};

static const struct fmt sdr_fmts[] = { {
			      .name = "nv12",
			      .pix_fmt = V4L2_PIX_FMT_NV12M,
			      .num_planes = 2,
		      },
		      {
			      .name = "rgb",
			      .pix_fmt = V4L2_PIX_FMT_RGB24,
			      .num_planes = 1,
		      },
		      {
			      .name = "yuy2",
			      .pix_fmt = V4L2_PIX_FMT_YUYV,
			      .num_planes = 1,
		      },
		      {
			      .name = "raw",
			      .pix_fmt = V4L2_PIX_FMT_SRGGB12P,
			      .num_planes = 1,
		      },
			  {
			      .name = "raw12",
			      .pix_fmt = V4L2_PIX_FMT_SRGGB12,
			      .num_planes = 1,
		      } };

static const struct fmt hdr_fmts[] = { { // TODO: make sure all of those are supported in hdr mode (isp/p2a)
			      .name = "nv12",
			      .pix_fmt = V4L2_PIX_FMT_NV12M,
			      .num_planes = 2,
		      },
		      {
			      .name = "rgb",
			      .pix_fmt = V4L2_PIX_FMT_RGB24,
			      .num_planes = 1,
		      },
		      {
			      .name = "yuy2",
			      .pix_fmt = V4L2_PIX_FMT_YUYV,
			      .num_planes = 1,
		      },
		      {
			      .name = "raw",
			      .pix_fmt = V4L2_PIX_FMT_SRGGB12P,
			      .num_planes = 3,
		      },
			  {
			      .name = "raw12",
			      .pix_fmt = V4L2_PIX_FMT_SRGGB12,
			      .num_planes = 3,
		      } };

struct sensor_name {
	char *name;
};

struct sensor_name sensors_names[] = {
	{
		.name = "imx678",
	},
	{
		.name = "imx334",
	},
};

struct hailo15_vsm {
	int dx;
	int dy;
};

struct hailo15_get_vsm_params {
	int index;
	struct hailo15_vsm vsm;
};

struct buffer {
	int num_planes;
	int sizes[MAX_NUM_OF_PLANES];
	void *planes[MAX_NUM_OF_PLANES];
	struct v4l2_buffer v4l2_buf;
};

struct fmt *current_fmt;

struct buffer *buffers;
static unsigned int n_buffers;

struct arg {
	char *key;
	char *val;
};

enum arg_type {
	STR,
	INT,
};

struct __args {
	char *name;
	int type;
	union {
		int i;
		char *s;
	} data;
	char *help;
} args[] = {
	{
		.name = "width",
		.type = INT,
		.data.i = 3840,
		.help = "width of output frame",
	},
	{
		.name = "height",
		.type = INT,
		.data.i = 2160,
		.help = "height of output frame",
	},
	{
		.name = "format",
		.type = STR,
		.data.s = "nv12",
		.help = "format of output frame. currently supported: nv12, rgb, yuy2",
	},
	{
		.name = "device",
		.type = STR,
		.data.s = "/dev/video0",
		.help = "path of video device to test",
	},
	{
		.name = "out-path",
		.type = STR,
		.data.s = "capture.out",
		.help = "frames output file path",
	},
	{
		.name = "vsm-out-path",
		.type = STR,
		.data.s = "vsm.out",
		.help = "vsm data output file path. will be ignored unless test type is vsm-test",
	},
	{
		.name = "type",
		.type = STR,
		.data.s = "capture",
		.help = "type of test to run. currently supported: capture, vsm-test",
	},
	{
		.name = "num-frames",
		.type = INT,
		.data.i = 30,
		.help = "number of frames to capture. will be ignored unless test type is capture",
	},
	{
		.name = "num-buffers",
		.type = INT,
		.data.i = NUM_BUFFERS,
		.help = "number of buffers to use when capturing. will be ignored unless test type is capture",
	},
	{
		.name = "save",
		.type = INT,
		.data.i = 1,
		.help = "save raw captures",
	},
	{
		.name = "fps",
		.type = INT,
		.data.i = 30,
		.help = "requested fps",
	},
};

static void print_help()
{
	int num_of_args = sizeof(args) / sizeof(struct __args);
	int index;
	printf("Hailo15 v4l test\n");
	printf("Arguments:\n");
	for (index = 0; index < num_of_args; ++index) {
		printf("\t--%s=[%s], %s\n\t    %s\n", args[index].name,
		       args[index].name,
		       args[index].type == INT ? "integer" : "string",
		       args[index].help);
		if (args[index].type == INT)
			printf("\t    default: %d\n", args[index].data.i);
		else
			printf("\t    default: %s\n", args[index].data.s);
	}
}

static int get_arg(char *name, struct __args *arg)
{
	int num_of_args = sizeof(args) / sizeof(struct __args);
	int index;
	for (index = 0; index < num_of_args; ++index) {
		if (!strcmp(args[index].name, name)) {
			memcpy(arg, &args[index], sizeof(struct __args));
			return 0;
		}
	}
	return -EINVAL;
}

static int split_arg(char *arg, struct arg *argument)
{
	char *token;
	token = strtok(arg, "=");
	if (!token)
		return -EINVAL;

	/*remove -- at start of arguemnt*/
	argument->key = token + 2;
	token = strtok(NULL, "=");
	if (!token)
		return -EINVAL;
	argument->val = token;
	return 0;
}

static int process_arg(struct arg *argument)
{
	int index;
	int num_of_args = sizeof(args) / sizeof(struct __args);
	for (index = 0; index < num_of_args; ++index) {
		if (!strcmp(argument->key, args[index].name))
			break;
	}
	/* couldn't find the arg */
	if (index == num_of_args)
		return -EINVAL;

	if (args[index].type == INT)
		args[index].data.i = atoi(argument->val);
	else if (args[index].type == STR)
		args[index].data.s = argument->val;
	else
		return -EINVAL;
	return 0;
}

static int parse_args(int argc, char **argv)
{
	int index;
	struct arg argument;
	int ret;
	for (index = 1; index < argc; index++) {
		if (!strcmp(argv[index], "--")) {
			printf("invalid argument %s\n", argv[index]);
			return -EINVAL;
		}
		ret = split_arg(argv[index], &argument);
		if (ret) {
			printf("invalid argument %s\n", argv[index]);
			return -EINVAL;
		}

		ret = process_arg(&argument);
		if (ret) {
			printf("unable to process argument %s\n", argv[index]);
			return -EINVAL;
		}
	}
	return 0;
}

static void errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int xioctl(int fh, uint32_t request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static int open_device(char *dev_name)
{
	struct stat st;
	static int fd = 0;

	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name,
			errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		exit(EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR /* required */, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,
			strerror(errno));
		exit(EXIT_FAILURE);
	}
	return fd;
}

int set_format(int fd, int hdr_enabled)
{
	struct v4l2_format fmt;
	struct __args arg;
	int ret;
	int i;
	struct fmt *supported_fmts;
	int fmts_count = 0;

	ret = get_arg("format", &arg);
	if (ret)
		return ret;
	if(hdr_enabled){
		supported_fmts = (struct fmt *)hdr_fmts;
		fmts_count = sizeof(hdr_fmts) / sizeof(struct fmt);
	}
	else {
		supported_fmts = (struct fmt *)sdr_fmts;
		fmts_count = sizeof(sdr_fmts) / sizeof(struct fmt);
	}
	for (i = 0; i < fmts_count; ++i) {
		if (!strcmp(arg.data.s, supported_fmts[i].name)) {
			current_fmt = &supported_fmts[i];
		}
	}
	if (!current_fmt) {
		printf("unsupported format %s\n", arg.data.s);
		return -EINVAL;
	}
	printf("Setting format %s, of mode %s\n", current_fmt->name, hdr_enabled ? "hdr" : "sdr");

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	ret = get_arg("width", &arg);
	if (ret)
		return ret;
	fmt.fmt.pix_mp.width = arg.data.i;
	ret = get_arg("height", &arg);
	if (ret)
		return ret;
	fmt.fmt.pix_mp.height = arg.data.i;
	fmt.fmt.pix_mp.pixelformat = current_fmt->pix_fmt;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.num_planes = current_fmt->num_planes;

	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
		return -EINVAL;
	}
	return 0;
}

int set_fps(int fd, int fps)
{
	struct v4l2_streamparm streamparm;
	memset(&streamparm, 0, sizeof(streamparm));

	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	streamparm.parm.capture.timeperframe.numerator = 1;
	streamparm.parm.capture.timeperframe.denominator = fps;
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm)) {
		return -EINVAL;
	}
	return 0;
}

int start_stream(int fd)
{
	enum v4l2_buf_type type;
	printf("Sending VIDIOC_STREAMON\n");

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");

	printf("Set streamon\n");
	return 0;
}

static void stop_stream(int fd)
{
	enum v4l2_buf_type type;

	printf("Sending VIDIOC_STREAMOFF\n");
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");
	printf("Set streamoff\n");
}

static int queue_buffer(int fd, int index)
{
	if (index < 0 || index >= n_buffers) {
		return -EINVAL;
	}

	return xioctl(fd, VIDIOC_QBUF, &buffers[index].v4l2_buf);
}

static int queue_buffers(int fd)
{
	int i = 0;
	int ret;
	for (i = 0; i < n_buffers; ++i) {
		ret = queue_buffer(fd, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int init_buffers(int fd)
{
	struct v4l2_requestbuffers req;
	struct __args arg;
	int ret;

	ret = get_arg("num-buffers", &arg);
	if (ret){
		printf("failed to get num-buffers arg\n");
		return ret;
	}

	memset(&req, 0, sizeof(struct v4l2_requestbuffers));

	req.count = arg.data.i;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	req.memory = V4L2_MEMORY_MMAP;

	printf("sending VIDIOC_REQBUFS\n");
	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		return -EINVAL;
	}

	if (req.count < arg.data.i) {
		return -ENOMEM;
	}

	buffers = calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		return -ENOMEM;
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;
		int plane;

		memset(&buf, 0, sizeof(struct v4l2_buffer));

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;
		buf.length = current_fmt->num_planes;
		buf.m.planes = malloc(current_fmt->num_planes *
				      sizeof(struct v4l2_plane));
		memset(buf.m.planes, 0,
		       current_fmt->num_planes * sizeof(struct v4l2_plane));

		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
			return errno;

		buffers[n_buffers].num_planes = buf.length;
		for (plane = 0; plane < buf.length; ++plane) {
			buffers[n_buffers].sizes[plane] =
				buf.m.planes[plane].length;
			buffers[n_buffers].planes[plane] =
				mmap(NULL /* start anywhere */,
				     buf.m.planes[plane].length,
				     PROT_READ | PROT_WRITE /* required */,
				     MAP_SHARED /* recommended */, fd,
				     buf.m.planes[plane].m.mem_offset);

			if (MAP_FAILED == buffers[n_buffers].planes[plane])
				return -ENOMEM;
		}
		memcpy(&buffers[n_buffers].v4l2_buf, &buf,
		       sizeof(struct v4l2_buffer));
	}
	return 0;
}

static void free_buffers()
{
	int frame;
	int plane;
	for (frame = 0; frame < n_buffers; ++frame) {
		free(buffers[frame].v4l2_buf.m.planes);
		for (plane = 0; plane < buffers[frame].v4l2_buf.length; ++plane)
			munmap(buffers[frame].planes[plane],
			       buffers[frame].sizes[plane]);
	}

	free(buffers);
}

static int process_frame(int index, int video_fd, int out_fd, int vsm_fd)
{
	int plane;
	struct buffer *buf = &buffers[index];
	struct hailo15_get_vsm_params params;
	int ret;

	memset(&params, 0, sizeof(params));
	if (out_fd >= 0) {
		for (plane = 0; plane < buf->num_planes; ++plane) {
			ret = write(out_fd, buf->planes[plane],
				    buf->sizes[plane]);
			if (ret != buf->sizes[plane])
				return -ENOMEM;
		}
	}

	if (vsm_fd >= 0) {
		params.index = index;
		if (-1 == xioctl(video_fd, HAILO15_GET_VSM_IOC, &params))
			return -EINVAL;
		dprintf(vsm_fd, "(%d,%d),", params.vsm.dx, params.vsm.dy);
	}
	return 0;
}

static int write_frame(int index, const char *output_path)
{
	int plane;
	struct buffer *buf = &buffers[index];
	struct hailo15_get_vsm_params params;
	int ret;
	int dump_file = 0;
	dump_file = open(output_path, O_RDWR | O_SYNC | O_CREAT, S_IRWXU);

	memset(&params, 0, sizeof(params));
	if (dump_file >= 0) {
		for (plane = 0; plane < buf->num_planes; ++plane) {
			ret = write(dump_file, buf->planes[plane],
				    buf->sizes[plane]);
			if (ret != buf->sizes[plane])
				return -ENOMEM;
		}
		close(dump_file);
	}
	return 0;
}

static int read_frame(int fd)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[MAX_NUM_OF_PLANES];
	memset(&buf, 0, sizeof(struct v4l2_buffer));

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.length = current_fmt->num_planes;
	buf.m.planes = planes;
	memset(buf.m.planes, 0, MAX_NUM_OF_PLANES * sizeof(struct v4l2_plane));
	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		return -EINVAL;
	}

	if (buf.index >= n_buffers) {
		return -EINVAL;
	}

	return buf.index;
}

static int index_padding_length(int total_frames)
{
    static const size_t max_index_padding_length = 10;
    uint8_t index_padding = (uint8_t)log10(total_frames) + 1;
    return index_padding < max_index_padding_length ? index_padding : max_index_padding_length;
}

static char* capture_file_name(int frame, int index_padding)
{
    static char file_name[32];
    snprintf(file_name, sizeof(file_name), "out_%0*d.raw", index_padding, frame);
    return file_name;
}

static int capture_frames_test_loop(int fd, int out_fd)
{
	unsigned int frame = 0;
	int ret = 0;
	struct __args arg;
	int num_frames;
	int current_frame;
	int is_raw = 0;
	int save_raw = 0;
	int index_padding = 0;

	ret = get_arg("num-frames", &arg);
	if (ret)
		return ret;
	num_frames = arg.data.i;
	if (num_frames < 1)
		return -EINVAL;

	ret = get_arg("format", &arg);
	if (ret)
		return ret;

	if (!strcmp(arg.data.s, "raw") || !strcmp(arg.data.s, "raw12")) {
		is_raw = 1;
		ret = get_arg("save", &arg);
		if (ret)
			return ret;
		save_raw = arg.data.i;
	}

	index_padding = index_padding_length(num_frames);

	for (frame = 0; frame < num_frames; ++frame) {
		ret = read_frame(fd);
		if (ret < 0 || ret > n_buffers) {
			return -EINVAL;
		}
		current_frame = ret;
		if (is_raw) {
			if (save_raw) {
				char* raw_output_path = capture_file_name(frame, index_padding);
				printf("Writing frame %d to %s\n", frame, raw_output_path);
				write_frame(current_frame, raw_output_path);
			}
		} else {
			ret = process_frame(current_frame, fd, out_fd, -1);
			if (ret)
				return ret;
		}
		queue_buffer(fd, current_frame);
	}

	return 0;
}

static int frames_vsm_test_loop(int fd, int out_fd, int vsm_fd)
{
	//hard coded - read 2 framses
	unsigned int frames = 0;
	int ret = 0;

	for (frames = 0; frames < n_buffers; ++frames) {
		ret = read_frame(fd);
		if (ret < 0 || ret > n_buffers) {
			return -EINVAL;
		}
	}

	/* wait for ae and awb to balance*/
	sleep(5);

	/* queue all buffers back*/
	queue_buffers(fd);

	for (frames = 0; frames < n_buffers; ++frames) {
		ret = read_frame(fd);
		if (ret < 0 || ret > n_buffers) {
			return -EINVAL;
		}
	}

	for (frames = 0; frames < n_buffers; ++frames) {
		process_frame(frames, fd, out_fd, vsm_fd);
	}

	return 0;
}

static int run_test(char *name, int fd)
{
	int ret;
	int out_fd;
	int vsm_fd;
	struct __args arg;
	ret = get_arg("out-path", &arg);
	if (ret)
		return ret;

	out_fd = open(arg.data.s, O_RDWR | O_CREAT, 0666);
	if (out_fd < 0) {
		printf("unable to open out file");
		return ret;
	}

	if (!strcmp(name, "capture")) {
		printf("Running capture test\n");
		ret = capture_frames_test_loop(fd, out_fd);
	} else if (!strcmp(name, "vsm-test")) {
		ret = get_arg("vsm-out-path", &arg);
		if (ret)
			goto out;

		vsm_fd = open(arg.data.s, O_RDWR | O_CREAT, 0666);
		if (vsm_fd < 0) {
			printf("unable to open vsm file\n");
			goto out;
		}

		printf("Running vsm test\n");
		ret = frames_vsm_test_loop(fd, out_fd, vsm_fd);
		close(vsm_fd);
	} else {
		printf("No such test: %s\n", name);
		ret = -EINVAL;
	}
out:
	close(out_fd);
	return ret;
}

int find_sensor_subdev() {
    DIR *dir = opendir(SUBDEV_PATH);
    struct dirent *entry;

    if (dir == NULL) {
        perror("opendir");
        return -1;
    }

    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, "v4l-subdev") != NULL) {
            char path[512];
            snprintf(path, sizeof(path), "%s/%s/name", SUBDEV_PATH, entry->d_name);

            FILE *file = fopen(path, "r");
            if (file != NULL) {
                char name[64];
                if (fscanf(file, "%s", name) == 1) {
					for (int i = 0; i < sizeof(sensors_names) / sizeof(struct sensor_name); ++i) {
						if (strcmp(name, sensors_names[i].name) == 0) {
							closedir(dir);
							return atoi(entry->d_name + strlen("v4l-subdev"));
                    	}
					}
                }
                fclose(file);
            }
        }
    }
	return -1;
}

int is_hdr_enabled(int fd) {
	struct v4l2_control ctrl;
	ctrl.id = V4L2_CID_WIDE_DYNAMIC_RANGE;
	if (-1 == xioctl(fd, VIDIOC_G_CTRL, &ctrl)) {
		return -EINVAL;
	}
	return ctrl.value;
}

int main(int argc, char *argv[])
{
	struct __args arg;
	static int fd_video = 0;
	static int fd_sensor = 0;
	int ret;
	ret = -EINVAL;
	current_fmt = NULL;
	int hdr_enabled = -1;
	int sensor_sd_index = -1;
	char sd_sensor_dev_path[32];

	if (argc == 2 &&
	    (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
		print_help();
		exit(0);
	}

	printf("Starting Hailo15 v4l test\n");
	ret = parse_args(argc, (char **)argv);
	if (ret) {
		printf("unable to parse args\n");
		goto out;
	}

	ret = get_arg("device", &arg);
	if (ret) {
		goto out;
	}

	fd_video = open_device(arg.data.s);
	if (fd_video < 0) {
		printf("unable to open video device\n");
		goto out;
	}

	sensor_sd_index = find_sensor_subdev();
	if (sensor_sd_index < 0) {
		printf("unable to find sensor subdev\n");
		goto out;
	}
    snprintf(sd_sensor_dev_path, sizeof(sd_sensor_dev_path), "/dev/v4l-subdev%d", sensor_sd_index);

	fd_sensor = open_device(sd_sensor_dev_path);

	hdr_enabled = is_hdr_enabled(fd_sensor);

	ret = set_format(fd_video, hdr_enabled);
	if (ret) {
		printf("unable to set format\n");
		goto err_set_fmt;
	}

	ret = get_arg("fps", &arg);
	if (ret){
		printf("failed to get fps arg\n");
		return ret;
	}

	ret = set_fps(fd_video, arg.data.i);
	if (ret) {
		printf("unable to set fps\n");
		goto err_set_fmt;
	}

	ret = init_buffers(fd_video);
	if (ret) {
		printf("unable to init buffers\n");
		goto err_init_buffers;
	}

	ret = queue_buffers(fd_video);
	if (ret) {
		printf("unable to queue buffers\n");
		goto err_queue_buffers;
	}

	ret = start_stream(fd_video);
	if (ret) {
		printf("unable to start stream\n");
		goto err_start_stream;
	}
	ret = get_arg("type", &arg);
	if (ret)
		goto err_test_loop;

	ret = run_test(arg.data.s, fd_video);
	if (ret) {
		printf("test loop failed\n");
		goto err_test_loop;
	}

	stop_stream(fd_video);

	ret = 0;
	printf("finished\n");

err_test_loop:
err_start_stream:
err_queue_buffers:
	free_buffers();
err_init_buffers:
err_set_fmt:
	close(fd_video);
out:
	return ret;
}
