#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <linux/videodev2.h>


/******************************/
/* Statistics data structures */
/******************************/


#define CAMDEV_AEV2_STATIC (4096 * 4)

struct AeLumaStatus {
    uint8_t luminance[25];
};

struct AeExpStatistics {
    uint8_t expStatistic[CAMDEV_AEV2_STATIC];
};

struct AeHistogram {
    unsigned int hist[256];
};

struct AfMeasurementStatus {
    uint32_t suma;
    uint32_t sumb;
    uint32_t sumc;
    uint32_t luma;
    uint32_t lumb;
    uint32_t lumc;
};

enum hailo15_event_stat_id {
    HAILO15_UEVENT_ISP_EXP_STAT,
    HAILO15_UEVENT_ISP_EXPV2_STAT,
    HAILO15_UEVENT_ISP_HIST_STAT,
    HAILO15_UEVENT_ISP_AWB_STAT,
    HAILO15_UEVENT_ISP_AFM_STAT,
    HAILO15_UEVENT_SENSOR_DATALOSS_STAT,
    HAILO15_UEVENT_VSM_DONE_STAT,
    HAILO15_UEVENT_ISP_STAT_MAX
};

union hailo15_stats_event {
    struct AeLumaStatus exp_stat;
    struct AeExpStatistics expv2_stat;
    struct AeHistogram hist_stat;
    struct AfMeasurementStatus afm_stat;
};


/*****************/
/* Usage example */
/*****************/


#define VIDEO_DEVICE_PATH "/dev/video0"
#define HAILO15_UEVENT_ISP_STAT (V4L2_EVENT_PRIVATE_START + 2000)

#define ISP_CID_AE_BASE (V4L2_CID_USER_BASE + 0x2000)
#define ISP_CID_AF_BASE (V4L2_CID_USER_BASE + 0x3500)

#define ISP_CID_AE_HIST (ISP_CID_AE_BASE + 0x000A)
#define ISP_CID_AE_LUMA (ISP_CID_AE_BASE + 0x000B)
#define ISP_CID_AE_EXP_STATUS (ISP_CID_AE_BASE + 0x0012)
#define ISP_CID_AF_MEASUREMENT (ISP_CID_AF_BASE + 0x0002)


static bool events_to_handle[] = {
    [HAILO15_UEVENT_ISP_EXP_STAT] = true,
    [HAILO15_UEVENT_ISP_EXPV2_STAT] = true,
    [HAILO15_UEVENT_ISP_HIST_STAT] = true,
    [HAILO15_UEVENT_ISP_AWB_STAT] = false,
    [HAILO15_UEVENT_ISP_AFM_STAT] = true,
    [HAILO15_UEVENT_SENSOR_DATALOSS_STAT] = true,
    [HAILO15_UEVENT_VSM_DONE_STAT] = true
};

static bool successfully_subscribed[HAILO15_UEVENT_ISP_STAT_MAX] = {false};
static int video_fd = -1;


uint32_t checksum(uint8_t* data, size_t len) {
    uint32_t i, sum = 0;
    for (i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}


void handle_isp_stats_event(enum hailo15_event_stat_id id, void *data, size_t size, uint32_t frame) {
    uint32_t data_checksum = checksum(data, size);
    printf("Received data for event id %d (%lu bytes, frame: %d, checksum: %d)\n",
        id, size, frame, data_checksum);
}

void handle_extra_stats_event(enum hailo15_event_stat_id id, void *data) {
    uint32_t sensor_index = *(uint32_t *)data;

    switch (id) {
        case HAILO15_UEVENT_SENSOR_DATALOSS_STAT:
            printf("Sensor data loss detected on sensor %d\n", sensor_index);
            break;
        case HAILO15_UEVENT_VSM_DONE_STAT:
            printf("VSM done\n");
            break;
        default:
            printf("Error: unexpected event id %d\n", id);
            break;
    }
}

int handle_statistic_events(int vid_fd) {
    struct pollfd poll_fds;
    struct v4l2_event event;
    struct v4l2_ext_controls ctrls;
    struct v4l2_ext_control ctrl;
    union hailo15_stats_event stats;

    uint32_t frame;
    int time_out_ms = 100;
    int ret = 0;

    memset(&ctrls, 0, sizeof(ctrls));
    memset(&ctrl, 0, sizeof(ctrl));
    poll_fds.fd = vid_fd;
    poll_fds.events = POLLIN | POLLPRI;

    while (1) {
        ret = poll(&poll_fds, 1, time_out_ms);
        if (ret <= 0) {
            continue;
        }

        ret = ioctl(vid_fd, VIDIOC_DQEVENT, &event);
        if (ret < 0) {
            continue;
        }

        if (event.type != HAILO15_UEVENT_ISP_STAT) {
            continue;
        }

        printf("Got stats event for stat id %d\n", event.id);

        switch (event.id) {
            case HAILO15_UEVENT_ISP_EXP_STAT:
                ctrl.id = ISP_CID_AE_LUMA;
                ctrl.size = sizeof(stats.exp_stat);
                break;

            case HAILO15_UEVENT_ISP_EXPV2_STAT:
                ctrl.id = ISP_CID_AE_EXP_STATUS;
                ctrl.size = sizeof(stats.expv2_stat);
                break;

            case HAILO15_UEVENT_ISP_HIST_STAT:
                ctrl.id = ISP_CID_AE_HIST;
                ctrl.size = sizeof(stats.hist_stat);
                break;

            case HAILO15_UEVENT_ISP_AWB_STAT:
                printf("AWB stats not supported in this example\n");
                continue;

            case HAILO15_UEVENT_ISP_AFM_STAT:
                ctrl.id = ISP_CID_AF_MEASUREMENT;
                ctrl.size = sizeof(stats.afm_stat);
                break;

            case HAILO15_UEVENT_SENSOR_DATALOSS_STAT:
            case HAILO15_UEVENT_VSM_DONE_STAT:
                handle_extra_stats_event(event.id, event.u.data);
                continue;

            default:
                printf("Unknown event id %d\n", event.id);
                return -1;
        }

        ctrl.ptr = &stats;
        ctrls.controls = &ctrl;
        ctrls.count = 1;

        if (ioctl(vid_fd, VIDIOC_G_EXT_CTRLS, &ctrls)) {
            return -1;
        }

        frame = *(uint32_t *)event.u.data;
        handle_isp_stats_event(event.id, &stats, ctrl.size, frame);
    }

    return 0;
}

void subscribe(int vid_fd, enum hailo15_event_stat_id id) {
    struct v4l2_event_subscription sub;
    sub.type = HAILO15_UEVENT_ISP_STAT;
    sub.id = id;
        
    if(-1 == ioctl(vid_fd, VIDIOC_SUBSCRIBE_EVENT, &sub)){
        fprintf(stderr, "unable to ioctl VIDIOC_SUBSCRIBE_EVENT to %s...\n", VIDEO_DEVICE_PATH);
        return;
    }

    printf("Subscribed to event id %d\n", id);
    successfully_subscribed[id] = true;
}

void unsubscribe(int vid_fd, enum hailo15_event_stat_id id) {
    struct v4l2_event_subscription sub;
    sub.type = HAILO15_UEVENT_ISP_STAT;
    sub.id = id;
        
    if(-1 == ioctl(vid_fd, VIDIOC_UNSUBSCRIBE_EVENT, &sub)){
        fprintf(stderr, "unable to ioctl VIDIOC_UNSUBSCRIBE_EVENT to %s...\n", VIDEO_DEVICE_PATH);
        return;
    }

    printf("Unsubscribed from event id %d\n", id);
    successfully_subscribed[id] = false;
}

// Function to clean up resources
void cleanup() {
    int i;

    if (video_fd < 0) {
        return;
    }

    printf("Cleaning up resources...\n");
    for (i = 0; i < HAILO15_UEVENT_ISP_STAT_MAX; i++) {
        if (successfully_subscribed[i]) {
            unsubscribe(video_fd, i);
        }
    }

    close(video_fd);
    printf("Closed %s\n", VIDEO_DEVICE_PATH);
}

// Signal handler for SIGINT (Ctrl+C)
void sigint_handler(int sig) {
    printf("Received SIGINT (Ctrl+C)\n");
    cleanup();
    exit(EXIT_SUCCESS); // Exit the program
}

int main(int argc, char *argv[]) {
    int ret, i;

    // Set up signal handler for SIGINT (Ctrl+C)
    if (signal(SIGINT, sigint_handler) == SIG_ERR) {
        perror("signal");
        return EXIT_FAILURE;
    }

    video_fd = open(VIDEO_DEVICE_PATH, O_RDWR);
    if (video_fd < 0){
        fprintf(stderr, "unable to open %s. open returned %d\n", VIDEO_DEVICE_PATH, video_fd);
        return EXIT_FAILURE;
    }

    printf("Opened %s, subscribing to statistic events\n", VIDEO_DEVICE_PATH);

    for (i = 0; i < HAILO15_UEVENT_ISP_STAT_MAX; i++) {
        if (events_to_handle[i]) {
            subscribe(video_fd, i);
        }
    }

    printf("Waiting for events...\n");
    ret = handle_statistic_events(video_fd);
    if (ret < 0) {
        fprintf(stderr, "handle_statistic_events returned %d\n", ret);
        cleanup();
        return EXIT_FAILURE;
    }

    cleanup();
	return 0;
}
