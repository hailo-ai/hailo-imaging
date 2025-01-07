#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/v4l2-subdev.h>

#define NUM_SECONDS 10
#define MAX_CTRL_NAME_LEN 32

typedef struct QExtCtrlsList_s {
  char name[MAX_CTRL_NAME_LEN];
  struct v4l2_query_ext_ctrl *qExtCtrl;
  struct QExtCtrlsList_s *next;
} QExtCtrlsList_t;

static int xioctl(int fh, uint32_t request, void *arg) {
  int r;

  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

static int open_device(char *dev_name) {
  struct stat st;
  static int fd = 0;

  if (-1 == stat(dev_name, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno,
            strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no device\n", dev_name);
    exit(EXIT_FAILURE);
  }

  fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,
            strerror(errno));
    exit(EXIT_FAILURE);
  }
  return fd;
}

static QExtCtrlsList_t *addQExtCtrl(QExtCtrlsList_t *list_head, char *ctrl_name,
                                    struct v4l2_query_ext_ctrl *qExtCtrl) {
  QExtCtrlsList_t *new_ctrl;
  if (!qExtCtrl) {
    printf("%s - bad parameter\n", __func__);
    return NULL;
  }

  new_ctrl = (QExtCtrlsList_t *)malloc(sizeof(QExtCtrlsList_t));
  if (!new_ctrl) {
    printf("%s - Can't allocate new ctrl\n", __func__);
    return NULL;
  }

  strncpy(new_ctrl->name, ctrl_name, MAX_CTRL_NAME_LEN);
  new_ctrl->qExtCtrl = malloc(sizeof(struct v4l2_query_ext_ctrl));
  memcpy(new_ctrl->qExtCtrl, qExtCtrl, sizeof(*qExtCtrl));
  new_ctrl->next = list_head;

  return new_ctrl;
}

static struct v4l2_query_ext_ctrl *findQExtCtrl(QExtCtrlsList_t *list_head,
                                                char *ctrl_name) {
  QExtCtrlsList_t *cur_ctrl;
  if (!list_head || !ctrl_name) {
    printf("%s - bad parameter\n", __func__);
    return NULL;
  }

  cur_ctrl = list_head;
  while (cur_ctrl) {
    if (strncmp(ctrl_name, cur_ctrl->name, MAX_CTRL_NAME_LEN) == 0)
      return cur_ctrl->qExtCtrl;
    cur_ctrl = cur_ctrl->next;
  }

  return NULL;
}

static void releaseQExtCtrls(QExtCtrlsList_t *list_head) {
  QExtCtrlsList_t *tmp;
  if (list_head) {
    tmp = list_head->next;
    free(list_head->qExtCtrl);
    free(list_head);
    releaseQExtCtrls(tmp);
  }
}

static void printQExtCtrlsList(QExtCtrlsList_t *list_head) {
  QExtCtrlsList_t *cur_ctrl = list_head;
  printf("----------------- Query Extra Controls List -----------------\n");
  while (cur_ctrl) {
    printf("name: %s, id: 0x%x, num elems: %u, elem size: %u\n", cur_ctrl->name,
           cur_ctrl->qExtCtrl->id, cur_ctrl->qExtCtrl->elems,
           cur_ctrl->qExtCtrl->elem_size);
    cur_ctrl = cur_ctrl->next;
  }
}

static QExtCtrlsList_t *queryExtCtrls(int fd) {
  int rc = 0;
  struct v4l2_query_ext_ctrl qctrl;
  QExtCtrlsList_t *ctrlsList = NULL;
  const unsigned next_flag =
      V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;

  memset(&qctrl, 0, sizeof(qctrl));
  qctrl.id = next_flag;
  do {
    rc = xioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (rc < 0) {
      rc = 0;
      break;
    }
    ctrlsList = addQExtCtrl(ctrlsList, qctrl.name, &qctrl);
    if (!ctrlsList) {
      break;
    }
    qctrl.id |= next_flag;

  } while (1);

  return ctrlsList;
}

static int get_ext_ctrls(int fd, unsigned int ctrl_cid,
                         struct v4l2_ext_control *ExtCtrl) {
  int rc = 0;
  struct v4l2_ext_controls ExtCtrls;
  struct v4l2_query_ext_ctrl qctrl;

  memset(&qctrl, 0, sizeof(qctrl));
  qctrl.id = ctrl_cid;

  rc = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
  if (rc < 0) {
    printf("VIDIOC_QUERY_EXT_CTRL returned %d", rc);
    return rc;
  }

  memset(ExtCtrl, 0, sizeof(*ExtCtrl));
  ExtCtrl->id = qctrl.id;
  ExtCtrl->size = qctrl.elem_size * qctrl.elems;
  ExtCtrl->ptr = malloc(ExtCtrl->size);
  ExtCtrls.count = 1;
  ExtCtrls.controls = ExtCtrl;
  ExtCtrls.which = V4L2_CTRL_ID2WHICH(qctrl.id);

  rc = xioctl(fd, VIDIOC_G_EXT_CTRLS, &ExtCtrls);
  if (rc) {
    printf("%s ioctl VIDIOC_G_EXT_CTRLS failed with %d, errno: %d\n", __func__,
           rc, errno);
    return rc;
  }

  return 0;
}

static int set_ext_ctrls(int fd, struct v4l2_ext_control *ExtCtrl) {
  int rc = 0;
  struct v4l2_ext_controls ExtCtrls;

  ExtCtrls.count = 1;
  ExtCtrls.controls = ExtCtrl;
  ExtCtrls.which = V4L2_CTRL_ID2WHICH(ExtCtrl->id);

  rc = xioctl(fd, VIDIOC_S_EXT_CTRLS, &ExtCtrls);
  if (rc) {
    printf("%s ioctl VIDIOC_S_EXT_CTRLS failed with %d, errno: %d\n", __func__,
           rc, errno);
  }

  return rc;
}

int set_roi_ctrls(int fd_video, QExtCtrlsList_t *qCtrls)
{
  struct v4l2_ext_control ExtCtrl;
  struct v4l2_query_ext_ctrl *ae_roi_ext_ctrl;
  struct v4l2_query_ext_ctrl *ae_roi_weight_ctrl;
  int rc=0;

  uint32_t roi_matrix[] = {0, 0, 320, 180, //first window
                           320, 0 ,320 ,180, //2nd window
                           0, 180 ,320 ,180, //3rd ...
                           320, 180 ,320 ,180, /*xstart, ystart, width, height*/
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0,
                           0, 0 ,0 ,0};

  float roi_weight_list[] = {50.0, 25.0, 25.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; /*roi weight, 0-100*/

  ae_roi_ext_ctrl = findQExtCtrl(qCtrls, "isp_ae_roi");
  ae_roi_weight_ctrl = findQExtCtrl(qCtrls, "isp_ae_roi_weight");

  printf("Sending ROI controls\n");

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  ExtCtrl.id = ae_roi_ext_ctrl->id;
  ExtCtrl.size = ae_roi_ext_ctrl->elem_size * ae_roi_ext_ctrl->elems;
  ExtCtrl.ptr = roi_matrix;

  rc = set_ext_ctrls(fd_video, &ExtCtrl);
  if (rc != 0) {
    printf("set_ext_ctrls returned %d\n", rc);
    return 1;
  }
  printf("set isp_ae_roi succeeded\n");

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  ExtCtrl.id = ae_roi_weight_ctrl->id;
  ExtCtrl.size = ae_roi_weight_ctrl->elem_size * ae_roi_weight_ctrl->elems;
  ExtCtrl.ptr = roi_weight_list;

  rc = set_ext_ctrls(fd_video, &ExtCtrl);
  if (rc != 0) {
    printf("set_ext_ctrls returned %d\n", rc);
    return 1;
  }
  printf("set isp_ae_roi_weight succeeded\n");

  return 0;
}

int main(int argc, char *argv[]) {
  static int fd_video = 0;
  QExtCtrlsList_t *qCtrls;
  struct v4l2_ext_control ExtCtrl;
  struct v4l2_query_ext_ctrl *wb_r_gain_ctrl;
  struct v4l2_query_ext_ctrl *wb_cc_matrix_ctrl;
  struct v4l2_query_ext_ctrl *af_ext_ctrl;
  struct v4l2_query_ext_ctrl *af_measure_ctrl;
  struct v4l2_query_ext_ctrl *iris_limits_ext_ctrl;

  float *new_wb_matrix;
  int *new_af_windows;
  uint32_t *new_iris_limits;
  uint32_t* new_af_measure;
  float wb_matrix[] = {0.100, 0.150, 0.200, 0.250, 0.300,
                          0.350, 0.400, 0.450, 0.500};
  int af_windows[] = {
      /*first window: h,w,x,y*/ 100,  100, 100, 100,
      /*second window: h,w,x,y*/ 100, 100, 200, 100,
      /*first window: h,w,x,y*/ 50,   100, 150, 150};

  /*iris limits - min = 4.35, max = 5.67*/
  uint32_t iris_limits[] = {435, 567};
  int rc;

  printf("Starting V4L Control Example\n");

  fd_video = open_device("/dev/video0");

  qCtrls = queryExtCtrls(fd_video);
  if (!qCtrls) {
    printf("failed to query extra controls\n");
    return 1;
  }

  printQExtCtrlsList(qCtrls);

  rc = set_roi_ctrls(fd_video, qCtrls);
  if (rc != 0) {
    printf("set_roi_ctrls returned %d \n", rc);
  }

  // Ext ctrl set/get for basic numeric values example
  wb_r_gain_ctrl = findQExtCtrl(qCtrls, "isp_wb_r_gain");
  if (!wb_r_gain_ctrl) {
    printf("failed to find isp_wb_r_gain control\n");
    return 1;
  }

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  int val = 0;
  ExtCtrl.id = wb_r_gain_ctrl->id;
  ExtCtrl.size = wb_r_gain_ctrl->elem_size * wb_r_gain_ctrl->elems;
  ExtCtrl.ptr = &val;

  set_ext_ctrls(fd_video, &ExtCtrl);
  rc = get_ext_ctrls(fd_video, wb_r_gain_ctrl->id, &ExtCtrl);
  if (!val)
    printf("isp_wb_r_gain is now: %d\n", ExtCtrl.value);
  else
    printf("get_ext_ctrls returned %d\n", rc);

  // Ext ctrl set/get for strings and arrays example
  wb_cc_matrix_ctrl = findQExtCtrl(qCtrls, "isp_wb_cc_matrix");
  if (!wb_cc_matrix_ctrl) {
    printf("failed to find isp_wb_cc_matrix control\n");
    return 1;
  }

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  ExtCtrl.id = wb_cc_matrix_ctrl->id;
  ExtCtrl.size = wb_cc_matrix_ctrl->elem_size * wb_cc_matrix_ctrl->elems;
  ExtCtrl.ptr = wb_matrix;
  rc = set_ext_ctrls(fd_video, &ExtCtrl);
  if (rc != 0) {
    printf("set_ext_ctrls returned %d", rc);
    return 1;
  }
  printf("set isp_wb_cc_matrix succeeded\n");

  rc = get_ext_ctrls(fd_video, wb_cc_matrix_ctrl->id, &ExtCtrl);
  if (!rc) {
    new_wb_matrix = (float *)ExtCtrl.ptr;
    printf(
        "get isp_wb_cc_matrix returned: {%f, %f, %f, %f, %f, %f, %f, %f, %f}\n",
        new_wb_matrix[0], new_wb_matrix[1], new_wb_matrix[2], new_wb_matrix[3],
        new_wb_matrix[4], new_wb_matrix[5], new_wb_matrix[6], new_wb_matrix[7],
        new_wb_matrix[8]);
    free(new_wb_matrix);
  } else {
    printf("get isp_wb_cc_matrix returned NULL\n");
  }

  // enable AF
  af_ext_ctrl = findQExtCtrl(qCtrls, "isp_af_enable");
  if (!af_ext_ctrl) {
    printf("failed to find isp_af_enable control\n");
    return 1;
  }

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  val = 0;
  ExtCtrl.id = af_ext_ctrl->id;
  ExtCtrl.size = af_ext_ctrl->elem_size * af_ext_ctrl->elems;
  ExtCtrl.ptr = &val;

  rc = set_ext_ctrls(fd_video, &ExtCtrl);
  if (rc != 0) {
    printf("set_ext_ctrls isp_af_enable returned %d", rc);
    return 1;
  }
  printf("set isp_af_enable succeeded\n");

  // Set AF Windows
  af_ext_ctrl = findQExtCtrl(qCtrls, "isp_af_window");
  if (!af_ext_ctrl) {
    printf("failed to find isp_af_window control\n");
    return 1;
  }

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  ExtCtrl.id = af_ext_ctrl->id;
  ExtCtrl.size = sizeof(af_windows);
  ExtCtrl.ptr = af_windows;

  rc = set_ext_ctrls(fd_video, &ExtCtrl);
  if (rc != 0) {
    printf("set_ext_ctrls isp_af_window returned %d", rc);
    return 1;
  }

  printf("set isp_af_window succeeded\n");

  rc = get_ext_ctrls(fd_video, af_ext_ctrl->id, &ExtCtrl);
  if (!rc) {
    new_af_windows = (int *)ExtCtrl.ptr;
    printf(
        "get isp_af_window returned: \nwindow1: h = %d,\tw = %d,\tx = %d,\ty = "
        "%d\nwindow2: "
        "h = %d,\tw = %d,\tx = %d,\ty = %d\nwindow3: h = %d,\tw = %d,\tx = "
        "%d,\ty "
        "= %d\n",
        new_af_windows[0], new_af_windows[1], new_af_windows[2],
        new_af_windows[3], new_af_windows[4], new_af_windows[5],
        new_af_windows[6], new_af_windows[7], new_af_windows[8],
        new_af_windows[9], new_af_windows[10], new_af_windows[11]);
    free(new_af_windows);
  } else {
    printf("get isp_af_window returned NULL\n");
  }

  // Set iris limits
  iris_limits_ext_ctrl = findQExtCtrl(qCtrls, "isp_ae_iris_limits");
  if (!iris_limits_ext_ctrl) {
    printf("failed to find isp_ae_iris_limits control\n");
    return 1;
  }
  
  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  ExtCtrl.id = iris_limits_ext_ctrl->id;
  ExtCtrl.size = sizeof(iris_limits);
  ExtCtrl.ptr = iris_limits;

  rc = set_ext_ctrls(fd_video, &ExtCtrl);
  if (rc != 0) {
    printf("set_ext_ctrls isp_ae_iris_limits returned %d", rc);
    return 1;
  }

  printf("set isp_ae_iris_limits succeeded\n");

  rc = get_ext_ctrls(fd_video, iris_limits_ext_ctrl->id, &ExtCtrl);
  if (!rc) {
    new_iris_limits = (uint32_t *)ExtCtrl.ptr;
    printf(
        "get isp_ae_iris_limits returned min %f, max %f\n", new_iris_limits[0]/100.0f, new_iris_limits[1]/100.0f);
    free(new_iris_limits);
  } else {
    printf("get isp_ae_iris_limits returned NULL\n");
  }

  // Get AF Measurement
  af_measure_ctrl = findQExtCtrl(qCtrls, "isp_af_measurement");
  if (!af_measure_ctrl) {
    printf("failed to find isp_af_measurement control\n");
    return 1;
  }

  memset(&ExtCtrl, 0, sizeof(ExtCtrl));
  ExtCtrl.id = af_measure_ctrl->id;
  ExtCtrl.size = af_measure_ctrl->elem_size * af_measure_ctrl->elems;

  rc = get_ext_ctrls(fd_video, af_measure_ctrl->id, &ExtCtrl);
  if (!rc) {
    new_af_measure = (uint32_t *)ExtCtrl.ptr;
    printf(
        "get isp_af_measurement returned:\nsum_a: %d, sum_b: %d, sum_c: %d\nlum_a: %d, lum_b: %d, lum_c: %d\n",
        new_af_measure[0], new_af_measure[1], new_af_measure[2],
        new_af_measure[3], new_af_measure[4], new_af_measure[5]);
    free(new_af_measure);
  } else {
    printf("get isp_af_measurement returned NULL\n");
  }

  close(fd_video);
  releaseQExtCtrls(qCtrls);
  printf("finished\n");
  return 0;
}
