# v4l_stream_example
## Description

The v4l_stream_example is a tool written in C for capturing video frames from a V4L2 compatible device.
It provides options to capture frames in various formats and perform tests,
like capturing frames and extracting VSM data.

## Requirements

- V4L2 compatible video device
- Enough space to save the captured files, if capture mode is selected

## Features

- **Test Modes**: Run different test modes including frame capture and VSM data extraction.
- **Format Support**: Supports various frame formats including raw, nv12, rgb, and yuy2.
- **Raw Frames Capture**: Capture frames from a raw image video device (`/dev/video2`)
- **Customization**: Configure the number of frames to capture, number of buffers to use, requested FPS, width, height, and more.
- **Save Option**: Option to save raw captures.

## Parameters

- **width** (*int*): Width of the output frame. Default: 3840.
- **height** (*int*): Height of the output frame. Default: 2160.
- **format** (*str*): Format of the output frame. Currently supported formats: nv12, rgb, yuy2. Default: nv12.
- **device** (*str*): Path of the video device to test. Default: /dev/video0.
- **out-path** (*str*): File path for the captured frames. Default: capture.out.
- **vsm-out-path** (*str*): File path for VSM (Visual Slam Module) data output. Ignored unless the test type is vsm-test. Default: vsm.out.
- **type** (*str*): Type of test to run. Currently supported types: capture, vsm-test. Default: capture.
- **num-frames** (*int*): Number of frames to capture. Ignored unless the test type is capture. Default: 30.
- **num-buffers** (*int*): Number of buffers to use when capturing. Ignored unless the test type is capture. Default: NUM_BUFFERS.
- **save** (*int*): Option to save raw captures. Default: 1.
- **fps** (*int*): Requested frames per second (fps). Default: 30.

## Usage

### Help

For detailed usage instructions, run the following command:

```sh
v4l_stream_example --help
```

### Basic Usage

To capture frames from a video device, use the following command:

```sh
v4l_stream_example --type=capture --width=3840 --height=2160 --format=nv12 --device=/dev/video0 --out-path=capture.out --num-frames=30 --num-buffers=5 --save=1 --fps=30
```
This command captures 30 frames in nv12 format with a resolution of 3840x2160 pixels from `/dev/video0`.
It saves the captured frames to `capture.out` and uses 5 buffers for capturing with a requested frame rate of 30 frames per second.

### VSM Test

To run a VSM test and extract VSM data along with capturing frames, use the following command:

```sh
v4l_stream_example --type=vsm-test --width=3840 --height=2160 --format=nv12 --device=/dev/video0 --out-path=capture.out --vsm-out-path=vsm.out --num-frames=30 --num-buffers=5 --fps=30
```
This command captures 30 frames in nv12 format with a resolution of 3840x2160 pixels from `/dev/video0`.
It saves the captured frames to `capture.out` and extracts VSM data to `vsm.out`.

### Raw Capture Example

Before running the tool for raw capture, ensure that the sensor is preconfigured with the desired parameters.
For example:
```sh
hailo_ctrl -d $(find_subdevice_path.sh imx) -c exposure=150000
```
Then, run the raw capture tool with the following command:
```sh
v4l_stream_example --device=/dev/video2 --type=capture --format=raw --num-frames=400
```
This command captures 400 raw frames from the raw image video device `/dev/video2`.
The outputs will be stored as separate files (name format is hardcoded, starting with `out_0.raw`).
