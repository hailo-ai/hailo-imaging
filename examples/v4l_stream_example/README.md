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
- **HDR Support**: Supports HDR (High Dynamic Range) mode with different DOL configurations.
- **Raw Frames Capture**: Capture frames from a raw image video device (`/dev/video2`)
- **Customization**: Configure the number of frames to capture, number of buffers to use, requested FPS, width, height, and more.
- **Save Option**: Option to save raw captures.

## Parameters

- **width** (*int*): Width of the output frame. Default: 3840.
- **height** (*int*): Height of the output frame. Default: 2160.
- **format** (*str*): Format of the output frame.
  - **SDR formats**: nv12, rgb, yuy2, raw_rggb12p, raw_rggb12, raw_gbrg12
  - **HDR formats**: nv12, rgb, yuy2, raw_rggb12p_3dol, raw_rggb12_3dol, raw_rggb12p_2dol, raw_rggb12_2dol, raw_gbrg12_2dol
  - Default: nv12.
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
v4l_stream_example --device=/dev/video2 --type=capture --format=raw_rggb12p --num-frames=400
```
This command captures 400 raw frames of format rggb12p from the raw image video device `/dev/video2`.
The outputs will be stored as separate files (name format is hardcoded, starting with `out_0.raw`).

## HDR Raw Capture

HDR (High Dynamic Range) mode allows capturing multiple exposures in a single frame to extend the dynamic range. The tool supports different DOL configurations for HDR raw capture:

### DOL Modes

- **2 DOL**: Captures 2 exposures (Long and Short) in a single frame
  - Formats: `raw_rggb12p_2dol`, `raw_rggb12_2dol`, `raw_gbrg12_2dol`
  - Plane count: 2 planes (one per exposure)

- **3 DOL**: Captures 3 exposures (Long, Medium, and Short) in a single frame
  - Formats: `raw_rggb12p_3dol`, `raw_rggb12_3dol`
  - Plane count: 3 planes (one per exposure)

### HDR Raw Capture Usage

When capturing HDR raw frames, the format name must explicitly specify the DOL mode:
- For 2 DOL: Use `raw_rggb12p_2dol` or `raw_rggb12_2dol`
- For 3 DOL: Use `raw_rggb12p_3dol` or `raw_rggb12_3dol`

Example for 2 DOL HDR capture:
```sh
v4l_stream_example --device=/dev/video2 --type=capture --format=raw_rggb12p_2dol --num-frames=30 --width=3840 --height=2160
```

Example for 3 DOL HDR capture:
```sh
v4l_stream_example --device=/dev/video2 --type=capture --format=raw_rggb12p_3dol --num-frames=30 --width=3840 --height=2160
```

### Understanding HDR Raw Output

When capturing HDR raw frames:
- Each captured frame contains multiple planes (2 for 2 DOL, 3 for 3 DOL)
- Each plane represents a different exposure level
- The planes are written sequentially to the output file
- For 2 DOL: Plane 0 = Long exposure, Plane 1 = Short exposure
- For 3 DOL: Plane 0 = Long exposure, Plane 1 = Short exposure, Plane 2 = Very Short exposure

**Note**: HDR mode is automatically detected from the sensor configuration. The tool will use HDR format arrays when HDR is enabled on the sensor.
