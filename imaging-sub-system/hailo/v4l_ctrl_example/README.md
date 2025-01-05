# v4l_ctrl_example
## Code example using v4l2 Controls framework

This tool is a code example for using the v4l2 framework to get and set different controls in the isp controls API.
This is intended as a reference for impmelenting software to control the different isp configurations using v4l2 controls API. 

## Requirements

- Main Path video device exists (/dev/video0)
- A stream is already running

## Examples

- Set/get for the isp_wb_r_gain control
- Set/get for the isp_wb_cc_matrix control

> **Note - Auto algorithms might change the values that are set in the example, this means the get print might show a different value.**
## Usage
```sh
v4l_ctrl_example
```
