# v4l_event_handling_example
## Code example for handling stat events externally

This code example demonstrates how to subscribe to ISP stats events, in order to handle them externally.  
This option is available, if you prefer to add a custom handling for configuring ISP based on these stats.  

To replace the existing handling of the stats, 
you will also need to disable the appropriate auto algorithms in the `3aconfig.json` file.  

## Requirements

- Main Path video device exists (/dev/video0)
- A stream is already running

## Example behaviour
- Subscribes to some of the isp stats events (based on the hardcoded `bool events_to_handle[]` array)
- Polling for `VIDIOC_DQEVENT` event of type `HAILO15_UEVENT_ISP_STAT`
- Based on the event's `hailo15_event_stat_id`, we handle it accordingly
- Some statistic events do not require an additional request
    - For `HAILO15_UEVENT_VSM_DONE_STAT`, there is no additional data
    - For `HAILO15_UEVENT_SENSOR_DATALOSS_STAT`, the sensor index will be passed in the event's data directly
- For the rest of the statistics, an additional `VIDIOC_G_EXT_CTRLS` is called for the appropriate control.
    - A custom user handling of the statistic event is called. (In the example, only prints the checksum)

## Usage
```sh
v4l_event_handling_example
```
