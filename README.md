# image_denoising

This repository contains ROS nodes for simple opencv based image denoising.

## Supported platforms

Currently only tested under ROS Noetic on Ubuntu 20.04.


## How to build
Create a workspace (``image_denoising_ws``), then clone this repo,
configure, and build:
```
mkdir -p ~/image_denoising_ws/src
cd ~/image_denoising_ws
git clone https://github.com/berndpfrommer/image_denoising.git src/image_denoising
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

## How to use

Pass the input image topic as ``image``, the output image topic
as ``denoised_image``, and the filter type as ``filter``:
```
roslaunch image_denoising denoiser_node.launch image:=/camera/image denoised_image:=/camera/denoised_image filter:=bilateral
```

Filters implemented so far (only for monochrome 8 bit uchar images):
- ``bilateral``: simple bilateral filtering, parameters:
    - ``sigma_color``: (default 80): width of gaussian kernel in luminance/color space
    - ``sigma_space``: (default 11): width of gaussian kernel in space (pixels)
- ``nl_means``: nonlinear means, parameters:
    - ``num_frames``: (odd, default 1): how many temporal frames to use. Will get very slow if >1.
	- ``template_window_size``: (default: 7) size of window for template
	- ``search_window_size``: (default: 21) range in which to shift the template
	- ``h``: (default: 20) the h parameter, see opencv documentation. The larger the stronger the denoising

## License

This software is issued under the Apache License Version 2.0.
