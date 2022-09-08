# Line-scan camera control

<!---
![teaser figure](images/teaser.png)
**Line-scan camera control system:** From left to right--original image of patient; undistorted image; keypoint estimation using OpenPose; anthropometric measurements (projections). </p> 
--->

## Description

This repository contains the code for a program used two control two line-scan GigE cameras: a color camera (Teledyne Dalsa Spyder 3 GigE) and a 3D profiling camera (Automation Technology C2-2040 GigE). Unlike area-scan cameras, which capture a 2D image in every frame, line-scan cameras capture only a single line. The advantage of this type of cameras is that they can capture images of objects moving at very high speeds with very little motion blur. On the other hand, both cameras are also controlled by Gigabit Ethernet, which also has several advantages over consumer-oriented cameras. For example, they can be controlled remotely from a long distance, their images can be uploaded directly to a custom processing software, and they can use GigE Vision events, which are very useful for synchronizing multiple cameras and processing their images in real-time applications.

## Teledyne Dalsa Spyder 3 GigE

The Spyder 3 is a color camera with a maximum resolution of 2048 pixels that is able to acquire lines at a rate of up to 68 kHz, depending on the resolution and exposure time. The camera acquires images in the standard RGB color space and has dozens of settings that can be adjusted, and even modified during the acquisition process. In order to function properly, the Spyder 3 camera needs one or two high-power linear LEDs that illuminate the surface uniformly.

## Automation Technology C2-2040 GigE

The C2-2040 is a 3D profiling camera with a maximum resolution of 2048 pixels that is able to acquire lines at a rate of 3.3 kHz. This camera works together with a linear laser, and has a band-pass filter that attenuates light in other wavelengths besides the one from the laser. The camera has two operation modes: image and 3D mode. In 3D mode, the camera uses the triangulation principle to acquire 3D profiles from the laser distorsion with sub-pixel accuracy. In image mode, the camera is able to produce multiple images: a height image, a laser reflection image, and a laser-width distorsion image. Besides using the height image, which is useful for analyzing the geometry of objects, the other images can be used to analyze and classify materials based on their surface reflectance and scattering. This could be useful, for example, in ore sorting applications, where these properties are closely related to the abundance and size of crystals in the surface of rocks.

