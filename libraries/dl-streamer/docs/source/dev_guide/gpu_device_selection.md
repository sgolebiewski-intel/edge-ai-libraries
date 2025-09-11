# GPU device selection

This article describes how to select a GPU device on a multi-GPU system.

## 1. Media (VAAPI based) elements

[GStreamer](https://github.com/GStreamer/gstreamer-vaapi)
[VAAPI plugin][https://gstreamer.freedesktop.org/documentation/vaapi/index.html]
supports the `GST_VAAPI_DRM_DEVICE` environment variable, which allows
selecting GPU device for VAAPI elements (and `decodebin3` element in case
it internally works on VAAPI elements).

The `GST_VAAPI_DRM_DEVICE` environment variable expects the GPU device
driver path. The `/dev/dri/renderD128` path typically represents the first
GPU device on the system. `/dev/dri/renderD129` represents the second, etc.

For example, the following command forces VAAPI elements (and
`decodebin3`) to use the second GPU device:

```bash
export GST_VAAPI_DRM_DEVICE=/dev/dri/renderD129
```

## 2. Inference (OpenVINO™ based) elements

### Explicit selection

In case of video decode running on CPU and inference running on GPU, the
`device` property in inference elements enables you to select the GPU device
according to
[OpenVINO™ GPU device naming convention](https://docs.openvino.ai/2024/openvino-workflow/running-inference/inference-devices-and-modes/gpu-device.html#device-naming-convention)
, with devices enumerated as **GPU.0**, **GPU.1**, etc., for example:

```bash
gst-launch-1.0 "... ! decodebin3 ! gvadetect device=GPU.1 ! ..."
```

### Automatic selection

For running both video decode and inference on GPU, select the GPU
device by setting the environment variable for VAAPI decode element, and
setting `device=GPU` for all inference elements. It will enable inference
elements to query VAAPI context from VAAPI decode element and
automatically run inference and pre-processing on the same GPU device as
video decode (GPU device affinity). For example, to select the second GPU
device for decoding and inference:

```bash
export GST_VAAPI_DRM_DEVICE=/dev/dri/renderD129
gst-launch-1.0 "... ! decodebin3 ! gvadetect device=GPU ! ..."
```

## 3. Media and Inference elements for GStreamer 1.24.0 and later versions

> **NOTE:** Starting with
> [GStreamer 1.24 version](https://gstreamer.freedesktop.org/releases/1.24/),
> GStreamer-VAAPI should be considered deprecated in favor of the GstVA
> plugin. The `GST_VAAPI_ALL_DRIVERS` environment variable is deprecated in favor of
> `GST_VA_ALL_DRIVERS`.

As stated earlier, the GStreamer framework allows selecting the GPU
render device for VA codecs plugins if there is more than one GPU device
on the system.

For *single-GPU* device systems, the VA codecs plugin elements like,
vah264dec, vapostproc, etc., correspond to
`GPU (GPU.0) device -> /dev/dri/renderD128`

For *multi-GPU* device systems, each additional GPU device corresponds
to a separate DRI device. For example:
`GPU.1 -> /dev/dri/renderD129`, `GPU.2 -> /dev/dri/renderD130`, etc.

The command below lists the available VA codecs plugins on the system
for each GPU device:

```bash
gst-inspect-1.0 | grep va
. . .
va:  vah264dec: VA-API H.264 Decoder in Intel(R) Gen Graphics
va:  vapostproc: VA-API Video Postprocessor in Intel(R) Gen Graphics
. . .
va:  varenderD129h264dec: VA-API H.264 Decoder in Intel(R) Gen Graphics in renderD129
va:  varenderD129postproc: VA-API Video Postprocessor in Intel(R) Gen Graphics in renderD129
. . .
va:  varenderD130h265dec: VA-API H.265 Decoder in Intel(R) Gen Graphics in renderD130
va:  varenderD130postproc: VA-API Video Postprocessor in Intel(R) Gen Graphics in renderD130
```

Use the example below for **GPU.0** and the corresponding VA codec elements, e.g.
`vah264dec` and `vapostproc`:

```bash
gst-launch-1.0 filesrc location=${VIDEO_FILE} ! parsebin ! vah264dec ! vapostproc ! "video/x-raw(memory:VAMemory)" ! \
gvadetect model=${MODEL_FILE} device=GPU.0 pre-process-backend=va-surface-sharing batch_size=8 ! queue ! gvafpscounter ! fakesink
```

For GPU devices other than the default one, that is GPU or GPU.0, the
`renderD1XY` element component selects the assigned GPU device, for example:

- `GPU.1 -> varenderD129h264dec, varenderD129postproc`
- `GPU.2 -> varenderD130h264dec, varenderD130postproc`

Use the example below for **GPU.1** and the corresponding VA codec elements, e.g.
`varenderD129h264dec` and `varenderD129postproc`.

```bash
gst-launch-1.0 filesrc location=${VIDEO_FILE} ! parsebin ! varenderD129h264dec ! varenderD129postproc ! "video/x-raw(memory:VAMemory)" ! \
gvadetect model=${MODEL_FILE} device=GPU.1 pre-process-backend=va-surface-sharing batch_size=8 ! queue ! gvafpscounter ! fakesink
```
