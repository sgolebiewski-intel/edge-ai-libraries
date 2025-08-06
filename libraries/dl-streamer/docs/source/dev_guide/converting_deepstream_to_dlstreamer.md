# Converting NVIDIA DeepStream Pipelines to Intel® Deep Learning Streamer (Intel® DL Streamer) Pipeline Framework

This document describes the steps to convert a pipeline from NVIDIA
DeepStream to Intel® DL Streamer Pipeline Framework. We also have a
running example through the document that will be updated at each step
to help show the modifications being described.

::: note
::: title
Note
:::

The intermediate steps of the pipeline are not meant to run. They are
simply there as a reference example of the changes being made in each
section.
:::

## Contents

-   `Preparing Your Model <preparing_model>`{.interpreted-text
    role="ref"}
-   `Configuring Model for Intel® DL Streamer <configure_model>`{.interpreted-text
    role="ref"}
-   `GStreamer Pipeline Adjustments <gstreamer_pipeline_adj>`{.interpreted-text
    role="ref"}
-   `Mux and Demux Elements <mux_demux_elements>`{.interpreted-text
    role="ref"}
-   `Inferencing Elements <inferencing_elements>`{.interpreted-text
    role="ref"}
-   `Video Processing Elements <video_processing_elements>`{.interpreted-text
    role="ref"}
-   `Metadata Elements <metadata_elements>`{.interpreted-text
    role="ref"}
-   `Multiple Input Streams <multiple-input-streams>`{.interpreted-text
    role="ref"}
-   `DeepStream to DLStreamer Elements Mapping Cheetsheet <mapping_cheetsheet>`{.interpreted-text
    role="ref"}

------------------------------------------------------------------------

To use Intel® DL Streamer Pipeline Framework and OpenVINO™ Toolkit the
model needs to be in Intermediate Representation (IR) format. To convert
your model to this format, please follow
`model preparation <model_preparation>`{.interpreted-text role="doc"}
steps.

------------------------------------------------------------------------

NVIDIA DeepStream uses a combination of model configuration files and
DeepStream element properties to specify interference actions as well as
pre- and post-processing steps before/after running inference as
documented here:
[here](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvinfer.html).

Similarly, Intel® DL Streamer Pipeline Framework uses GStreamer element
properties for inference settings and
`model proc <model_proc_file>`{.interpreted-text role="doc"} files for
pre- and post-processing steps.

The following table shows how to map commonly used NVIDIA DeepStream
configuration properties to Intel® DL Streamer settings.

+-------------+-------------+-------------+-------------+-------------+
| NVIDIA      | NVIDIA      | Intel® DL   | Intel® DL   | Description |
| DeepStream  | DeepStream  | Streamer    | Streamer    |             |
| config file | element     | model proc  | element     |             |
|             | property    | file        | property    |             |
+=============+=============+=============+=============+=============+
| model-      | model-      |             | model       | Path to     |
| engine-file | engine-file |             | \<path\>    | inference   |
| \<path\>    | \<path\>    |             |             | model       |
|             |             |             |             | network     |
|             |             |             |             | file.       |
+-------------+-------------+-------------+-------------+-------------+
| lab         |             |             | labels-file | Path to     |
| elfile-path |             |             | \<path\>    | .txt file   |
| \<path\>    |             |             |             | containing  |
|             |             |             |             | object      |
|             |             |             |             | classes.    |
+-------------+-------------+-------------+-------------+-------------+
| n           |             |             | | gvadetect | Type of     |
| etwork-type |             |             |   for       | inference   |
| \<0..3\>    |             |             |             | operation.  |
|             |             |             |  detection, |             |
|             |             |             |   instance  |             |
|             |             |             |   s         |             |
|             |             |             | egmentation |             |
|             |             |             | |           |             |
|             |             |             | gvaclassify |             |
|             |             |             |   for       |             |
|             |             |             |   clas      |             |
|             |             |             | sification, |             |
|             |             |             |   semantic  |             |
|             |             |             |   s         |             |
|             |             |             | egmentation |             |
+-------------+-------------+-------------+-------------+-------------+
| batch-size  | batch-size  |             | batch-size  | Number of   |
| \<N\>       | \<N\>       |             | \<N\>       | frames      |
|             |             |             |             | batched     |
|             |             |             |             | together    |
|             |             |             |             | for a       |
|             |             |             |             | single      |
|             |             |             |             | inference.  |
+-------------+-------------+-------------+-------------+-------------+
| maintain-a  |             | resize:     |             | Number of   |
| spect-ratio |             | a           |             | frames      |
|             |             | spect-ratio |             | batched     |
|             |             |             |             | together    |
|             |             |             |             | for a       |
|             |             |             |             | single      |
|             |             |             |             | inference.  |
+-------------+-------------+-------------+-------------+-------------+
| num-detec   |             |             |             | Number of   |
| ted-classes |             |             |             | classes     |
|             |             |             |             | detected by |
|             |             |             |             | the model,  |
|             |             |             |             | inferred    |
|             |             |             |             | from label  |
|             |             |             |             | file by     |
|             |             |             |             | Intel® DL   |
|             |             |             |             | Streamer.   |
+-------------+-------------+-------------+-------------+-------------+
| interval    | interval    |             | inferen     | Inference   |
| \<N\>       | \<N\>       |             | ce-interval | action      |
|             |             |             | \<N+1\>     | executed    |
|             |             |             |             | every Nth   |
|             |             |             |             | frame,      |
|             |             |             |             | please note |
|             |             |             |             | Intel® DL   |
|             |             |             |             | Streamer    |
|             |             |             |             | value is    |
|             |             |             |             | greater by  |
|             |             |             |             | 1.          |
+-------------+-------------+-------------+-------------+-------------+
|             | threshold   |             | threshold   | Threshold   |
|             |             |             |             | for         |
|             |             |             |             | detection   |
|             |             |             |             | results.    |
+-------------+-------------+-------------+-------------+-------------+

------------------------------------------------------------------------

In the following sections we will be converting DeepStream pipeline to
Pipeline Framework. The DeepStream pipeline is taken from one of the
examples
[here](https://github.com/NVIDIA-AI-IOT/deepstream_reference_apps). It
reads a video stream from the input file, decodes it, runs inference,
overlays the inferences on the video, re-encodes and outputs a new .mp4
file.

``` shell
filesrc location=input_file.mp4 ! decodebin3 ! \
nvstreammux batch-size=1 width=1920 height=1080 ! queue ! \
nvinfer config-file-path=./config.txt ! \
nvvideoconvert ! "video/x-raw(memory:NVMM), format=RGBA" ! \
nvdsosd ! queue ! \
nvvideoconvert ! "video/x-raw, format=I420" ! videoconvert ! avenc_mpeg4 bitrate=8000000 ! qtmux ! filesink location=output_file.mp4
```

The below mapping represents the typical changes that need to be made to
the pipeline to convert it to Intel® DL Streamer Pipeline Framework. The
pipeline is broken down into sections based on the elements used in the
pipeline.

![image](deepstream_mapping_dlstreamer.png)

The next chapters give more details on how to replace each element.

------------------------------------------------------------------------

-   Remove `nvstreammux` and `nvstreamdemux` and all their properties.
    -   These elements combine multiple input streams into a single
        batched video stream (NVIDIA-specific). Intel® DL Streamer takes
        a different approach: it employs generic GStreamer syntax to
        define parallel streams. The cross-stream batching happens at
        the inferencing elements by setting the same `model-instance-id`
        property.
    -   In this example, there is only one video stream so we can skip
        this for now. See more on how to construct multi-stream
        pipelines in the following section
        `Multiple Input Streams <multiple-input-streams>`{.interpreted-text
        role="ref"} below.

At this stage we have removed `nvstreammux` and the `queue` that
followed it. Notably, the `batch-size` property is also removed. It will
be added in the next section as a property of the Pipeline Framework
inference elements.

``` shell
filesrc location=input_file.mp4 ! decodebin3 ! \
nvinfer config-file-path=./config.txt ! \
nvvideoconvert ! "video/x-raw(memory:NVMM), format=RGBA" ! \
nvdsosd ! queue ! \
nvvideoconvert ! "video/x-raw, format=I420" ! videoconvert ! avenc_mpeg4 bitrate=8000000 ! qtmux ! filesink location=output_file.mp4
```

------------------------------------------------------------------------

-   Remove `nvinfer` and replace it with `gvainference`, `gvadetect` or
    `gvaclassify` depending on the following use cases:
    -   For doing detection on full frames and outputting a region of
        interest, use
        `gvadetect <../elements/gvadetect>`{.interpreted-text
        role="doc"}. This replaces `nvinfer` when it is used in primary
        mode.
        -   Replace `config-file-path` property with `model` and
            `model-proc`.
        -   `gvadetect` generates GstVideoRegionOfInterestMeta.
    -   For doing classification on previously detected objects, use
        `gvaclassify <../elements/gvaclassify>`{.interpreted-text
        role="doc"}. This replaces nvinfer when it is used in secondary
        mode.
        -   Replace `config-file-path` property with `model` and
            `model-proc`.
        -   `gvaclassify` requires GstVideoRegionOfInterestMeta as
            input.
    -   For doing generic full frame inference, use
        `gvainference <../elements/gvainference>`{.interpreted-text
        role="doc"}. This replaces `nvinfer` when used in primary mode.
        -   `gvainference` generates GstGVATensorMeta.

In this example we will use gvadetect to infer on the full frame and
output region of interests. `batch-size` was also added for consistency
with what was removed above (the default value is 1 so it is not
needed). We replaced `config-file-path` property with `model` and
`model-proc` properties as described in "Configuring Model for Intel® DL
Streamer" above.

``` shell
filesrc location=input_file.mp4 ! decodebin3 ! \
gvadetect model=./model.xml model-proc=./model_proc.json batch-size=1 ! queue ! \
nvvideoconvert ! "video/x-raw(memory:NVMM), format=RGBA" ! \
nvdsosd ! queue ! \
nvvideoconvert ! "video/x-raw, format=I420" ! videoconvert ! avenc_mpeg4 bitrate=8000000 ! qtmux ! filesink location=output_file.mp4
```

------------------------------------------------------------------------

-   Replace NVIDIA-specific video processing elements with native
    GStreamer elements.
    -   `nvvideoconvert` with `vapostproc` (GPU) or `videoconvert`
        (CPU).
        -   If the `nvvideoconvert` is being used to convert to/from
            `memory:NVMM` it can just be removed.
    -   `nvv4ldecoder` can be replaced with `va{CODEC}dec`, for example
        `vah264dec` for decode only. Alternatively, the native GStreamer
        element `decodebin3` can be used to automatically choose an
        available decoder.
-   Some caps filters that follow an inferencing element may need to be
    adjusted or removed. Pipeline Framework inferencing elements do not
    support color space conversion in post-processing. You will need to
    have a `vapostproc` or `videoconvert` element to handle this.

Here we removed a few caps filters and instances of `nvvideoconvert`
used for conversions from DeepStream's NVMM because Pipeline Framework
uses standard GStreamer structures and memory types. We will leave the
standard gstreamer element `videoconvert` to do color space conversion
on CPU, however if available, we suggest using `vapostproc` to run on
Intel Graphics. Also, we will use the GStreamer standard element
`decodebin` to choose an appropriate demuxer and decoder depending on
the input stream as well as what is available on the system.

``` shell
filesrc location=input_file.mp4 ! decodebin3 ! \
gvadetect model=./model.xml model-proc=./model_proc.json batch-size=1 ! queue ! \
nvdsosd ! queue ! \
videoconvert ! avenc_mpeg4 bitrate=8000000 ! qtmux ! filesink location=output_file.mp4
```

------------------------------------------------------------------------

-   Replace `nvtracker` with
    `gvatrack <../elements/gvatrack>`{.interpreted-text role="doc"}
    -   Remove `ll-lib-file` property. Optionally replace with
        `tracking-type` if you want to specify the algorithm used. By
        default it will use the 'short-term' tracker.
    -   Remove all other properties.
-   Replace `nvdsosd` with
    `gvawatermark <../elements/gvawatermark>`{.interpreted-text
    role="doc"}
    -   Remove all properties
-   Replace `nvmsgconv` with
    `gvametaconvert <../elements/gvametaconvert>`{.interpreted-text
    role="doc"}
    -   `gvametaconvert` can be used to convert metadata from
        inferencing elements to JSON and to output metadata to the
        GST_DEBUG log.
    -   It has optional properties to configure what information goes
        into the JSON object including frame data for frames with no
        detections found, tensor data, the source the inferences came
        from, and tags, a user defined JSON object that is attached to
        each output for additional custom data.
-   Replace `nvmsgbroker` with
    `gvametapublish <../elements/gvametapublish>`{.interpreted-text
    role="doc"}
    -   `gvametapublish` can be used to output the JSON messages
        generated by `gvametaconvert` to stdout, file, MQTT or Kafka.

The only metadata processing that is done in this pipeline is to overlay
the inferences on the video for which we use `gvawatermark`.

``` shell
filesrc location=input_file.mp4 ! decodebin3 ! \
gvadetect model=./model.xml model-proc=./model_proc.json batch-size=1 ! queue ! \
gvawatermark ! queue ! \
videoconvert ! avenc_mpeg4 bitrate=8000000 ! qtmux ! filesink location=output_file.mp4
```

------------------------------------------------------------------------

| Unlike DeepStream, where all sources need to be linked to the sink
  pads of the `nvstreammux` element, Pipeline Framework uses existing
  GStreamer mechanisms to define multiple parallel video processing
  streams. This approach allow to reuse native GStreamer elements within
  the pipeline. The input stream can share same Inference Engine if they
  have same `model-instance-id` property. This allows creating inference
  batching across streams.

| For DeepStream, the simple pipeline involving two streams would look
  like code snippet below. The first line defines a common inference
  element for two (muxed and batched) streams. The second line defines
  per-stream input operations prior to muxing. The third line defines
  per-stream output operations after de-muxing.

``` shell
nvstreammux ! nvinfer batch-size=2 config-file-path=./config.txt ! nvstreamdemux \ 
filesrc ! decode ! mux.sink_0 filesrc ! decode ! mux.sink_1 \
demux.src_0 ! encode ! filesink demux.src_1 ! encode ! filesink
```

When using Pipeline Framework, the command line defines operations for
two parallel streams using native GStreamer syntax. By setting
`model-instance-id` to the same value, both streams share the same
instance `gvadetect` element. Hence, the shared inference parameters
(model, batch size, \...) can be defined only in the first line.

``` shell
filesrc ! decode ! gvadetect model-instance-id=model1 model=./model.xml batch-size=2 ! encode ! filesink \ 
filesrc ! decode ! gvadetect model-instance-id=model1 ! encode ! filesink
```

------------------------------------------------------------------------

Below table lists quick reference for mapping typical DeepStream
elements to Intel® DL Streamer elements or GStreamer.

  DeepStream Element                                                                                           DLStreamer Element
  ------------------------------------------------------------------------------------------------------------ -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  [nvinfer](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvinfer.html)           `gvadetect <../elements/gvadetect>`{.interpreted-text role="doc"}, `gvaclassify <../elements/gvaclassify>`{.interpreted-text role="doc"}, `gvainference <../elements/gvainference>`{.interpreted-text role="doc"}
  [nvdsosd](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvdsosd.html)           `gvawatermark <../elements/gvawatermark>`{.interpreted-text role="doc"}
  [nvtracker](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvtracker.html)       `gvatrack <../elements/gvatrack>`{.interpreted-text role="doc"}
  [nvmsgconv](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvmsgconv.html)       `gvametaconvert <../elements/gvametaconvert>`{.interpreted-text role="doc"}
  [nvmsgbroker](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvmsgbroker.html)   `gvametapublish <../elements/gvametapublish>`{.interpreted-text role="doc"}

  DeepStream Element   GStreamer Element
  -------------------- -------------------
  nvvideoconvert       videoconvert
  nvv4l2decoder        decodebin3
  nvv4l2h264dec        vah264dec
  nvv4l2h265dec        vah265dec
  nvv4l2h264enc        va264enc
  nvv4l2h265enc        va265enc
  nvv4l2vp8dec         vavp8dec
  nvv4l2vp9dec         vavp9dec
  nvv4l2vp8enc         vavp8enc
  nvv4l2vp9enc         vavp9enc
