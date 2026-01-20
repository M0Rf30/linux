# CAMSS Camera Streaming Fix - Manual Loop Restoration

## Problem Identified

The attempt to modernize CAMSS to use `v4l2_subdev_enable_streams()` API **failed** because:

1. **CAMSS subdevs don't implement `.enable_streams()` pad operation**
2. **CAMSS subdevs don't propagate `s_stream` calls down the pipeline**
3. Without propagation, only the first subdev (VFE) receives s_stream, and the sensor never powers on

## Solution Applied

**Restored the original manual while loop** in `drivers/media/platform/qcom/camss/camss-video.c`:

### Changes Made

#### video_start_streaming() - Lines 243-282
```c
static int video_start_streaming(struct vb2_queue *q, unsigned int count)
{
    struct camss_video *video = vb2_get_drv_priv(q);
    struct video_device *vdev = &video->vdev;
    struct media_entity *entity;
    struct media_pad *pad;
    struct v4l2_subdev *subdev;
    int ret;

    ret = video_device_pipeline_alloc_start(vdev);
    if (ret < 0) {
        dev_err(video->camss->dev, "Failed to start media pipeline: %d\n", ret);
        goto flush_buffers;
    }

    ret = video_check_format(video);
    if (ret < 0)
        goto error;

    /* Manual loop to call s_stream(1) on entire pipeline */
    entity = &vdev->entity;
    while (1) {
        pad = &entity->pads[0];
        if (!(pad->flags & MEDIA_PAD_FL_SINK))
            break;

        pad = media_pad_remote_pad_first(pad);
        if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
            break;

        entity = pad->entity;
        subdev = media_entity_to_v4l2_subdev(entity);

        ret = v4l2_subdev_call(subdev, video, s_stream, 1);
        if (ret < 0 && ret != -ENOIOCTLCMD)
            goto error;
    }

    return 0;

error:
    video_device_pipeline_stop(vdev);

flush_buffers:
    video->ops->flush_buffers(video, VB2_BUF_STATE_QUEUED);

    return ret;
}
```

#### video_stop_streaming() - Lines 285-312
```c
static void video_stop_streaming(struct vb2_queue *q)
{
    struct camss_video *video = vb2_get_drv_priv(q);
    struct video_device *vdev = &video->vdev;
    struct media_entity *entity;
    struct media_pad *pad;
    struct v4l2_subdev *subdev;

    /* Manual loop to call s_stream(0) on entire pipeline */
    entity = &vdev->entity;
    while (1) {
        pad = &entity->pads[0];
        if (!(pad->flags & MEDIA_PAD_FL_SINK))
            break;

        pad = media_pad_remote_pad_first(pad);
        if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
            break;

        entity = pad->entity;
        subdev = media_entity_to_v4l2_subdev(entity);

        v4l2_subdev_call(subdev, video, s_stream, 0);
    }

    video_device_pipeline_stop(vdev);

    video->ops->flush_buffers(video, VB2_BUF_STATE_ERROR);
}
```

## Why This Works

The manual loop walks the entire media pipeline:
1. **/dev/video0** → VFE → `s_stream(1)` called
2. **VFE** → CSID → `s_stream(1)` called  
3. **CSID** → CSIPHY → `s_stream(1)` called
4. **CSIPHY** → OV02A10 sensor → `s_stream(1)` called
5. **OV02A10 s_stream** → calls `pm_runtime_resume_and_get()` → powers regulators → enables sensor

Each subdev's s_stream implementation handles its hardware-specific operations.

## Build and Deploy

### Using pmbootstrap:
```bash
cd /home/gianluca/M0Rf30/linux

# Build kernel with pmbootstrap
pmbootstrap build linux-postmarketos-qcom-sdm660

# Flash to device
pmbootstrap flasher flash_kernel

# Or if doing full rootfs update
pmbootstrap install
pmbootstrap flasher flash_rootfs
```

### Manual build (if not using pmbootstrap):
```bash
cd /home/gianluca/M0Rf30/linux

# Build modules
make ARCH=arm64 O=.output LLVM=1 -j$(nproc) modules

# Build device tree
make ARCH=arm64 O=.output LLVM=1 dtbs

# Deploy to device (example)
scp .output/drivers/media/platform/qcom/camss/qcom_camss.ko root@172.16.42.1:/lib/modules/6.18.0-sdm660/
ssh root@172.16.42.1 'sync && reboot'
```

## Testing After Reboot

```bash
ssh root@172.16.42.1

# Clear kernel log
dmesg -C

# Test streaming
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=1 --stream-to=/tmp/capture.raw

# Check results
dmesg  # Should show no s_stream duplicate warnings
ls -lh /tmp/capture.raw  # Should be 2,400,000 bytes

# Verify sensor powered on during streaming
cat /sys/kernel/debug/regulator/regulator_summary | grep -A 2 '3-003d'
# Should show non-zero mA when streaming is active
```

## Expected Behavior

**Before streaming:**
- Sensor runtime PM: suspended
- Regulators: 0mA

**During streaming:**
- Sensor runtime PM: active
- cam_avdd_gpio: ~10-20mA
- 3-003d-dovdd: ~10mA  
- 3-003d-avdd: ~10-20mA

**Capture file:**
- Size: 2,400,000 bytes (packed 10-bit Bayer)
- Format: SBGGR10P (pBAA)
- Resolution: 1600x1200

## Files Modified

- `drivers/media/platform/qcom/camss/camss-video.c` - Restored manual s_stream loops

## Reference

This approach has been used in CAMSS since its initial mainline merge. Migrating to `v4l2_subdev_enable_streams()` requires implementing `.enable_streams()` in all CAMSS subdevs (VFE, CSID, CSIPHY), which is a larger refactoring effort beyond the scope of getting the OV02A10 camera working.
