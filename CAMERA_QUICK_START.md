# Camera Sensor Status Summary - Xiaomi Redmi Note 6 Pro (Tulip)

## Quick Reference

### Hardware (from STRIX kernel downstream DTS)
| Sensor | Use | Upstream Driver | Status | GPIOs | MCLK | Notes |
|--------|-----|-----------------|--------|-------|------|-------|
| **OV02A10** | Depth (2MP) | ✅ `ov02a10.c` | **READY** | 34,52,40 | MCLK2 24MHz | Full support |
| **S5K2L7** | Rear Main (12MP) | ⚠️ `s5k2xx.c` (as s5k2x7sp) | **TEST** | 32,48,50 | MCLK0 19-24MHz | May work |
| **S5K5E8** | Front (5MP) | ❌ None | **PORT** | 35,52,45 | MCLK3 24MHz | Needs driver |
| **S5K3T1** | Front Sec (13MP) | ❌ None | **PORT** | 33,47,GPIO3 | MCLK1 24MHz | Needs driver |

---

## Files Created

1. **CAMERA_DRIVER_STATUS.md** - Overview of upstream driver availability
2. **TULIP_CAMERA_REFERENCE.md** - Complete hardware reference from downstream kernel
3. **camera-sensors-tulip.dts.patch** - Device tree template for mainline integration

---

## Immediate Actions (OV02A10 - easiest to test)

### 1. Create Device Tree Node

Add to `arch/arm64/boot/dts/qcom/sdm636-xiaomi-tulip.dts`:

```dts
/* Add to root node */
/ {
    camera_ov02a10_clk: camera-ov02a10-clk {
        compatible = "fixed-clock";
        #clock-cells = <0>;
        clock-frequency = <24000000>;
    };
};

/* Add I2C node - need to verify which i2c bus */
&blsp_i2c2 {  /* or i2c3, check CCI mapping */
    status = "okay";
    
    ov02a10_depth: camera-sensor@3d {
        compatible = "ovti,ov02a10";
        reg = <0x3d>;
        
        /* Power supplies */
        dovdd-supply = <&vreg_l11a_1p8>;  /* PM660 L11 - 1.8V I/O */
        avdd-supply = <&cam_avdd_gpio_regulator>;  /* 2.8V via GPIO 46 */
        dvdd-supply = <&cam_avdd_gpio_regulator>;  /* Share with AVDD */
        
        /* Clock */
        clocks = <&camera_ov02a10_clk>;
        clock-names = "eclk";
        
        /* GPIOs from downstream */
        powerdown-gpios = <&tlmm 40 GPIO_ACTIVE_HIGH>;
        reset-gpios = <&tlmm 52 GPIO_ACTIVE_LOW>;
        
        /* CSI-2 port */
        port {
            ov02a10_ep: endpoint {
                link-frequencies = /bits/ 64 <390000000>;
                ovti,mipi-clock-voltage = <3>;
                remote-endpoint = <&csiphy1_ep>;  /* Connect to CAMSS CSIPHY1 */
                data-lanes = <1>;
            };
        };
    };
};

/* Add GPIO regulator */
&soc {
    cam_avdd_gpio_regulator: cam-avdd-regulator {
        compatible = "regulator-fixed";
        regulator-name = "cam_avdd_gpio";
        regulator-min-microvolt = <2800000>;
        regulator-max-microvolt = <2800000>;
        enable-active-high;
        gpio = <&tlmm 46 0>;
        vin-supply = <&vreg_bob>;  /* PM660L BOB */
    };
};
```

### 2. Enable Kernel Config

```bash
cd /home/gianluca/M0Rf30/linux
./scripts/config --module VIDEO_OV02A10
./scripts/config --enable VIDEO_V4L2_SUBDEV_API
./scripts/config --enable MEDIA_CONTROLLER
./scripts/config --module VIDEO_QCOM_CAMSS  # If not already enabled
```

### 3. Build and Test

```bash
make -j$(nproc) dtbs modules
# Install and reboot

# Check if driver loaded
dmesg | grep ov02a10
ls /sys/bus/i2c/drivers/ov02a10

# Check V4L2 devices
v4l2-ctl --list-devices
media-ctl -p
```

---

## Integration Strategy

### Stage 1: OV02A10 Only (1-2 days)
- Add device tree node with exact hardware from downstream
- Enable driver in kernel config
- Test sensor detection and basic functionality
- **Goal**: Verify CAMSS and sensor probe successfully

### Stage 2: S5K2L7 Experimental (1 week)
- Try with `samsung,s5k2x7sp` compatible string
- May need register adjustments
- Focus on rear main camera (most important)
- **Goal**: Get image capture working

### Stage 3: Port S5K5E8 (2-4 weeks)
- Extract register init sequences from downstream
- Port to mainline V4L2 framework
- Create device tree binding documentation
- **Goal**: Front camera working

### Stage 4: Port S5K3T1 (2-4 weeks)
- Similar process as S5K5E8
- Handle PM660L GPIO for VDIG control
- **Goal**: Complete camera support

---

## Mainline vs Downstream

### Downstream (STRIX Kernel - Android)
- Uses Qualcomm proprietary `MSMB_CAMERA` framework
- Sensors defined with `qcom,camera` and `qcom,eeprom`
- CCI (Camera Control Interface) - Qualcomm's custom I2C
- Drivers in `drivers/media/platform/msm/camera_v2/`

### Upstream (Mainline Linux)
- Uses standard V4L2 Media Controller framework
- Sensors as V4L2 subdevices with proper bindings
- Standard I2C or CCI mapped to I2C
- Drivers in `drivers/media/i2c/` and `drivers/media/platform/qcom/camss/`

---

## Common Issues & Solutions

### Issue: Sensor not detected on I2C
**Solution**: 
- Verify CCI → I2C bus mapping
- Check power sequence timing
- Ensure regulators are enabled
- Check GPIO polarity (ACTIVE_HIGH vs ACTIVE_LOW)

### Issue: CAMSS driver not loading
**Solution**:
- Enable CONFIG_VIDEO_QCOM_CAMSS
- Check SDM660 CAMSS support in mainline
- Verify clock and regulator definitions in DTS

### Issue: No image/black screen
**Solution**:
- Check CSIPHY port assignment
- Verify MIPI CSI-2 lane count matches sensor
- Check sensor register initialization
- Verify ISP (Image Signal Processor) pipeline

---

## Resources

### Upstream Kernel Paths
- OV02A10 driver: `drivers/media/i2c/ov02a10.c`
- S5K2XX driver: `drivers/media/i2c/s5k2xx.c`
- CAMSS driver: `drivers/media/platform/qcom/camss/`
- DT bindings: `Documentation/devicetree/bindings/media/`

### Downstream References
- STRIX kernel DTS: `~/Scaricati/STRIX_kernel_xiaomi-sdm660-sdm660-eas-test/arch/arm/boot/dts/qcom/xiaomi/tulip/`
- Camera sensor DTS: `tulip-camera-sensor-mtp.dtsi`
- Defconfig: `arch/arm64/configs/tulip_defconfig`

### Similar Devices
- Xiaomi Redmi Note 7 (Lavender) - SDM660 with upstream camera support
- Check existing SDM660 camera implementations in mainline

---

## Testing Tools

```bash
# List media devices
media-ctl -d /dev/media0 -p

# List video devices
v4l2-ctl --list-devices

# Capture test image
v4l2-ctl --device /dev/video0 --stream-mmap --stream-to=test.raw

# Configure media pipeline (example)
media-ctl -d /dev/media0 --set-v4l2 '"ov02a10 0-003d":0[fmt:SBGGR10_1X10/1600x1200]'

# GStreamer test (if working)
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```

---

## Success Criteria

### Minimum (OV02A10)
- ✅ Sensor probes on I2C
- ✅ CAMSS driver loads
- ✅ V4L2 subdevice appears
- ✅ Can configure media pipeline

### Good (OV02A10 + S5K2L7)
- ✅ Can capture raw frames
- ✅ Rear main camera functional
- ✅ Depth sensor functional

### Complete (All sensors)
- ✅ All 4 cameras working
- ✅ Autofocus working (S5K2L7)
- ✅ Flash LED working
- ✅ Userspace camera app functional

---

## Timeline Estimate

| Task | Time | Difficulty |
|------|------|------------|
| OV02A10 integration | 1-3 days | Easy |
| S5K2L7 experimental | 1 week | Medium |
| S5K5E8 driver port | 2-4 weeks | Hard |
| S5K3T1 driver port | 2-4 weeks | Hard |
| Full integration & testing | 1 week | Medium |
| **Total** | **6-11 weeks** | - |

---

## Questions to Research

1. ✅ Does mainline kernel support SDM660 CAMSS? 
   - Check: `drivers/media/platform/qcom/camss/camss-sdm660.c` or similar

2. ✅ How is CCI mapped to I2C in mainline?
   - Check: CAMSS driver CCI I2C master implementation

3. ✅ Are there existing SDM660 camera device trees in mainline?
   - Check: `arch/arm64/boot/dts/qcom/sdm660-*.dts`

4. ⚠️ Does s5k2xx driver actually support s5k2l7?
   - **Action**: Test with `samsung,s5k2x7sp` compatible string
   - May need register map verification

---

## Contact / Support

- **PostmarketOS**: Community for mainline device support
- **LineageOS**: May have camera HAL references
- **MSM8953 Mainline**: barni2000's repository has SDM660 camera work

Good luck! Start with OV02A10 first - it's the easiest win.
