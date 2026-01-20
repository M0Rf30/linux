# Camera Driver Status for Xiaomi Redmi Note 6 Pro (Tulip) - SDM636
## Branch: qcom-sdm660-6.18.y

## Device Camera Hardware
Based on https://deviceinfohw.ru/devices/item.php?item=23777

| Sensor | Module | Type | Resolution | Status |
|--------|--------|------|------------|--------|
| **OV02A10** | Sunny | Depth | 2MP | ✅ **Full upstream support** |
| **S5K2L7** | Ofilm | Main Rear | ~12MP | ⚠️ **Partial** (may work as s5k2x7sp) |
| **S5K5E8** | Ofilm | Front | 5MP | ❌ **No driver** - needs porting |
| **S5K3T1** | Sunny | Secondary | ~13MP | ❌ **No driver** - needs porting |

---

## Upstream Driver Analysis

### ✅ Available Upstream Drivers (from msm8953-mainline/linux barni2000/6.18/develop)

#### 1. OV02A10 - **READY TO USE**
- **Driver**: `drivers/media/i2c/ov02a10.c`
- **Kconfig**: `CONFIG_VIDEO_OV02A10`
- **Binding**: `Documentation/devicetree/bindings/media/i2c/ovti,ov02a10.yaml`
- **Compatible**: `ovti,ov02a10`
- **Requirements**:
  - 3 power supplies: `dovdd-supply`, `avdd-supply`, `dvdd-supply`
  - External clock (`eclk`)
  - `powerdown-gpios` (active high)
  - `reset-gpios` (active low)
  - MIPI CSI-2 link-frequencies (default: 390MHz)
  - Supports 1 data lane

#### 2. S5K2XX Family Driver - **MAY WORK**
- **Driver**: `drivers/media/i2c/s5k2xx.c`
- **Kconfig**: `CONFIG_VIDEO_S5K2XX`
- **Supported sensors**:
  - `samsung,s5k3l8`
  - `samsung,s5k3p8sp`
  - `samsung,s5k2p6sx`
  - **`samsung,s5k2x7sp`** ← Closest match for S5K2L7
- **Note**: S5K2L7 may be compatible with s5k2x7sp driver (same family)
- **Requirements**:
  - Power supplies: `vdda-supply`, `vddd-supply`, `vddio-supply`
  - External clock (`extclk`)
  - `reset-gpios`
  - MIPI CSI-2 (typically 4 data lanes for main cameras)

---

## ❌ Missing Upstream Drivers

### 3. Samsung S5K5E8 - **NEEDS DRIVER PORT**
- No upstream kernel driver available
- Must be ported from:
  - Downstream Xiaomi/Qualcomm kernel
  - Similar sensor drivers (s5k5e2, s5k5e8yx, etc.)
- Typical 5MP front camera sensor
- Driver structure similar to other Samsung sensors

### 4. Samsung S5K3T1 - **NEEDS DRIVER PORT**
- No upstream kernel driver available
- Must be ported from downstream kernel
- Likely 13MP sensor based on naming convention
- May require significant driver development

---

## Integration Checklist

### Phase 1: Enable Available Drivers

**1. Kernel Configuration**
Add to defconfig or enable in menuconfig:
```kconfig
CONFIG_VIDEO_OV02A10=m
CONFIG_VIDEO_S5K2XX=m
CONFIG_MEDIA_CONTROLLER=y
CONFIG_VIDEO_V4L2_SUBDEV_API=y
```

**2. Verify CAMSS Support**
Ensure SDM660/636 CAMSS driver is enabled:
```kconfig
CONFIG_VIDEO_QCOM_CAMSS=m
```

**3. Device Tree Work**
- [ ] Find actual GPIO assignments from downstream DTS
- [ ] Map PM660/PM660L regulators for camera power rails
- [ ] Verify I2C bus assignments (likely I2C2/3 for rear, I2C4/5 for front)
- [ ] Determine correct CSIPHY port assignments (0-2)
- [ ] Add camera sensor nodes to `sdm636-xiaomi-tulip.dts`

**Example GPIO search in downstream kernel:**
```bash
# Look for camera GPIO definitions
grep -r "camera.*gpio\|cam.*gpio" downstream_kernel/arch/arm64/boot/dts/qcom/sdm636-tulip*
grep -r "ov02a10\|s5k2l7\|s5k5e8\|s5k3t1" downstream_kernel/arch/arm64/boot/dts/
```

**Example regulator mapping:**
```bash
# Find camera regulator assignments
grep -r "cam.*-supply\|camera.*regulator" downstream_kernel/arch/arm64/boot/dts/qcom/sdm636-tulip*
```

---

### Phase 2: Port Missing Drivers

**For S5K5E8 (5MP Front Camera):**
1. Locate downstream driver source:
   ```bash
   find downstream_kernel/drivers/media -name "*s5k5e8*"
   ```
2. Port driver to mainline V4L2 framework
3. Create device tree binding document
4. Add to `drivers/media/i2c/Kconfig` and `Makefile`
5. Test with mainline CAMSS driver

**For S5K3T1 (Secondary Camera):**
1. Locate downstream driver source
2. Same porting process as S5K5E8
3. May require more work if heavily customized

---

## Testing Plan

### Stage 1: OV02A10 (Easiest)
1. Add device tree node with correct GPIOs and regulators
2. Enable `CONFIG_VIDEO_OV02A10=m`
3. Load driver: `modprobe ov02a10`
4. Check kernel logs: `dmesg | grep ov02a10`
5. Verify V4L2 device: `v4l2-ctl --list-devices`
6. Test with: `media-ctl`, `v4l2-ctl`, or `gstreamer`

### Stage 2: S5K2L7 (Experimental)
1. Try with `samsung,s5k2x7sp` compatible string
2. May need register map adjustments if incompatible
3. Test image capture and verify resolution

### Stage 3: Port S5K5E8 and S5K3T1
1. Port drivers from downstream
2. Integrate with mainline CAMSS
3. Full camera stack testing

---

## Downstream Kernel Resources

To get accurate GPIO/regulator/I2C information, check:
1. **Xiaomi kernel source** (if available)
2. **CAF (Code Aurora Forum) SDM660 trees**
3. **LineageOS/PostmarketOS device trees for Tulip**
4. **Downstream DTS files**: 
   - `arch/arm64/boot/dts/qcom/sdm636-tulip.dtsi`
   - `arch/arm64/boot/dts/qcom/sdm636-camera-sensor-mtp.dtsi`

---

## Reference Files Generated

1. **camera-sensors-tulip.dts.patch** - Device tree template with:
   - OV02A10 definition
   - S5K2L7 definition (experimental)
   - Placeholder GPIOs and regulators
   - CAMSS endpoint configuration
   - TODO markers for missing information

---

## Next Steps

1. **Extract downstream device tree**:
   ```bash
   # If you have downstream kernel source
   grep -A 50 "camera\|cci\|camss" downstream/arch/arm64/boot/dts/qcom/*tulip*
   ```

2. **Verify branch has CAMSS driver**:
   ```bash
   ls drivers/media/platform/qcom/camss/
   ```

3. **Check existing camera examples** in your kernel:
   ```bash
   grep -r "ovti,ov02a10" arch/arm64/boot/dts/
   ```

4. **Enable drivers and test OV02A10 first** (has full support)

---

## Summary

**Working now:** 
- OV02A10 (2MP depth) - Full support, just needs DT integration

**May work with tuning:**
- S5K2L7 (12MP main) - Try s5k2x7sp driver, may need adjustments

**Requires driver porting:**
- S5K5E8 (5MP front) - Port from downstream
- S5K3T1 (secondary) - Port from downstream

**Success rate estimate:**
- OV02A10: 95% (just needs correct GPIOs/regulators)
- S5K2L7: 60% (depends on register compatibility with s5k2x7sp)
- S5K5E8/S5K3T1: Requires development effort (weeks-months)
