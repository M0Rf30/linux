# Camera Power Domain Fix for SDM660

## Problem Summary

The OV02A10 camera sensor integration was failing with clock enable errors:
- `camss_top_ahb_clk status stuck at 'off'`
- `camss_vfe0_clk status stuck at 'off'`
- Error: `clock enable failed: -16 (EBUSY)`
- Error: `Failed to power up pipeline: -16`

Root cause: Missing power domain configuration in device tree and driver.

## Solution

### 1. Device Tree Changes (`arch/arm64/boot/dts/qcom/sdm636-xiaomi-tulip.dts`)

Added power domain configuration to CAMSS node:

```dts
&camss {
	status = "okay";
	
	power-domains = <&mmcc CAMSS_VFE0_GDSC>,
			<&mmcc CAMSS_VFE1_GDSC>,
			<&mmcc CAMSS_TOP_GDSC>;
	power-domain-names = "vfe0", "vfe1", "top";
	
	ports {
		/* ... existing port configuration ... */
	};
};
```

**Order is critical**: VFE0, VFE1, TOP (matches legacy driver expectations)

### 2. Driver Changes (`drivers/media/platform/qcom/camss/camss.c`)

#### Change 1: Add pd_name to VFE0 resource (line ~706)
```c
.vfe = { 
	.line_num = 3,
	.has_pd = true,
	.pd_name = "vfe0",  // ADDED
	.hw_ops = &vfe_ops_4_8,
	.formats_rdi = &vfe_formats_rdi_8x96,
	.formats_pix = &vfe_formats_pix_8x96 
}
```

#### Change 2: Add pd_name to VFE1 resource (line ~729)
```c
.vfe = { 
	.line_num = 3,
	.has_pd = true,
	.pd_name = "vfe1",  // ADDED
	.hw_ops = &vfe_ops_4_8,
	.formats_rdi = &vfe_formats_rdi_8x96,
	.formats_pix = &vfe_formats_pix_8x96 
}
```

#### Change 3: Add pd_name to sdm660_resources (line ~3880)
```c
static const struct camss_resources sdm660_resources = {
	.version = CAMSS_660,
	.pd_name = "top",  // ADDED
	.csiphy_res = csiphy_res_660,
	.csid_res = csid_res_660,
	.ispif_res = &ispif_res_660,
	.vfe_res = vfe_res_660,
	.csiphy_num = ARRAY_SIZE(csiphy_res_660),
	.csid_num = ARRAY_SIZE(csid_res_660),
	.vfe_num = ARRAY_SIZE(vfe_res_660),
};
```

## How It Works

### Power Domain Hierarchy
```
camss_top_gdsc (parent)
├── camss_vfe0_gdsc
├── camss_vfe1_gdsc  
└── camss_cpp_gdsc
```

### Before Fix
- No power domains specified in device tree
- Driver tried to enable clocks without powering on GDSCs
- Hardware status bits never transitioned from 'off' to 'on'
- Clock framework returned -EBUSY (error -16)

### After Fix
- Device tree specifies all three power domains with names
- VFE0 attaches to "vfe0" power domain by name
- VFE1 attaches to "vfe1" power domain by name
- CAMSS top level attaches to "top" power domain by name
- Power domains are enabled before clocks
- Hardware transitions properly: off → on

## Testing

After deploying the fix, use the test script:

```bash
# On device:
/path/to/test_camera_capture.sh
```

Expected result:
- Power domains show as attached in `pm_genpd_summary`
- No clock enable errors in dmesg
- Frame capture succeeds
- Output file `/tmp/capture.raw` is 3,840,000 bytes (1600x1200x2)

## Next Steps

1. Deploy updated kernel to device
2. Run test script
3. Verify capture works
4. Analyze captured raw Bayer frame
5. Consider upstreaming these fixes to mainline

## Reference

- Oracle consultation session: ses_422ac1a4effe8W4KTMn7Fe30zJ
- Similar implementation: MSM8953 (vfe_res_8x53) already has pd_name fields
- Newer platforms (SC7280+) all use pd_name for power domain attachment
