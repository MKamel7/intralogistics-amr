# Data sheets

Every physical constant in this repository traces to a line in one of these documents. If a value
is not in here, it is listed in `docs/platform_spec.md` as derived, estimated or tuned, with the
reasoning. A reviewer should never have to guess which numbers are real.

| File | Covers | Source |
|---|---|---|
| `MiR250_Specifications_v2.pdf` | Reference platform envelope, masses, wheels, speed and acceleration limits, corridor widths, minimum detectable object, battery and runtime, sensor set, operating envelope | Mobile Industrial Robots A/S, copyright 2021. Retrieved 2026-08-06. |
| `Intel_RealSense_D435i_TechSpec.pdf` | RGB-D camera: 87 deg +/- 3 x 58 deg +/- 1 depth field of view, Min-Z 0.105 m, maximum range about 10 m, depth up to 1280 x 720 at up to 90 fps, RGB 1920 x 1080 at 30 fps, module 90 x 25 x 25 mm | Intel Corporation, D435i tech spec. Retrieved 2026-08-06. |
| `SICK_nanoScan3_NANS3-CAAZ30ZA1_dataSheet.pdf` | Safety laser scanner: 275 deg aperture, 0.17 deg angular resolution, 3 m protective field, 10 m warning field, 40 m measuring range, 70 ms response time, 65 mm protective field supplement, 20 mm detection resolution, PL d / Category 3 / SIL 2 / Type 3 | SICK AG, part no. 1126793. Retrieved 2026-08-06. |

Each PDF is kept with a `pdftotext -layout` extraction beside it so values can be grepped and cited
in code comments without opening a reader.

Two independent sheets agree on one number, which is a useful cross-check: MiR claims a 20 mm object
detectable at 1000 mm, and the nanoScan3 offers a configurable 20 mm resolution. The scanner model
uses the 20 mm setting on that basis rather than picking a resolution to suit the result.

A second cross-check, and this one resolved a real error. The platform sheet gives 114 degrees
horizontal for its two 3D cameras without saying whether that is per camera or combined. The Intel
sheet gives 87 degrees for one D435, which settles it: 114 is the pair. The model had been using
114 degrees per camera, overstating each sensor by 27 degrees. Two 87 degree cameras toed out by
13.5 degrees span 114 degrees exactly, so the two documents now agree and a test asserts it.

## Outstanding

- **VDA 4500 KLT.** Small load carrier dimension table, needed for the payload model.

## A note on what these documents are used for

The robot built here is a *class* of machine derived from a published specification. It is not a
model of any vendor's product, carries no vendor branding, and is not presented as equivalent to
one. Where our model deviates from the reference, `docs/platform_spec.md` says so.
