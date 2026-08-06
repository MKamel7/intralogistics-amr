# Data sheets

Every physical constant in this repository traces to a line in one of these documents. If a value
is not in here, it is listed in `docs/platform_spec.md` as derived, estimated or tuned, with the
reasoning. A reviewer should never have to guess which numbers are real.

| File | Covers | Source |
|---|---|---|
| `MiR250_Specifications_v2.pdf` | Reference platform envelope, masses, wheels, speed and acceleration limits, corridor widths, minimum detectable object, battery and runtime, sensor set, operating envelope | Mobile Industrial Robots A/S, copyright 2021. Retrieved 2026-08-06. |
| `SICK_nanoScan3_NANS3-CAAZ30ZA1_dataSheet.pdf` | Safety laser scanner: 275 deg aperture, 0.17 deg angular resolution, 3 m protective field, 10 m warning field, 40 m measuring range, 70 ms response time, 65 mm protective field supplement, 20 mm detection resolution, PL d / Category 3 / SIL 2 / Type 3 | SICK AG, part no. 1126793. Retrieved 2026-08-06. |

Each PDF is kept with a `pdftotext -layout` extraction beside it so values can be grepped and cited
in code comments without opening a reader.

Two independent sheets agree on one number, which is a useful cross-check: MiR claims a 20 mm object
detectable at 1000 mm, and the nanoScan3 offers a configurable 20 mm resolution. The scanner model
uses the 20 mm setting on that basis rather than picking a resolution to suit the result.

## Outstanding

- **VDA 4500 KLT.** Small load carrier dimension table, needed for the payload model.

## A note on what these documents are used for

The robot built here is a *class* of machine derived from a published specification. It is not a
model of any vendor's product, carries no vendor branding, and is not presented as equivalent to
one. Where our model deviates from the reference, `docs/platform_spec.md` says so.
