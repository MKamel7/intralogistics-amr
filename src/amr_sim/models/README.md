# Imported warehouse scenery

Everything in this directory whose name starts with `wf_` is **generated**, not
hand-authored. Do not edit it: change `tools/import_aws_warehouse.py` and re-run

```bash
python3 tools/import_aws_warehouse.py \
  --src ../aws-robomaker-small-warehouse-world --dst .
```

which clears and rewrites the `wf_*` model set and regenerates
`worlds/warehouse.sdf`. Hand-authored models (conveyor stations, chargers, KLT
load carriers, pedestrians) live alongside without the prefix and are never
touched by the importer.

## Provenance

Source: the AWS RoboMaker small warehouse world
(<https://github.com/aws-robotics/aws-robomaker-small-warehouse-world>),
Copyright Amazon.com, Inc. or its affiliates, licensed **MIT-0**. Meshes and
textures are byte-for-byte the AWS originals. The full upstream licence is kept
in the source checkout referenced above.

## What the importer changes, and why

1. **Mesh URIs.** The AWS models use `file://models/<model>/meshes/<mesh>.DAE`,
   a path relative to the Gazebo Classic model-database root. Gazebo Harmonic
   resolves `model://<model>/...` against `GZ_SIM_RESOURCE_PATH`, so every URI
   is rewritten. The COLLADA materials themselves need no conversion, which is
   the reason this import is cheap rather than a rebuild.

2. **Model names.** Each model is re-emitted under a `wf_` prefix. This is not
   cosmetic: with the original names, an AWS checkout still on disk shadows the
   converted copy, and the parser then reports errors from a `model.sdf` that is
   not the one being edited. The prefix makes shadowing impossible.

3. **Inertia.** Two models ship tensors that are not physically realisable:
   `Ground_B` has `ixx + izz < iyy`, and `Roof_B` has `ixx + iyy < izz` by about
   0.7 percent. Classic never validated this; Harmonic correctly rejects the
   whole world with `A link named link has invalid inertia`. Both models are
   static, so their inertia is never integrated and the repair changes no
   observable behaviour. The importer sets the offending diagonal entries to the
   largest of the three and prints which models it repaired. A **non-static**
   model with a bad tensor is a real modelling error, so the importer raises
   instead of patching it.

Nothing else about the geometry is altered.

## Measured

World alone, 25 model instances, headless, no robots and no sensors, on an
i5-1235U: **20.0 s of simulated time in 2.44 s wall, real-time factor 8.2, at
38 percent CPU and 216 MB RSS**. The scenery is effectively free; the real-time
budget belongs to the robots and their sensors.
