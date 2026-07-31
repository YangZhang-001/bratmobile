# Stereo Camera Mount

This directory contains the 3D model and print-ready files for the stereo camera mount used in the ROCK 5 robot vision system.

## Design

Following the supervisor's guidance, one side of the mount was redesigned with an inward angle to improve the overlap between the two camera views.

Several angle variants were printed and tested. The final design was selected based on camera alignment and stereo image performance.

The mount is printed as a single component to maintain a fixed relative position between the two cameras. This improves durability and reduces the risk of camera movement or alignment changes during operation.

With the current mount installed, the two camera views overlap at a point approximately **750 mm** with no observable alignment error. 

This convergence distance provides a repeatable reference for reproducing the camera geometry used in subsequent stereo matching and target-coordinate calculation tests.

## Specifications

- Dimensions: **108 × 28 × 21 mm**
- Printing method: **single-piece 3D printing**
- Recommended layer height: **0.2 mm**
- Camera alignment reference: **750 mm**
- Required fasteners:
  - **2 × M2 bolts**
  - **2 × M3 bolts**

## Files

This directory includes:

- The final 3D model of the camera mount
- A print-ready file prepared for the **Ultimaker S3**

Printing settings should be checked before using a different printer, material, nozzle size, or layer height.

## Installation Notes

After printing and installation:

1. Confirm that the mount has no visible deformation.
2. Secure the two camera modules using the specified M2 and M3 bolts.
3. Check that both cameras remain firmly fixed.
4. Verify image overlap at the **750 mm** calibration position.
5. Confirm the stereo image alignment before performing coordinate-calculation tests.

Any change to the printed geometry, camera position, fastening method, or mounting angle may alter the stereo camera relationship and should be validated again.
