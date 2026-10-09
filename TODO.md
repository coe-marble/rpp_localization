# RPP localization TODO

## Supported EKF/UKF path

- [x] Keep the 15-state EKF and UKF numerical implementations in a ROS-free
      core library.
- [x] Define `NavModel15` for interchangeable 15-state process-model plugins.
- [x] Define `LocalizationFilter15` for interchangeable EKF/UKF filter plugins.
- [x] Provide `ConstantAccelerationModel15`, `Ekf15`, and `Ukf15` RPP entry
      points.
- [x] Provide `VehicleModel3DNavModel15` as an explicit VehicleModel3D bridge.
- [x] Keep model, adapter, and filter ownership at the RPP plugin composition
      root; generic filters do not own models.
- [x] Configure constant-acceleration and UKF process covariance through RPP
      parameters.
- [x] Preserve control timeout/configuration and measurement-covariance
      normalization at the RPP boundary.
- [x] Replay stale measurements from the retained initialization state through
      recorded prediction/correction history.

## ROS/RPP merge

- [x] Make `RosFilterBase` the direct owner of the RPP component context and
      `LocalizationFilter15`; remove the phantom `NavFilter<T>` wrapper.
- [x] Make `RosFilterBase` and `RosFilter` concrete, non-template classes;
      remove `RosFilterTypes` and type-tag aliases.
- [x] Preserve labust queue ordering, TF transforms, two-D handling, controls,
      services, timer prediction, diagnostics, and lagged-data reversion.
- [x] Make the RPP script the ROS composition root: its `filter` slot
      selects `Ekf15` or `Ukf15`, and each filter owns its `NavModel15` model.
- [x] Keep `ekf_node` and `ukf_node` as configuration-default compatibility
      aliases around the generic `localization_node`.
- [x] Keep ROS topic/frame and adapter settings in YAML; RPP compositions
      supply model-specific process and filter tuning.
- [ ] Restore/adapt ROS coverage after the structural merge; do not change
      InEKF in this work.

## InEKF

- [x] Design a dedicated RPP process and measurement contract for the
      Lie-group InEKF state, tangent covariance, and IMU input
      (`InertialModel`, `InvariantFilter`).
- [x] Port the InEKF behind that contract as a right-invariant filter with
      IMU biases, without forcing it into `NavModel15` or the 15-state
      EKF/UKF lifecycle (`ImuInertialModel`, `Inekf`, `inekf_node`).
- [ ] Left-invariant error convention; the contract carries the flag but
      only the right-invariant form is implemented.
- [ ] Measurement history and replay of late measurements.
- [ ] Sensor frames rotated against the base link.

## Verification

- [ ] Build and run core parity plus ROS adapter tests when authorized.
- [ ] Build and run the RPP boundary tests when the generated RPP interfaces
      and Capn Proto runtime use a matching ABI.
- [ ] Enable the VehicleModel3D bridge for a concrete model only after its
      numeric `step()` behavior is verified.
