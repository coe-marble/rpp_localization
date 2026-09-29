# RPP localization migration TODO

## Current baseline

This package is a renamed copy of labust_localization. Its existing behavior is
the reference for every migration step.

Already present:

- EKF, UKF, and InEKF implementations;
- the generic NavModelBase abstraction, ConstantAccelerationModel, and InEKF
  InertialProcess;
- InEKF pose, twist, and attitude measurement models;
- NavFilter, RosFilter, RosBagFilter, ROS nodes, services, launch files,
  parameter files, and rosbag fixtures;
- unit, interface, launch, and bag-replay tests;
- package, generated-service, source, test, launch, and public-header
  namespaces renamed from robot_localization to rpp_localization.

Not yet validated after the package/namespace rename:

- configuration and compilation;
- the existing test suite;
- launch and bag-replay behavior.

No build or test has been run for this migration.

## Dependency-extraction status

The direct source and build dependency on labust_common is removed. Its
behavior-critical types are now owned by rpp_localization:

- filters derive from rpp_localization::FilterBase;
- models derive from rpp_localization::ModelBase;
- filters and ROS wrappers use rpp_localization::Measurement and
  ControlCommand.

These compatibility types intentionally retain rclcpp time and lifecycle
interfaces for now. Keep the legacy source as the behavior reference until
the renamed baseline and focused parity tests have run.

## Target RPP composition

Keep the legacy ownership boundary:

- a filter owns state, timestamps, prediction scheduling, and correction;
- a model owns its legacy state/covariance prediction behavior;
- an RPP filter plugin receives an RPP model plugin through
  RPP_COMPONENTS and ComponentContext.

The first RPP pair is:

- LocalizationNavModel15: the 15-state EKF/UKF model contract;
- LocalizationFilter15: the EKF/UKF filter contract.

The model step must carry the complete legacy input/output needed to preserve
state and covariance propagation, process noise, control use, time validation,
and status. Finalize its Cap'n Proto schema before implementing wrappers.

InEKF is not assumed to fit LocalizationNavModel15. Its existing
InertialProcess and Lie-group measurement models require a separate contract
decision after the EKF/UKF model path is proven.

VehicleModel3D is not a substitute for LocalizationNavModel15 without an
explicit 15-state mapping plus covariance and uncertainty-propagation
semantics.

## Work items

### 0. Lock down the renamed baseline

- [x] Copy the full legacy package.
- [x] Rename the package, public include path, code namespace, generated
      service namespace, launch/test package references, and install-visible
      library target to rpp_localization.
- [ ] Record the source labust_localization commit used for the copy.
- [ ] Configure and build the renamed package when authorization is given.
- [ ] Run the existing tests unchanged when authorization is given.
- [ ] Separate pre-existing legacy failures from rename regressions.

### 1. Extract the required legacy core types

- [x] Inventory every direct use of FilterBase, ModelBase, Measurement,
      ControlCommand, and their helper types.
- [x] Localize behavior-compatible Measurement, ControlCommand, ModelBase,
      and FilterBase under include/rpp_localization/ and remove the direct
      source, CMake, and package dependency.
- [x] Keep the legacy definitions as comparison references until parity tests
      pass.
- [x] Move the localized compatibility surface into
      include/rpp_localization/core/; keep its shared implementation utility
      in src/core/.
- [ ] Move common state, covariance, measurement, control, time, and status
      validation into the new core without changing defaults.
- [ ] Remove rclcpp from the new core API; keep ROS conversions in adapters.
- [ ] Add and run focused parity tests against the legacy reference when test
      execution is authorized.

### 1a. Establish package boundaries

- [x] Group implementation-independent compatibility types in core/.
- [x] Group generic filters, models, InEKF-specific code, and ROS adapters
      under filters/, models/, inekf/, and ros/.
- [x] Move ROS node entry points under src/ros/nodes/ and update CMake and
      internal include paths.

### 2. Preserve the existing generic filter/model boundary

- [ ] Adapt NavModelBase into a runtime rpp_localization core model interface;
      do not rewrite the EKF or UKF equations.
- [ ] Adapt Ekf and Ukf to consume that runtime interface rather than their
      current template-owned model.
- [ ] Port ConstantAccelerationModel unchanged behind the new interface.
- [ ] Verify state prediction, covariance prediction, controls, stale timing,
      near-zero/negative covariance handling, and pitch-singularity behavior
      against the baseline.
- [ ] Decide the separate InEKF model interface from existing
      InertialProcess behavior; do not force it into the 15-state contract.

### 3. Define RPP plugin contracts

- [ ] Add plugin_types/localization.capnp and plugins.json.
- [ ] Define canonical state-15, covariance-15x15 row-major, control-6,
      measurement, estimate, and status payloads.
- [ ] Define LocalizationNavModel15 step input/output and
      LocalizationFilter15 lifecycle operations.
- [ ] Document units, clock domain, optional-control encoding, covariance
      layout, validation, error status, and Cap'n Proto ordinal rules.
- [ ] Generate the C++ interfaces and review the generated method signatures.

### 4. Add RPP model and filter wrappers

- [ ] Implement plugins/models/constant_acceleration_model.hpp as
      LocalizationNavModel15 with RPP_PARAMETERS.
- [ ] Implement plugins/filters/ekf.hpp as LocalizationFilter15.
- [ ] Inject the model with:

      RPP_COMPONENTS(
        {"model", "rpp_localization::LocalizationNavModel15"}
      )

- [ ] Retrieve that component during initialization and bind it to the generic
      core EKF.
- [ ] Register both headers in plugins.json.
- [ ] Add plugin-boundary validation and parity tests.
- [ ] Add a UKF filter wrapper only after the shared model contract is proven.

### 5. Refactor ROS only after core/plugin parity

- [ ] Split ROS-specific code into an adapter target/package.
- [ ] Preserve existing nodes, topics, services, TF, parameters, diagnostics,
      launch files, queues, and replay behavior.
- [ ] Have the adapter construct the selected LocalizationFilter15 plugin.
- [ ] Migrate InEKF only after its process/measurement plugin contract is
      explicitly designed and tested.

## Working rule

Complete one unchecked item at a time. Do not start a later item, change
legacy behavior, or run builds/tests without explicit authorization.
