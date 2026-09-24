# lynsense_utils compatibility package

This directory installs the ROS package name `lynsense_utils` as a
**simulation-only reconstruction** of `NavToPose` and `MoveDistance` from the
fields observed on the caller side. It is not evidence that the company's
original interface package or its exact semantics have been obtained or
reviewed.

Before using a company-provided `lynsense_utils`, diff the field names, units,
result semantics, feedback semantics, and action names against these
definitions. The original package **must be replaced or reconciled** with this
reconstruction after that review.

The wheel `maxTorque 50 N*m` value in the Webots smoke model is a smoke-model
parameter because the supplied URDF does not provide a reviewed wheel torque limit.
It is not a validated CR100 actuator rating.
