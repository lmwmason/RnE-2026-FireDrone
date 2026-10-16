UKF-only attitude selection experiment. Not linked into production firmware and not flashed.
No baseline attitude is passed to this selector: finite valid UKF is selected directly, without envelope/disagreement fallback or entry blend. Existing rate PID is conceptually retained; UKF is an estimator, not a torque controller.
Invalid/stale input produces an explicit INVALID result with no replacement output. This is not a complete flight fault-handling strategy. A flight integration must define and validate that behavior rather than silently reuse this host result as actuator authorization.
Flight22 provides no external attitude truth. Third-flight estimate disagreement reaches34.8deg in pitch, equivalent to174deg/s difference at the pre-filter ANGLE rate-command input with current levelGain5. This counterfactual is not a closed-loop trajectory prediction and cannot establish which estimate is correct. Therefore live UKF-only operation is not validated by these recordings.

User explicitly deferred upload until a later instruction. Do not upload or enable this experiment without that later instruction.
