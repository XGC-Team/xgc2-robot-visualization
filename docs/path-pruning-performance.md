# Path-history pruning without throttling

Related integration audit: XGC-Team/xgc2-harness#104.
Baseline: noetic@cd99d9517d85970605b4966c08437db4f51afde2.

`BoundedPathRuntime::append` and `expire` previously erased the first vector
entry repeatedly. Each erase moves the remaining poses. A batch of k expired
samples in a history of n samples can therefore cause O(k*n) move assignments.

The candidate scans the same prefix and performs one range erase. It preserves
all existing decisions: configured sampling rate, count and age limits, the
last-old-point continuity anchor in append, inclusive time-window boundary,
empty-expiry stamp, input pose/quaternion values, sample order, namespace,
frame and clock-rollback handling. No publishing rates, TF/URDF chain, message
schema, saved layout or history configuration is changed.

Three regression tests were added to the existing PathRuntime suite: bulk
append with continuity/quaternion preservation, bulk expiry including the
one-nanosecond boundary and every surviving pose, and count-only pruning
with unchanged sampling behavior. Existing tests remain unchanged.

## Validation at submission

A local isolated C++17 differential check compiled with `-O2 -Wall -Wextra
-Werror -fsanitize=undefined` and passed 20,000 append-pruning plus 20,000
expiry-pruning cases against the old loops. Candidate loops were extracted
from the edited source. All surviving timestamps and seven pose components
matched. This check uses minimal timestamp/point records, not ROS libraries;
it is not a substitute for compiling and running the actual ROS/gtest suite.

A deterministic 10,000-point stress case that expires 5,000 points changed
move-assignment count from 37,497,500 to 5,000 with identical survivors.
This measures vector movement, not FPS or ROS latency. It is deliberately
larger than the default six-second, 10 Hz product history and must not be
presented as a typical scene speedup.

The local environment has no ROS/gtest runtime. The actual PathRuntime tests,
package build/install checks and same-input product playback remain required.
Keep the integration issue open until the packaged candidate is evaluated
with 3D and camera/AR concurrently active, recording frame-interval tails
and verifying unchanged trajectory points, quaternion axes and TF alignment.
