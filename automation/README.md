# Subaru Crosstrek synchronization

This automation lives on `master` so the installable `subaru-crosstrek` branch can initially
remain byte-for-byte identical to `jacobwaller/opendbc:jul-angle-based` at `8acbecc`.

When comma publishes an openpilot stable tag newer than the currently published `v0.11.1`, the workflow reads that
tag's exact `opendbc_repo` pin, retains Jacob Waller's original commits in ancestry, replays
the 14 source commits individually, verifies their attribution and patch IDs, runs the full
test suite, and publishes only if every check succeeds.

Conflicts, empty patches, changed source history, metadata differences, and test failures stop
the update without changing `subaru-crosstrek`.
