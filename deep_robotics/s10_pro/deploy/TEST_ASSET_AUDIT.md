# S10 test asset removal — 2026-10-30

At the maintainer's explicit request, this update removes all S10-specific test
modules, standalone smoke tools and the captured `s10_pro_state.json` fixture:
51 files and 12,103 physical lines, including comments and blank lines. The
selected-test manifest and pytest development dependency are also removed.
Some removed modules were active regression tests; this is a deliberate removal
of the S10 test suite, not a claim that every deleted file was unused.

Other robots' tests, shared test infrastructure, manufacturer dependencies,
Driver runtime sources, site state and recorded task history are retained.
Existing Git commits retain the deleted sources. The release helper no longer
invokes missing tests and only validates release structure and configuration.

Before removal, the final full S10 run had 1,341 passes, 35 failures and one
skip. The failures were existing catalogue/manual-preparation expectations;
none was newly introduced by the preceding fixture cleanup. Its selected
release subset had 717 passes and one skip. These historical results do not
constitute a passing full-suite result or verification of the test-free tree.

Current validation uses the catalogue/schema/Docker-input checker, service and
container-contract checks, Python syntax validation and Git whitespace checks.
No regression suite, image build or physical motion acceptance is claimed for
this cleanup. No robot connection or deployment is performed by these checks.
