"""Keep tests away from the real rig configuration in ~/.openhead-body."""
import os
import tempfile

# Read when openhead_body.joints is imported, so it is set before any test module loads.
os.environ["OPENHEAD_BODY_JOINTS"] = os.path.join(tempfile.mkdtemp(prefix="openhead-body-"), "joints.json")
