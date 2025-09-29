import sys
import os

# Ensure the package parent directory is on sys.path so tests can import
# top-level package `horiokart_slam` (e.g., `from horiokart_slam.tools ...`).
THIS_DIR = os.path.abspath(os.path.dirname(__file__))
# parent of tests -> horiokart_slam, parent of that -> horiokart (package root)
PARENT_DIR = os.path.abspath(os.path.join(THIS_DIR, '..', '..'))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)
# Also add the package directory itself so tests that `import tools` work
PKG_DIR = os.path.abspath(os.path.join(THIS_DIR, '..'))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)
# Also add the repository root (/app) so that 'horiokart_slam' package can be imported
REPO_ROOT = os.path.abspath(os.path.join(PARENT_DIR, '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
# Also add /app (workspace root) to ensure package resolution works when /root/ros2_ws/src/horiokart is a symlink to /app
APP_PATH = '/app'
if os.path.isdir(APP_PATH) and APP_PATH not in sys.path:
    sys.path.insert(0, APP_PATH)
