#!/usr/bin/env python3
"""MG-01 Terminal UI entrypoint.

キーバインド:
  s  START waypoint navigation (3s countdown)
  x  STOP waypoint navigation
  p  PAUSE
  r  RESUME
  m  Save map (SLAM)
  q  Quit
"""
import threading

import rclpy

from mg_tui import AppState, RosBackend, SystemClient, MgTuiApp


def main(args=None):
    rclpy.init(args=args)
    state = AppState()

    app = MgTuiApp.__new__(MgTuiApp)

    backend = RosBackend(state, lambda: app.call_from_thread(app._update_ui))
    sys_client = SystemClient(
        state, lambda: app.call_from_thread(app._update_ui))

    MgTuiApp.__init__(app, backend, sys_client, state)

    threading.Thread(
        target=lambda: rclpy.spin(backend),
        daemon=True,
    ).start()

    sys_client.start_polling()

    try:
        app.run()
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
