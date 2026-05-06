import threading
import time

import requests

from .state import AppState

_BASE_URL = 'http://localhost:8001'


class SystemClient:
    def __init__(self, state: AppState, refresh_cb):
        self._state = state
        self._refresh = refresh_cb

    def start_polling(self) -> None:
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def _poll_loop(self) -> None:
        while True:
            try:
                r = requests.get(f'{_BASE_URL}/status', timeout=2)
                self._state.containers = r.json()
                self._refresh()
            except Exception:
                pass
            time.sleep(2.0)

    def save_map(self) -> None:
        def _do() -> None:
            try:
                r = requests.post(f'{_BASE_URL}/map/save', timeout=35)
                data = r.json()
                self._state.last_service_msg = (
                    f'save_map: {"OK" if data["success"] else "FAIL"} {data["message"]}'
                )
            except Exception as e:
                self._state.last_service_msg = f'save_map: error {e}'
            self._refresh()

        threading.Thread(target=_do, daemon=True).start()

    def start_service(self, name: str) -> None:
        def _do() -> None:
            try:
                r = requests.post(f'{_BASE_URL}/{name}/start', timeout=10)
                data = r.json()
                self._state.last_service_msg = (
                    f'{name}/start: {"OK" if data["success"] else "FAIL"} {data.get("message", "")}'
                )
            except Exception as e:
                self._state.last_service_msg = f'{name}/start: error {e}'
            self._refresh()

        threading.Thread(target=_do, daemon=True).start()

    def stop_service(self, name: str) -> None:
        def _do() -> None:
            try:
                r = requests.post(f'{_BASE_URL}/{name}/stop', timeout=10)
                data = r.json()
                self._state.last_service_msg = (
                    f'{name}/stop: {"OK" if data["success"] else "FAIL"} {data.get("message", "")}'
                )
            except Exception as e:
                self._state.last_service_msg = f'{name}/stop: error {e}'
            self._refresh()

        threading.Thread(target=_do, daemon=True).start()

    def reset_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        def _do() -> None:
            try:
                r = requests.post(
                    f'{_BASE_URL}/simulation/reset-pose',
                    json={'x': x, 'y': y, 'z': z, 'yaw': yaw},
                    timeout=10,
                )
                data = r.json()
                self._state.last_service_msg = (
                    f'reset_pose: {"OK" if data["success"] else "FAIL"} {data.get("message", "")}'
                )
            except Exception as e:
                self._state.last_service_msg = f'reset_pose: error {e}'
            self._refresh()

        threading.Thread(target=_do, daemon=True).start()
