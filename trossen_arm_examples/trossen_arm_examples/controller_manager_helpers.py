from collections.abc import Iterable

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.duration import Duration
from rclpy.node import Node


class ControllerManagerClient:
    def __init__(self, node: Node, controller_manager_name: str) -> None:
        self._node = node
        self._controller_manager_name = controller_manager_name.rstrip('/')
        self._list_client = node.create_client(
            ListControllers, f'{self._controller_manager_name}/list_controllers'
        )
        self._load_client = node.create_client(
            LoadController, f'{self._controller_manager_name}/load_controller'
        )
        self._configure_client = node.create_client(
            ConfigureController, f'{self._controller_manager_name}/configure_controller'
        )
        self._switch_client = node.create_client(
            SwitchController, f'{self._controller_manager_name}/switch_controller'
        )

    def ensure_active(
        self,
        controller_names: Iterable[str],
        timeout_sec: float = 5.0,
        deactivate_controller_names: Iterable[str] | None = None,
    ) -> None:
        controller_names = list(controller_names)
        deactivate_controller_names = list(deactivate_controller_names or [])
        self._wait_for_services(timeout_sec)

        self.ensure_loaded_configured(controller_names, timeout_sec)

        states = self._controller_states()
        active_to_deactivate = [
            name for name in deactivate_controller_names if states.get(name) == 'active'
        ]
        inactive_to_activate = [
            name for name in controller_names if states.get(name) != 'active'
        ]

        if inactive_to_activate or active_to_deactivate:
            self._switch_controllers(
                activate_names=inactive_to_activate,
                deactivate_names=active_to_deactivate,
                timeout_sec=timeout_sec,
            )

    def ensure_loaded_configured(
        self, controller_names: Iterable[str], timeout_sec: float = 5.0
    ) -> None:
        controller_names = list(controller_names)
        self._wait_for_services(timeout_sec)

        states = self._controller_states()
        for name in controller_names:
            if name not in states:
                self._load_controller(name, timeout_sec)

        states = self._controller_states()
        for name in controller_names:
            if states.get(name) == 'unconfigured':
                self._configure_controller(name, timeout_sec)

    def _wait_for_services(self, timeout_sec: float) -> None:
        clients = [
            self._list_client,
            self._load_client,
            self._configure_client,
            self._switch_client,
        ]
        for client in clients:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f'Service `{client.srv_name}` is not available.')

    def _controller_states(self) -> dict[str, str]:
        request = ListControllers.Request()
        response = self._call(self._list_client, request)
        return {controller.name: controller.state for controller in response.controller}

    def _load_controller(self, name: str, timeout_sec: float) -> None:
        request = LoadController.Request()
        request.name = name
        response = self._call(self._load_client, request, timeout_sec)
        if not response.ok:
            raise RuntimeError(f'Failed to load controller `{name}`.')

    def _configure_controller(self, name: str, timeout_sec: float) -> None:
        request = ConfigureController.Request()
        request.name = name
        response = self._call(self._configure_client, request, timeout_sec)
        if not response.ok:
            raise RuntimeError(f'Failed to configure controller `{name}`.')

    def _switch_controllers(
        self,
        activate_names: list[str],
        deactivate_names: list[str],
        timeout_sec: float,
    ) -> None:
        request = SwitchController.Request()
        request.activate_controllers = activate_names
        request.deactivate_controllers = deactivate_names
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(seconds=timeout_sec).to_msg()
        response = self._call(self._switch_client, request, timeout_sec)
        if not response.ok:
            activate_joined = ', '.join(activate_names) or '<none>'
            deactivate_joined = ', '.join(deactivate_names) or '<none>'
            raise RuntimeError(
                'Failed to switch controller(s): '
                f'activate [{activate_joined}], deactivate [{deactivate_joined}].'
            )

    def _call(self, client, request, timeout_sec: float = 5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            raise RuntimeError(f'Service call `{client.srv_name}` timed out or failed.')
        return future.result()
