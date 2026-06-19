"""Unit tests for MoveGroup cancel helper."""

from unittest.mock import MagicMock, patch

from risk_engine.move_group_cancel import MoveGroupCancelClient


@patch('risk_engine.move_group_cancel.ActionClient')
def test_cancel_all_calls_action_client_when_server_ready(mock_action_client_cls):
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    mock_client = MagicMock()
    mock_client.server_is_ready.return_value = True
    mock_action_client_cls.return_value = mock_client

    client = MoveGroupCancelClient(node, action_name='/move_action')
    assert client.cancel_all() is True
    mock_client.cancel_all_goals_async.assert_called_once()


@patch('risk_engine.move_group_cancel.ActionClient')
def test_cancel_all_returns_false_when_server_unavailable(mock_action_client_cls):
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    mock_client = MagicMock()
    mock_client.server_is_ready.return_value = False
    mock_client.wait_for_server.return_value = False
    mock_action_client_cls.return_value = mock_client

    client = MoveGroupCancelClient(node)
    assert client.cancel_all() is False
    mock_client.cancel_all_goals_async.assert_not_called()
