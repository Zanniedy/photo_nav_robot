import os
import pytest
import robot_config
from conftest import TMP


def test_default_when_missing(clean):
    d = robot_config.load()
    assert d["configured"] is False
    assert d["robot_id"].startswith("unset-")
    assert robot_config.public_info()["api_version"] == "1.0"


def test_set_identity_only_once(clean):
    info = robot_config.set_identity("max-001", "一号机")
    assert info == {"robot_id": "max-001", "name": "一号机", "configured": True,
                    "api_version": "1.0", "tokens": []}
    robot_config.set_identity("max-001")                 # 同一个 id 不算改
    robot_config.set_identity(name="大厅机")
    assert robot_config.load()["name"] == "大厅机"
    with pytest.raises(robot_config.ConfigError) as e:
        robot_config.set_identity("max-002")
    assert e.value.code == "ROBOT_NOT_CONFIGURED"


def test_name_only_keeps_unconfigured(clean):
    robot_config.set_identity(name="测试")
    d = robot_config.load()
    assert d["configured"] is False and d["name"] == "测试"


@pytest.mark.parametrize("bad", ["Max-01", "max_01", "", "a" * 33, "机器人"])
def test_bad_robot_id(clean, bad):
    with pytest.raises(robot_config.ConfigError) as e:
        robot_config.set_identity(bad)
    assert e.value.code == "BAD_REQUEST"


def test_tokens(clean):
    tok = robot_config.create_token("tablet")
    assert len(tok) >= 32
    assert robot_config.check_token(tok) == "tablet"
    assert robot_config.check_token("wrong") is None
    assert robot_config.public_info()["tokens"] == ["tablet"]
    assert oct(os.stat(os.path.join(TMP, "robot.yaml")).st_mode & 0o777) == "0o600"
    with pytest.raises(robot_config.ConfigError):
        robot_config.create_token("tablet")              # 重名
    assert robot_config.delete_token("tablet") is True
    assert robot_config.check_token(tok) is None
    assert robot_config.delete_token("tablet") is False


def test_corrupt_file_falls_back(clean):
    with open(os.path.join(TMP, "robot.yaml"), "w") as f:
        f.write("robot_id: [unclosed\n")
    assert robot_config.load()["configured"] is False
