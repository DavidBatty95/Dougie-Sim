import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from onboard_visual_session import should_restart


def test_early_fall_restarts_but_time_budget_and_user_goal_limit_end_session():
    assert should_restart(0, "fallen", 600., None)
    assert should_restart(0, "odom_stale", 600., None)
    assert not should_restart(0, "duration_elapsed", 600., None)
    assert not should_restart(0, "fallen", 4., None)
    assert not should_restart(0, "fallen", 600., 4)
    assert not should_restart(1, "process_failed", 600., None)


def test_session_restarts_a_clean_episode_with_remaining_budget(tmp_path, monkeypatch):
    import json
    import onboard_visual_session as session
    clock = [0.]
    commands = []
    monkeypatch.setattr(session, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(session, "preflight", lambda *args: None)
    monkeypatch.setattr(session.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(session.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    monkeypatch.setattr(session.signal, "signal", lambda *args: None)
    monkeypatch.setattr(sys, "argv", ["visual-session", "--duration", "60", "--seed", "42",
                                      "--result-root", str(tmp_path)])

    def episode(command, log, timeout):
        commands.append(command)
        result_dir = Path(command[command.index("--result-dir") + 1])
        result_dir.mkdir(parents=True, exist_ok=True)
        reason = "fallen" if len(commands) == 1 else "duration_elapsed"
        clock[0] += 10. if len(commands) == 1 else 48.
        (result_dir / "person_avoidance_test.json").write_text(json.dumps({
            "session_end_reason": reason, "collisions": 1, "goals_reached": 2}))
        return 0, False

    monkeypatch.setattr(session, "stream_process", episode)
    assert session.main() == 0
    assert len(commands) == 2
    assert [c[c.index("--seed") + 1] for c in commands] == ["42", "43"]
    assert float(commands[1][commands[1].index("--duration-seconds") + 1]) == 48.
    assert "--skip-build" not in commands[0] and "--skip-build" in commands[1]
    assert all(c[c.index("--people") + 1] == "8" for c in commands)
    result = json.loads(next(tmp_path.glob("visual_session_*/session.json")).read_text())
    assert result["status"] == "completed" and not result["scored"]
    assert [e["end_reason"] for e in result["episodes"]] == ["fallen", "duration_elapsed"]
    assert [e["collisions"] for e in result["episodes"]] == [1, 1]
