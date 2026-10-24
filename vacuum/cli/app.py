"""Command-line front end: drive the robot to named scene goals, without a GUI.

    python3 -m vacuum.cli.app --scene 01 --seed 42 kitchen sofa
    python3 -m vacuum.cli.app --scene 01 --all
    python3 -m vacuum.cli.app --list
    python3 -m vacuum.cli.app              # pick goals interactively
"""
from __future__ import annotations

import argparse
import logging
import sys
from typing import Optional

from vacuum.mission_behaviors.navigate_to_goal import GoalResult
from vacuum.mission_executor.mission_executor import ExecutorConfig, Goal, MissionExecutor

log = logging.getLogger("cli")


def pick_goals(goals: list[Goal]) -> list[str]:
    """Ask the user which goals to visit, by name or number, in order."""
    names = [g.name for g in goals]
    for i, g in enumerate(goals, 1):
        print(f"  {i}. {g.name}  ({g.x:.2f}, {g.y:.2f}{'' if g.yaw is None else f', {g.yaw:.2f} rad'})")
    while True:
        answer = input("Goals to visit (names or numbers, space separated; 'all'): ").split()
        if answer == ["all"]:
            return names
        picked = []
        for a in answer:
            if a.isdigit() and 1 <= int(a) <= len(names):
                picked.append(names[int(a) - 1])
            elif a in names:
                picked.append(a)
            else:
                print(f"unknown goal {a!r}")
                break
        else:
            if picked:
                return picked


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Drive the simulated robot to named scene goals.")
    p.add_argument("goals", nargs="*", help="goal names, visited in order (default: ask)")
    p.add_argument("--all", action="store_true", help="visit every goal of the scene, in scene order")
    p.add_argument("--list", action="store_true", help="list the scene's goals and exit")
    ExecutorConfig.add_arguments(p)
    p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s", datefmt="%H:%M:%S")
    try:
        executor = MissionExecutor.connect(ExecutorConfig.from_args(args))
    except ConnectionError as e:
        log.error("%s", e)
        return 2

    with executor:  # on exit: stops the mission and its recording, closes the drivers
        if args.list:
            for g in executor.goals:
                print(f"{g.name}  ({g.x:.2f}, {g.y:.2f}{'' if g.yaw is None else f', {g.yaw:.2f} rad'})")
            return 0
        if args.all:
            goal_names = [g.name for g in executor.goals]
        elif args.goals:
            goal_names = args.goals
        elif sys.stdin.isatty():
            goal_names = pick_goals(executor.goals)
        else:
            log.error("no goals given; pass goal names or --all")
            return 2

        executor.start(goal_names)
        try:
            executor.join()
        except KeyboardInterrupt:
            log.info("interrupted, stopping robot")
            return 130  # leaving the with block cancels the mission and finalises the recording
        if executor.error:
            log.error("mission aborted: %s", executor.error)
            return 1
        results = executor.results

    for name, result in results.items():
        print(f"{name:20s} {result.value}")
    rec = executor.recording_status()
    if rec is not None:
        print(f"recording: {rec.path} ({rec.frames} frames, {rec.lost} lost)")
    return 0 if all(r is GoalResult.SUCCEEDED for r in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
