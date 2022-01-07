""" This module contains data structures for the Tactics level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp.action
import stp.rc
import stp.role
import stp.skill
import stp.utils.enum
import stp.utils.typed_key_dict

from rj_msgs.msg import RobotIntent


class ITactic(ABC):
    pass


RoleRequests = Any
RoleResults = Any
SkillEntry = Any


class Tactic(ABC):
    """Complex single-robot role, such as Goalie or Striker. Created and ticked by Plays. Uses Skills to achieve behavior."""

    def __init__(self, robot: stp.rc.Robot):
        """All Tactics should apply to one robot's behavior; thus, robot is defined as a formal argument here. Concrete Tactics should overwrite init with their own fields, but call super()'s init to use this shared code."""
        self.robot: rc.Robot = robot

    @abstractmethod
    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> RobotIntent:
        """Logic for role goes here. RobotIntents obtained via Skills. (e.g. Goalie can tick Capture to get ball or Intercept to block a shot.)

        :param world_state: Current world state.
        :return: A single RobotIntent.
        """
        ...

    @abstractmethod
    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        # TODO: add docstring here
        ...
