from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class Navigation_test(AbstractBehaviour):
    
    def init(self):
        self.navigation_sub = self.get_behaviour('NavigationSub')
        self.current_goal = 0
        self.poses = [
            {
                'pos': (-2.2, 0.04, 0),
                'orientation': (0, 0, 0.7, 0.69)
            },
            {
                'pos': (-2.35, -0.91, 0),
                'orientation': (0, 0, -0.7, 0.7)
            }
        ]

    def update(self):
        if self.state == State.start:
            self.navigation_sub.start_nagivate_to(**self.poses[self.current_goal])
            self.set_state(State.navigating)
        elif self.state == State.navigating:
            if self.navigation_sub.finished():
                self.current_goal = 1 - self.current_goal
                self.start()
            elif self.navigation_sub.failed():
                self.start()
