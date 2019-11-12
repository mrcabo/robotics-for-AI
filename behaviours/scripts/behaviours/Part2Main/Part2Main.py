from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class Part2Main(AbstractBehaviour):
    
    def init(self):
        self.action_client_sub_behaviour = self.get_behaviour('InteractWithAddIntegersActionServer')

    def update(self):
        # When in starting state, start the behaviour that will talk to the ActionServer
        if self.state == State.start:
            self.action_client_sub_behaviour.start()
            self.state = State.waiting
        # When the state is waiting check the whether the sub behaviour finished or failed
        elif self.state == State.waiting:
            if self.action_client_sub_behaviour.finished():
                self.finish()
            elif self.action_client_sub_behaviour.failed():
                self.start()
