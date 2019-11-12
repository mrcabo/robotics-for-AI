from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class Main(AbstractBehaviour):
    
    def init(self):
        self.sub1_name = self.get_behaviour('Sub1Name')
        self.sub2_wait = self.get_behaviour('Sub2Wait')
        self.sub3_print = self.get_behaviour('Sub3Print')
        self.alice_counter = 0

    def update(self):
        # In start state start the first sub behaviour
        if self.state == State.start:
            self.sub1_name.start()
            self.state = State.selecting_name
        # When sub1 finished pass the name to sub3 and start sub2
        elif self.state == State.selecting_name:
            if self.sub1_name.finished():
                self.sub3_print.set_name(self.sub1_name.name)
                self.sub2_wait.start()
                self.set_state(State.waiting)
        # When in state runningSub2Wait wait until sub2 is finished and then start sub3
        elif self.state == State.waiting:
            if self.sub2_wait.finished():
                self.sub3_print.start()
                self.set_state(State.printing)
        # When sub3 finished check the alice counter and restart or finish accordingly
        elif self.state == State.printing:
            if self.sub3_print.failed():
                print "Failed because of", self.sub3_print.failure_reason
                self.sub3_print.start()
            elif self.sub3_print.finished():
                self.alice_counter += self.sub3_print.name == "ALICE"
                if self.alice_counter >= 2:
                    self.finish()
                else:
                    # Restart the behaviour
                    self.sub1_name.reset()
                    self.sub2_wait.reset()
                    self.set_state(State.start)
