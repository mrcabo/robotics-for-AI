from state import State


class AbstractBehaviour(object):

    def __init__(self, all_behaviours):
        self.all_behaviours = all_behaviours
        self.state = State.idle
        self.failure_reason = ""
        
    def get_behaviour(self, name):
        return self.all_behaviours.get_behaviour(name)

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = state

    def stopped(self):
        return self.state == State.failed or self.state == State.finished
    
    def finished(self):
        return self.state == State.finished

    def failed(self):
        return self.state == State.failed
    
    def get_failure_reason(self):
        return self.failure_reason
    
    def finish(self):
        self.state = State.finished
        
    def fail(self, reason):
        self.state = State.failed
        self.failure_reason = reason

    def start(self):
        self.state = State.start

