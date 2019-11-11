from behaviours.state import State


class BaseBehaviour(object):

    def __init__(self):

        self.state = State.stop

    def get_state(self):
        return self.state

    def set_state(self, state):

        if state not in list(State):
            print 'Error: state is not defined, exiting...'
            exit(0)

        self.state = state


