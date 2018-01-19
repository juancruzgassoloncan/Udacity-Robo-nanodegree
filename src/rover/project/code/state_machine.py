# -*- coding: utf-8 -*-
from time import sleep


class state_machine(object):
    def __init__(self, machine, *args):
        self.machine = machine
        self.state_stack = list(args)
        self.current_state = self.state_stack[0]
        self.current_state.run()
        # self.next_state = ''


    def run(self):
        if len(self.state_stack) == 0:
            print('end')
            raw_input()
            return

        print('current:')
        print(self.current_state)
        self.next_state = self.current_state.next()
        # sleep(5)
        # raw_input()
        print('Next:')
        print(self.next_state)

        if str(self.next_state) == str(self.current_state):
            # print('same state')
            # sleep(5)
            pass
        elif self.next_state is None:
            # print('Non state')
            # raw_input()
            self.state_stack.pop(0)
        else:
            # print('new state')
            self.state_stack.insert(0, self.next_state)
            self.current_state = self.state_stack[0]
            self.current_state.run()
