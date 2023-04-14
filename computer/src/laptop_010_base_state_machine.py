#!/usr/bin/env python

import smach

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['found', 'turn', 'keep_searching'],)

    def execute(self, ud):
        blocks_found = False
        at_wall = False

        if blocks_found:
            next_state = 'found'
        elif at_wall:
            next_state = 'turn'
        else:
            next_state = 'keep_searching'

        return next_state


class TurnAround(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['search', 'keep_turning'],)

    def execute(self, ud):
        at_wall = False

        if at_wall:
            next_state = 'keep_turning'
        else:
            next_state = 'search'

        return next_state


class ObjectFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['stage', 'found_continue'],)

    def execute(self, ud):
        block_in_catcher = False

        if block_in_catcher:
            next_state = 'found_continue'
        else:
            next_state = 'stage'

        return next_state


class StagingPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['arrange', 'to_last', 'keep_staging'],)

    def execute(self, ud):
        at_staging = False
        all_blocks_found = False

        if not at_staging:
            next_state = 'keep_staging'
        else:
            if all_blocks_found:
                next_state = 'arrange'
            else:
                next_state = 'to_last'
        
        return next_state


class ReturnToLast(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['find_next', 'returning'],)

    def execute(self, ud):
        returned_to_last = False

        if returned_to_last:
            next_state = 'returning'
        else:
            next_state = 'find_next'

        return next_state


class Arrange(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['keep_arranging', 'done'],)

    def execute(self, ud):
        arranged = False

        if arranged:
            next_state = 'done'
        else:
            next_state = 'keep_arranging'

        return next_state


def main():
    sm = smach.StateMachine(outcomes=['exit'])

    with sm:
        smach.StateMachine.add('SEARCH', Search(),
                                transitions={
                                    'found' : 'OBJECTFOUND',
                                    'turn'  : 'TURNAROUND',
                                    'keep_searching' : 'SEARCH'
                                })

        smach.StateMachine.add('TURNAROUND', TurnAround(), 
                                transitions={
                                    'search' : 'SEARCH',
                                    'keep_turning' : 'TURNAROUND'
                                })

        smach.StateMachine.add('OBJECTFOUND', ObjectFound(), 
                                transitions={
                                    'stage' : 'STAGINGPOINT',
                                    'found_continue' : 'OBJECTFOUND'
                                })

        smach.StateMachine.add('STAGINGPOINT', StagingPoint(),
                                transitions={
                                    'arrange' : 'ARRANGE',
                                    'to_last'  : 'RETURNTOLAST',
                                    'keep_staging' : 'STAGINGPOINT'
                                })

        smach.StateMachine.add('RETURNTOLAST', ReturnToLast(), 
                                transitions={
                                    'find_next' : 'SEARCH',
                                    'returning' : 'RETURNTOLAST'
                                })

        smach.StateMachine.add('ARRANGE', Arrange(), 
                                transitions={
                                    'keep_arranging' : 'ARRANGE',
                                    'done' : 'exit'
                                })
    sm.execute()


if __name__ == "__main__":
    main()