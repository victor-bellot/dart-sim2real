import fsm
import time
import dummyrob  # to be replaced by dartv2

# global variables
f = fsm.Fsm()  # finite state machine

time_start = time.time()
time_begin = 5.0  # wait 5 s before turning
turn_count = 0  # count half turns

# create a robot (to be replaced by dartv2)
myBot = dummyrob.MyDummyRob()


# functions (actions of the fsm)
def doWait():
    global time_start, time_begin
    print(">>>>>> action")
    myBot.doNothing(1)
    if time.time() - time_start > time_begin:  # auto start in 5 seconds
        event = "go"
    else:
        event = "stop"
    return event


def doTurn():
    global turn_count
    turn_count = turn_count + 1
    print(">>>>>> action")
    myBot.halfTurn(3)
    if turn_count == 2:  # full turn
        event = "stop"
    else:
        event = "go"
    return event


def doStop():
    print(">>>>>> action")
    event = None
    return event


if __name__ == "__main__":
    use_file = True
    if use_file:
        f.load_fsm_from_file("fsm_full_turn.txt")
    else:
        # define the states
        f.add_state("Idle")
        f.add_state("Turn")
        f.add_state("End")

        # defines the transition matrix
        # current state, next state, event, action in next state
        f.add_transition("Idle", "Idle", "stop", doWait)
        f.add_transition("Idle", "Turn", "go", doTurn)
        f.add_transition("Turn", "Turn", "go", doTurn)
        f.add_transition("Turn", "End", "stop", doStop)

        # defines the events
        f.add_event("go")
        f.add_event("stop")

        # initial state
        f.set_state("Idle")
        # first event
        f.set_event("stop")
        # end state
        f.set_end_state("End")

    # fsm loop
    running = True
    while running:
        funct = f.run()  # function to be executed in the new state
        if f.curState != f.endState:
            newEvent = funct()  # new event when state action is finished
            print("New Event : ", newEvent)
            f.set_event(newEvent)  # set new event for next transition
        else:
            funct()
            running = False

    print("End of the programm")
