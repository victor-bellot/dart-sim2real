import time
from fsm import Fsm
from dartv2b_simu import DartV2


# functions (actions of the fsm)
def doWait():
    print(">>>>>> action")
    my_bot.wait(duration=1)

    if time.time() - my_bot.time_start > my_bot.begin_wait:  # auto start in 5 seconds
        event = "continue"
    else:
        event = "stay"

    return event


def doForward():
    print(">>>>>> action")
    my_bot.go_straight(duration=5)

    return 'continue'


def doTurn():
    print(">>>>>> action")
    my_bot.half_turn()

    my_bot.turnCount -= 1
    if my_bot.turnCount == 0:
        event = 'stop'
    else:
        event = 'continue'

    return event


def doStop():
    print(">>>>>> action")
    return


if __name__ == "__main__":
    f = Fsm()  # finite state machine
    f.load_fsm_from_file("fsm_b&f.txt")

    # create a robot (to be replaced by dartv2)
    my_bot = DartV2()
    my_bot.turnCount = 2

    # fsm loop
    running = True
    while running:
        action = f.run()  # function to be executed in the new state
        nextEvent = action()  # new event when state action is finished
        if f.curState != f.endState:
            print("New Event : ", nextEvent)
            f.set_event(nextEvent)  # set new event for next transition
        else:
            running = False

    print("End of the program")
    exit()
