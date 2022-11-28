from fsm import Fsm
from dartv2b import DartV2


# functions (actions of the fsm)
def doWait():
    print(">>>>>> action")
    my_bot.fast_compass_calibration()

    return 'continue'


def doForwardWalls():
    print(">>>>>> action")
    my_bot.follow_walls()

    event = None
    while event is None:
        event = my_bot.get_free_turn()

    return event


def doTurnLeft():
    print(">>>>>> action")
    my_bot.turn_left()

    my_bot.turnCount -= 1
    if my_bot.turnCount == 0:
        event = 'stop'
    else:
        event = 'continue'

    return event


def doTurnRight():
    print(">>>>>> action")
    my_bot.turn_right()

    my_bot.turnCount -= 1
    if my_bot.turnCount == 0:
        event = 'stop'
    else:
        event = 'continue'

    return event


def doForwardCap():
    print(">>>>>> action")
    my_bot.go_straight_to_obs_compass()

    event = None
    while event is None:
        event = my_bot.get_free_turn()

    return event


def doStop():
    print(">>>>>> action")
    return


if __name__ == "__main__":
    f = Fsm()  # finite state machine
    f.load_fsm_from_file("fsm_dart_challenge_2020.txt")

    # create a robot (to be replaced by dartv2)
    my_bot = DartV2()
    my_bot.turnCount = 12

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
