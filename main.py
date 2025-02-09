from inputs import get_gamepad

state = 0


# middle button (between select and start is not mapped)
mapped_keys = {
    "BTN_SOUTH": "A", # 0
    "BTN_NORTH": "Y", # 0
    "BTN_EAST": "B", # 0
    "BTN_WEST": "X", # 0
    "BTN_TR": "RB", # [0,1]
    "BTN_TL": "LB", # [0.1]
    "ABS_Z": "LT", # 0 - 1023
    "ABS_RZ": "RT", # 0 - 1023
    "BTN_START": "Start", # [0,1]
    "BTN_SELECT": "Select", # [0,1],

    # this is used for joysticks (lsb - rsb) plus the d-pad
    # 2^16, half for up, half for down [-32768, 32768]
    "ABS_Y": "LSB Y",
    "ABS_X": "LSB X",
    "ABS_RY": "RSB Y",
    "ABS_RX": "RSB X",
    "ABS_HAT0Y": "DPAD Y", # [0,1]
    "ABS_HAT0X": "DPAD X", # [0,1]
}


while True:
    events = get_gamepad()
    # https://docs.kernel.org/input/event-codes.html
    for event in events:
        if event.code in mapped_keys:
            # print(mapped_keys[event.code])
            if event.code == "ABS_Y":
                print(event.state)
        else:
            print(f"type: {event.ev_type}, code: {event.code}, state: {event.state}")

