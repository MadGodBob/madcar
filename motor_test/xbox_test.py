import struct

with open('/dev/input/js0', 'rb') as js:
    while True:
        event = js.read(8)
        if event:
            time_ms, value, event_type, number = struct.unpack('IhBB', event)
            event_type = event_type & 0x7f
            
            if event_type == 1:
                state = "按下" if value else "松开"
                print(f"按键 {number:2d} {state}")
            elif event_type == 2:
                print(f"摇杆/扳机 {number:2d} 值: {value:6d}")