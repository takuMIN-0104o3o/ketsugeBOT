import hid


class PingFind:
    def __init__(self, dev):
        self._dev = dev

    @classmethod
    def getMouse(cls, vid=0x046d, pid=0xc539):
        found = False
        for ping_code in range(256):
            dev = find_mouse_device(vid, pid, ping_code)
            if dev:
                ping_code_hex = format(ping_code, '02x')
                print(f"Mouse found with ping code: 0x{ping_code_hex}")
                found = True
                yield cls(dev)
        if not found:
            raise DeviceNotFoundError("Mouse not found with any ping code.")


class DeviceNotFoundError(Exception):
    pass


def check_ping(dev, ping_code):
    dev.write([0, ping_code])
    try:
        resp = dev.read(max_length=1, timeout_ms=10)
    except OSError as e:
        return False
    else:
        return resp and resp[0] == ping_code


def find_mouse_device(vid, pid, ping_code):
    dev = hid.device()
    for dev_info in hid.enumerate(vid, pid):
        dev.open_path(dev_info['path'])
        found = check_ping(dev, ping_code)
        if found:
            return dev
        else:
            dev.close()
    return None


for mouse in PingFind.getMouse():
    pass
