import time
import struct
import usb.core
import usb.util

from respeaker_ros.parameters import PARAMETERS


class RespeakerInterface(object):
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    TIMEOUT = 100000

    def __init__(self):
        try:
            from pixel_ring import usb_pixel_ring_v2
        except IOError as e:
            print(e)
            raise RuntimeError("Check the device is connected and recognized")

        self.dev = usb.core.find(idVendor=self.VENDOR_ID,
                                 idProduct=self.PRODUCT_ID)
        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")
        try:
            self.dev.reset()
            self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
            self.set_led_think()
            time.sleep(10)  # it will take 10 seconds to re-recognize as audio device
            self.set_led_trace()
            self.pixel_ring_available = True
        except Exception as e:
            print(f"Warning: Could not initialize pixel ring: {e}")
            print("ReSpeaker will work without LED control. Run fix_respeaker_usb.py to fix this.")
            self.pixel_ring = None
            self.pixel_ring_available = False

    def __del__(self):
        try:
            self.close()
        except:
            pass
        finally:
            self.dev = None

    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self.TIMEOUT)

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)

        response = struct.unpack(b'ii', response)

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2. ** response[1])

        return result

    def set_led_think(self):
        if self.pixel_ring_available and self.pixel_ring:
            try:
                self.pixel_ring.set_brightness(10)
                self.pixel_ring.think()
            except Exception as e:
                print(f"Warning: LED control failed: {e}")

    def set_led_trace(self):
        if self.pixel_ring_available and self.pixel_ring:
            try:
                self.pixel_ring.set_brightness(20)
                self.pixel_ring.trace()
            except Exception as e:
                print(f"Warning: LED control failed: {e}")

    def set_led_color(self, r, g, b, a):
        if self.pixel_ring_available and self.pixel_ring:
            try:
                self.pixel_ring.set_brightness(int(20 * a))
                self.pixel_ring.set_color(r=int(r * 255), g=int(g * 255), b=int(b * 255))
            except Exception as e:
                print(f"Warning: LED control failed: {e}")

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    @property
    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)
