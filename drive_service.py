##Author(s): the one and only emma
import struct

from _bleio import PacketBuffer

from adafruit_ble.attributes import Attribute
from adafruit_ble.characteristics import Characteristic, ComplexCharacteristic
from adafruit_ble_adafruit.adafruit_service import AdafruitService


class _DrivePacket(ComplexCharacteristic):
    uuid = AdafruitService.adafruit_service_uuid(0x221)

    format = "<HHHH"
    format_size = struct.calcsize(format)

    def __init__(self):
        super().__init__(
            properties=Characteristic.WRITE,
            read_perm=Attribute.NO_ACCESS,
            max_length=self.format_size,
            fixed_length=True,
        )

    def bind(self, service):
        """Binds the characteristic to the given Service."""
        bound_characteristic = super().bind(service)
        return PacketBuffer(bound_characteristic, buffer_size=1)


class DriveService(AdafruitService):
    # Make robo go zoom zoom

    uuid = AdafruitService.adafruit_service_uuid(0x220)
    _drive_packet = _DrivePacket()

    def __init__(self, service=None):
        super().__init__(service=service)
        self._drive_packet_buf = bytearray(_DrivePacket.format_size)

    @property
    def drive(self):
        """Return (forward, backward, left, right), or None if no value available"""
        buf = self._drive_packet_buf
        if self._drive_packet.readinto(buf) == 0:
            # No new values available.
            return None
        return struct.unpack(_DrivePacket.format, buf)

    def move(self, forward, backward, left, right):
        self._drive_packet = struct.pack(
            _DrivePacket.format,
            forward, backward, left, right
        )
