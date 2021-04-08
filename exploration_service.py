##Author(s): the one and only emma
import struct

from _bleio import PacketBuffer

from adafruit_ble.attributes import Attribute
from adafruit_ble.characteristics import Characteristic, ComplexCharacteristic
from adafruit_ble_adafruit.adafruit_service import AdafruitService


class _ExplorationPacket(ComplexCharacteristic):
    uuid = AdafruitService.adafruit_service_uuid(0x551)

    format = "<HH"
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


class ExplorationService(AdafruitService):
    # robot searches for sun

    uuid = AdafruitService.adafruit_service_uuid(0x550)
    _exploration_packet = _ExplorationPacket()

    def __init__(self, service=None):
        super().__init__(service=service)
        self._exploration_packet_buf = bytearray(_ExplorationPacket.format_size)

    @property
    def exploration(self):
        buf = self._exploration_packet_buf
        if self._exploration_packet.readinto(buf) == 0:
            # No new values available.
            return None
        return struct.unpack(_ExplorationPacket.format, buf)

    def explore(self, go, stop):
        self._exploration_packet = struct.pack(
            _ExplorationPacket.format,
            go, stop
        )
