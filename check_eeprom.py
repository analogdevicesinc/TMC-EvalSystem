
from pytrinamic.connections import ConnectionManager
from pytrinamic.tmcl import TMCLCommand


with ConnectionManager().connect() as my_interface:
    spi_channel = 1
    address = 0
    if 0:
        response = my_interface.send(TMCLCommand.TMCL_UF1, spi_channel, 0, address)
        data = response.value.to_bytes(length=4, byteorder="little", signed=False)
        print(",".join([f"0x{x:02x}" for x in data]))
    else:
        #data = bytes([0x42, 0x2a, 0x5b, 0x9b])
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF])
        for i, bt in enumerate(data):
            my_interface.send(TMCLCommand.TMCL_UF2, spi_channel, bt, address + i)

