import asyncio
from bleak import BleakClient
import struct

ADDRESS = "DA:D3:83:42:56:01"
# ISPRAVLJENI UUID na 00002a70-0000-1000-8000-00805f9b34fb
# Ovo je "Angle Measurement Characteristic" prema dostavljenom kodu
ANGLE_MEASUREMENT_UUID = "00002a70-0000-1000-8000-00805f9b34fb"

def notification_handler(sender, data):
    """
    Rukovatelj obavijestima koji prima podatke i dekodira ih.
    """
    print(f"Notification from sender handle {sender}: Raw data (hex): {data.hex()}")

    # Format podataka: 8 bajtova, dva little-endian float broja
    if len(data) == 8:
        try:
            # '<ff' raspakira dva little-endian float broja
            angle1, angle2 = struct.unpack('<ff', data)
            print(f"  Decoded Angles: Angle 1 = {angle1:.4f} degrees, Angle 2 = {angle2:.4f} degrees")
        except struct.error as e:
            print(f"  Error decoding data as two floats: {e}. Raw data length: {len(data)} bytes.")
    else:
        print(f"  Received data of unexpected length: {len(data)} bytes. Expected 8 bytes.")
        # Opcionalno, pokušajte dekodirati kao string ako format nije poznat
        try:
            decoded_str = data.decode('utf-8', errors='ignore').strip()
            if decoded_str: # Ispiši samo ako nije prazan string
                print(f"  Attempted string decode: '{decoded_str}'")
        except:
            pass


async def run():
    print(f"Attempting to connect to {ADDRESS}...")
    try:
        async with BleakClient(ADDRESS) as client:
            connected = client.is_connected
            if connected:
                print(f"Successfully connected to {ADDRESS}")

                try:
                    await client.start_notify(ANGLE_MEASUREMENT_UUID, notification_handler)
                    print(f"Subscribed to notifications for UUID: {ANGLE_MEASUREMENT_UUID}. Listening for 30 seconds...")
                    await asyncio.sleep(30) # Neka sluša 30 sekundi, produžite ako treba
                    print("Stopping notifications.")
                    await client.stop_notify(ANGLE_MEASUREMENT_UUID)
                except Exception as e:
                    print(f"Error subscribing or handling notifications: {e}")
                    print(f"Make sure the device is advertising and the characteristic {ANGLE_MEASUREMENT_UUID} is notifying.")
            else:
                print("Failed to connect after initial attempt.")
    except Exception as e:
        print(f"An error occurred during connection: {e}")
        print("Common issues: Bluetooth adapter not ready, device out of range, or permissions.")

if __name__ == "__main__":
    asyncio.run(run())

# import asyncio
# from bleak import BleakClient
# import struct

# ADDRESS = "DA:D3:83:42:56:01"
# # Ažuriran UUID na najvjerojatniji kandidat
# ANGLE_MEASUREMENT_UUID = "00001531-1212-efde-1523-785feabcd123"

# def notification_handler(sender, data):
#     """
#     Rukovatelj obavijestima koji prima podatke i pokušava ih interpretirati.
#     """
#     print(f"Notification from sender handle {sender}: Raw data (hex): {data.hex()}")

#     # Pokušajte s različitim načinima dekodiranja ako ne vidite smislene podatke
#     # Ovisno o tome kako senzor šalje podatke (npr. integer, float, string)
#     try:
#         # Primjer: Ako podaci predstavljaju jedan 4-bajtni float
#         # Ako vaš senzor šalje kut kao float, to je čest format.
#         # '<f' označava little-endian float.
#         if len(data) == 4:
#             angle_value = struct.unpack('<f', data)[0]
#             print(f"  Decoded as float: {angle_value:.2f} degrees")
#         # Primjer: Ako podaci predstavljaju dva 2-bajtna signed integera (npr. x i y os)
#         elif len(data) == 4: # 2 shorts * 2 bytes/short = 4 bytes
#             x, y = struct.unpack('<hh', data)
#             print(f"  Decoded as two int16 (X, Y): {x}, {y}")
#         # Primjer: Ako podaci predstavljaju jedan 2-bajtni signed integer
#         elif len(data) == 2:
#             value = struct.unpack('<h', data)[0]
#             print(f"  Decoded as int16: {value}")
#         else:
#             print(f"  Data length ({len(data)} bytes) does not match common formats for this example.")
#     except struct.error as e:
#         print(f"  Error decoding data: {e}. Raw data length: {len(data)} bytes.")
#     except Exception as e:
#         print(f"  Unexpected error in decoding: {e}")


# async def run():
#     print(f"Attempting to connect to {ADDRESS}...")
#     try:
#         async with BleakClient(ADDRESS) as client:
#             connected = client.is_connected
#             if connected:
#                 print(f"Successfully connected to {ADDRESS}")

#                 try:
#                     await client.start_notify(ANGLE_MEASUREMENT_UUID, notification_handler)
#                     print(f"Subscribed to notifications for UUID: {ANGLE_MEASUREMENT_UUID}. Listening for 30 seconds...")
#                     await asyncio.sleep(120)
#                     print("Stopping notifications.")
#                     await client.stop_notify(ANGLE_MEASUREMENT_UUID)
#                 except Exception as e:
#                     print(f"Error subscribing or handling notifications: {e}")
#                     print(f"Please double-check if {ANGLE_MEASUREMENT_UUID} is a NOTIFY characteristic on your device. The presence of a 0x2902 Client Characteristic Configuration Descriptor suggests it should be.")
#             else:
#                 print("Failed to connect after initial attempt.")
#     except Exception as e:
#         print(f"An error occurred during connection: {e}")
#         print("Common issues: Bluetooth adapter not ready, device out of range, or permissions.")

# if __name__ == "__main__":
#     asyncio.run(run())