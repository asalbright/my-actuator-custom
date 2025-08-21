import can
import struct  # Add this import at the top of your file

"""
In your code, you use this this with:

    from manualCAN import ManualCAN
    with ManualCAN() as manual_can:

then whenever you want to get the gain value, you can call:

        gain_value = manual_can.get_gain()
"""


class ManualCAN:
    # gain_id = 0x141
    # receive_data = 0x241

    def __init__(self, channel="can0", bustype="socketcan", listen_timeout=0.5):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        self.listen_timeout = listen_timeout

    def get_gain(self, message_id):
        """
        Read the gain value from the CAN bus and format the output.
        """
        responses = {}
        for i in [1, 2, 4, 5, 7, 8, 9]:  # Indices to request
            # Encode the request for the current index
            encoded_request = self.encode_request(i)
            if encoded_request is None:
                continue

            # Send the request
            self.bus.send(
                can.Message(
                    arbitration_id=message_id,
                    data=encoded_request,
                    is_extended_id=False,
                )
            )
            print(f"Sent request for index {i}: {encoded_request}")

            while True:
                # Wait for the response
                response = self.bus.recv(timeout=self.listen_timeout)
                if response is None:
                    print(f"No response received for index {i}")
                    break

                # Check if the response matches the requested index
                if (
                    response.arbitration_id == message_id + 0x100
                    and response.data[1] == i
                ):
                    # print(f"Matched response for index {i}: {response.data}")
                    # Decode the response
                    decoded_response = self.decode_response(response)
                    if decoded_response is not None:
                        responses[i] = decoded_response
                    break
                else:
                    print(f"Ignored response: {response.data}")

        if not responses:
            raise RuntimeError("No valid responses received from CAN bus.")

        # Format the responses into the desired string
        formatted_output = (
            f"current: {{kp: {round(responses.get(1, 0), 4)}, ki: {round(responses.get(2, 0), 4)}}}, "
            f"speed: {{kp: {round(responses.get(4, 0), 4)}, ki: {round(responses.get(5, 0), 4)}}}, "
            f"position: {{kp: {round(responses.get(7, 0), 4)}, ki: {round(responses.get(8, 0), 4)}, kd: {round(responses.get(9, 0), 4)}}}"
        )

        return formatted_output

    def encode_request(self, i):
        """
        Encode the request to send to the CAN bus.
        """
        # TODO: Implement the actual encoding logic
        return [0x30, i, 0, 0, 0, 0, 0, 0]

    def decode_response(self, response):
        """
        Decode the response from the CAN bus.
        """
        # Extract the data payload from the CAN message
        data = response.data  # Access the 'data' attribute of the 'response' object
        # print(f"Raw data received: {data}")  # Debugging: Print raw data

        # Ensure the data has at least 4 bytes to decode
        if len(data) < 4:
            raise ValueError("Response data is too short to decode.")

        # Extract the last 4 bytes
        float_bytes = data[-4:]
        # print(f"Last 4 bytes for decoding: {float_bytes}")  # Debugging: Print the bytes being decoded

        # Decode the 4 bytes as an IEEE 754 float (little-endian)
        decoded_float = struct.unpack("<f", bytes(float_bytes))[
            0
        ]  # '<f' for little-endian float
        rounded_float = round(decoded_float, 4)  # Round to 4 decimal places
        # print(f"Decoded and rounded float value: {rounded_float}")  # Debugging: Print the rounded float value

        return rounded_float

    def set_gains(self, message_id, gain_type, kp, ki, kd=None):
        """
        Send separate CAN messages to set the gains (current, speed, or position).

        Parameters:
            message_id: The CAN message ID to send to.
            gain_type: The type of gain ('current', 'speed', or 'position').
            kp: The proportional gain (float).
            ki: The integral gain (float).
            kd: The derivative gain (float, only for 'position').
        """
        gain_map = {"current": [1, 2], "speed": [4, 5], "position": [7, 8, 9]}

        if gain_type not in gain_map:
            raise ValueError(
                f"Invalid gain type: {gain_type}. Must be 'current', 'speed', or 'position'."
            )

        indices = gain_map[gain_type]

        kp_bytes = struct.pack("<f", kp)
        ki_bytes = struct.pack("<f", ki)

        print(f"kp_bytes: {list(kp_bytes)}")
        print(f"ki_bytes: {list(ki_bytes)}")

        self.bus.send(
            can.Message(
                arbitration_id=message_id,
                data=[0x32, indices[0]] + [0] * 2 + list(kp_bytes),
                is_extended_id=False,
            )
        )
        self.bus.send(
            can.Message(
                arbitration_id=message_id,
                data=[0x32, indices[1]] + [0] * 2 + list(ki_bytes),
                is_extended_id=False,
            )
        )

        if gain_type == "position":
            if kd is None:
                raise ValueError("For 'position', you must provide a value for kd.")
            kd_bytes = struct.pack("<f", kd)
            print(f"kd_bytes: {list(kd_bytes)}")
            self.bus.send(
                can.Message(
                    arbitration_id=message_id,
                    data=[0x32, indices[2]] + [0] * 2 + list(kd_bytes),
                    is_extended_id=False,
                )
            )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.bus.shutdown()
