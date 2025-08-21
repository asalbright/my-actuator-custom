import time
import socket
import struct


def udp_server():
    # Server configuration
    HOST = "0.0.0.0"  # Listen on localhost
    PORT = 12345  # Port to listen on

    # Create UDP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Bind socket to address
        server_socket.bind((HOST, PORT))
        print(f"UDP Server listening on {HOST}:{PORT}")

        while True:
            try:
                # Receive data from client
                data, client_address = server_socket.recvfrom(1024)

                # Decode and display received data
                sequence_number, x = struct.unpack("!I1f", data)
                message = f"Seq: {sequence_number}, X: {x:.2f}"
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

                print(f"[{timestamp}] Received from {client_address}: {message}")

                # Send acknowledgment back to client
                ack_message = f"ACK: Received '{message}'"
                server_socket.sendto(ack_message.encode("utf-8"), client_address)

            except KeyboardInterrupt:
                print("\nShutting down server...")
                break
            except Exception as e:
                print(f"Error handling client: {e}")

    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()
        print("Server socket closed")


if __name__ == "__main__":
    udp_server()
