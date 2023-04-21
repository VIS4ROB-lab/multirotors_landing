#!/usr/bin/python

import socket
import cv2
import numpy as np
from threading import Thread


class TcpClientTest:
    def __init__(self, image_path, id=0):

        image_test = cv2.imread(image_path)
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '127.0.0.1'
        port = 12334 + id

        print('Waiting for connection')
        try:
            client_socket.connect((host, port))
        except socket.error as e:
            print(str(e))
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        result, imgencode = cv2.imencode('.jpg', image_test, encode_param)
        data = np.array(imgencode)
        stringData = data.tostring()

        while True:
            cv2.imshow('Client', image_test)
            cv2.waitKey(1000)
            try:
                client_socket.sendall(str('L83F').encode('utf-8'))
                client_socket.sendall(str(len(stringData)).ljust(16).encode('utf-8'))
                client_socket.sendall(stringData)
                response = client_socket.recv(2048)
                print(response.decode('utf-8'))
            except socket.error as e:
                print(str(e))


class ClientHandlerThread(Thread):
    def __init__(self, connection, processing_function, verbose=False):
        Thread.__init__(self)
        self.connection = connection
        self.running = True
        self.func = processing_function
        self.verbose = verbose

    def recvall(self, count):
        buf = b''
        while count:
            new_buffer = self.connection.recv(count)
            if not new_buffer:
                return None
            buf += new_buffer
            count -= len(new_buffer)

        return buf

    def recv_image(self):
        magic_id = self.connection.recv(4)
        if not magic_id:
            return None

        if magic_id.decode('utf-8') != 'L83F':
            print("Client sent a strange msg, something wrong...: {}".format(magic_id.decode('utf-8')))
            return None

        data_header = self.connection.recv(16)
        image_size = int(data_header.decode('utf-8'))

        if self.verbose:
            print("Client sent a image with: {} (bytes)".format(image_size))
        buffer = self.recvall(image_size)
        return buffer

    def terminate(self):
        self.running = False

    def run(self):
        while self.running:
            try:
                rgb_image = self.recv_image()
                if rgb_image is None:
                    continue

                rgb_image = np.fromstring(rgb_image, dtype=np.uint8).reshape((480, 752, 3))

                # Show
                cv2.imshow("Server", rgb_image)
                cv2.waitKey(10)

                # Send back
                result = self.func(rgb_image)
                self.connection.sendall(bytes(result))
                if self.verbose:
                    print('send back: {}'.format(len(bytes(result))))

            except KeyboardInterrupt:  # exit from the client
                break
            except socket.timeout:
                pass
            except ConnectionResetError:
                print('\033[93mConnection reset error! Keep going\033[0m')

        self.connection.close()
        print("connection with client closed")


class TcpServer:
    def __init__(self, processing_function, id, verbose=False):

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        host = '127.0.0.1'
        port = 12334 + id

        try:
            server_socket.bind((host, port))
        except socket.error as e:
            print("Socket error: " + str(e))

        print('Waiting for a Connection.. on {}:{}'.format(host, port))
        server_socket.listen(5)
        server_socket.settimeout(1.0)

        def recvall(connection, count):
            buf = b''
            while count:
                new_buffer = connection.recv(count)
                if not new_buffer:
                    return None
                buf += new_buffer
                print(len(new_buffer))
                count -= len(new_buffer)

            return buf

        current_client_thread = None
        thread_count = 0

        while True:
            try:
                client_conn, address = server_socket.accept()
                if client_conn:
                    if current_client_thread:
                        print('Terminating previous connection ' + str(port) +
                              '- ' + 'Connection Number: ' + str(thread_count))
                        current_client_thread.terminate()
                        current_client_thread.join()
                        print('Terminated previous connection (port {})'.format(port))
                    print('Connected to: ' + address[0] + ':' + str(address[1]))
                    current_client_thread = ClientHandlerThread(client_conn, processing_function, verbose=verbose)
                    current_client_thread.start()
                    thread_count += 1
                    print('Connection Number (port {}) {}'.format(port, thread_count))
            except socket.timeout:
                pass
            except ConnectionResetError:
                print('\033[93mConnection reset error! Keep going\033[0m')
            except KeyboardInterrupt: # exit from the server
                current_client_thread.terminate()
                current_client_thread.join(100)
                server_socket.close()
                print("socket closed")
                break
            except:
                raise


class TcpCommunicator:
    def __init__(self, processing_function, num_sockets=1):
        # Create and starts the threads responsible for TCP connections
        threads = []
        for i in range(num_sockets):
            print("Starting thread {}".format(i + 1))
            threads.append(Thread(target=TcpServer, args=(processing_function, i),
                                  daemon=True))
            threads[-1].start()
        print("Threads successfully initialized!")

        # Here multi-threading happens
        for i in range(num_sockets):
            threads[i].join()
