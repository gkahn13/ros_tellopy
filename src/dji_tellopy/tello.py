import libh264decoder
import numpy as np
import socket
import threading
import time


class Tello(object):

    LOCAL_IP = '' # local
    TELLO_IP = '192.168.10.1'

    CMD_PORT = 8889
    STATE_PORT = 8890
    VIDEO_PORT = 11111

    def __init__(self, init_sleep_time=2.0, command_timeout=0.3):
        # create sockets
        self._socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sockets = [self._socket_cmd, self._socket_state, self._socket_video]

        # bind ports
        self._socket_cmd.bind((Tello.LOCAL_IP, Tello.CMD_PORT))
        self._socket_state.bind((Tello.LOCAL_IP, Tello.STATE_PORT))
        self._socket_video.bind((Tello.LOCAL_IP, Tello.VIDEO_PORT))

        # initializing commands
        self._socket_cmd.sendto(b'command', (Tello.TELLO_IP, Tello.CMD_PORT))
        self._socket_cmd.sendto(b'streamon', (Tello.TELLO_IP, Tello.CMD_PORT))
        time.sleep(init_sleep_time)

        # command
        self._cmd_response = None
        self._cmd_timeout = command_timeout
        self._cmd_lock = threading.RLock()
        self._cmd_thread = threading.Thread(target=self._receive_cmd_thread)
        self._cmd_thread.daemon = True
        self._cmd_thread.start()

        # state
        self._state_dict = None
        self._state_thread = threading.Thread(target=self._receive_state_thread)
        self._state_thread.daemon = True
        self._state_thread.start()

        # video
        self._video_frame = None
        self._video_decoder = libh264decoder.H264Decoder()
        self._video_thread = threading.Thread(target=self._receive_video_thread)
        self._video_thread.daemon = True
        self._video_thread.start()

    def __del__(self):
        """Closes the local socket."""
        for socket in self._sockets:
            socket.close()

    ###############
    ### Command ###
    ###############

    def _receive_cmd_thread(self):
        """Listen to responses from the Tello.

        Runs as a thread, sets self._cmd_response to whatever the Tello last returned.

        """
        while True:
            try:
                self._cmd_response, ip = self._socket_cmd.recvfrom(3000)
            except socket.error as exc:
                print ("Caught exception socket.error : %s" % exc)

    def _send_command(self, command):
        """
        Send a command to the Tello and wait for a response.

        :param command: Command to send.
        :return (bool): Success

        """
        with self._cmd_lock:
            self._socket_cmd.sendto(command.encode('utf-8'), self.tello_address)

            start_time = time.time()
            while self._cmd_response is None:
                if time.time() - start_time > self._cmd_timeout:
                    break

            success =  (self._cmd_response.lower() == 'ok')
            self._cmd_response = None

        return success

    def takeoff(self):
        """Initiates take-off.

        Returns:
            bool: command success

        """
        return self._send_command('takeoff')

    def land(self):
        """Initiates landing.

        Returns:
            bool: command success

        """

        return self._send_command('land')

    def emergency(self):
        """Emergency landing.

        Returns:
            bool: command success

        """

        return self._send_command('emergency')

    def set_velocity(self, vx, vy, vz, vyaw):
        """

        :param vx (float): m/s
        :param vy (float): m/s
        :param vz (float): m/s:
        :param vyaw (float): rad/s
        :return (bool): command success
        """
        vx_cm_s = int(100 * vx)
        vy_cm_s = int(100 * vy)
        vz_cm_s = int(100 * vz)
        vyaw_deg_s = int(np.rad2deg(vyaw))
        
        return self._send_command('rc {vy:d} {vx:d} {vz:d} {vyaw:d}'.format(
            vx=vx_cm_s, vy=vy_cm_s, vz=vz_cm_s, vyaw=vyaw_deg_s
        ))

    #############
    ### State ###
    #############

    def get_state(self):
        return self._state_dict

    def _receive_state_thread(self):
        """Listen to responses from the Tello.

        Runs as a thread, sets # TODO

        """
        while True:
            try:
                response, ip = self._socket_state.recvfrom(3000)

                # parse string
                names_and_value_strs = response.strip().split(';')[:-1]
                d = dict()
                for name_and_value_str in names_and_value_strs:
                    name, value_str = name_and_value_str.split(':')
                    value = float(value_str)
                    d[name] = value

                # convert to metric
                d_metric = {
                    'acceleration': 0.01 * np.array([d['agx'], d['agy'], d['agz']]),
                    'velocity': 0.01 * np.array([d['vgx'], d['vgy'], d['vgz']]),
                    'rpy': np.deg2rad(np.array([d['roll'], d['pitch'], d['yaw']])),
                    'battery': d['bat'],
                    'barometer': 0.01 * d['baro'],
                    'height': 0.01 * d['tof'] if d['tof'] < 6550 else 0.,
                }

                self._state_dict = d_metric
            except socket.error as exc:
                print ("Caught exception socket.error : %s" % exc)
            except:
                pass

    #############
    ### Video ###
    #############

    def get_video_frame(self):
        """Return the last frame from camera."""
        return self._video_frame

    def _receive_video_thread(self):
        """
        Listens for video streaming (raw h264) from the Tello.

        Runs as a thread, sets self.frame to the most recent frame Tello captured.

        """
        packet_data = ""
        while True:
            try:
                res_string, ip = self._socket_video.recvfrom(2048)
                packet_data += res_string
                # end of frame
                if len(res_string) != 1460:
                    for frame in self._h264_decode(packet_data):
                        self._video_frame = frame
                    packet_data = ""

            except socket.error as exc:
                print("Caught exception socket.error : %s" % exc)

    def _h264_decode(self, packet_data):
        """
        decode raw h264 format data from Tello

        :param packet_data: raw h264 data array

        :return: a list of decoded frame
        """
        res_frame_list = []
        frames = self._video_decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                # print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)

                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, ls / 3, 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)

        return res_frame_list


if __name__ == '__main__':
    tello = Tello()

    import matplotlib.pyplot as plt
    f, ax = plt.subplots(1, 1, figsize=(10, 10))
    imshow = None

    import rospy
    rospy.init_node('bla', anonymous=True)
    rate = rospy.Rate(15.)
    while not rospy.is_shutdown():
        # rate.sleep()

        print(tello.get_state()['velocity'])

        img = tello.get_video_frame()
        if img is not None:
            img = np.array(img)
            if imshow is None:
                imshow = ax.imshow(img)
                plt.show(block=False)
                plt.pause(0.1)
            else:
                imshow.set_data(img)
            f.canvas.draw()

