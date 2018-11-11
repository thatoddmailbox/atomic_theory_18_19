package org.firstinspires.ftc.teamcode.utils;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

public class PIDLogger {
    InetAddress listener;
    DatagramSocket socket;

    public PIDLogger(String listenerName) throws UnknownHostException, SocketException {
        listener = InetAddress.getByName(listenerName);
        socket = new DatagramSocket();
    }

    private byte[] doubleToByteArray(double d) {
        byte[] output = new byte[8];
        long lng = Double.doubleToLongBits(d);
        for(int i = 0; i < 8; i++) output[i] = (byte)((lng >> ((7 - i) * 8)) & 0xff);
        return output;
    }

    public void sendPacket(double target, double current, double output) {
        byte[] packetData = new byte[8 * 3];
        System.arraycopy(doubleToByteArray(target), 0, packetData, 0, 8);
        System.arraycopy(doubleToByteArray(current), 0, packetData, 8, 8);
        System.arraycopy(doubleToByteArray(output), 0, packetData, 8 * 2, 8);

        DatagramPacket packet = new DatagramPacket(packetData, 8 * 3, listener, 4174);

        try {
            socket.send(packet);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
