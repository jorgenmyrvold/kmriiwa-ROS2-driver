package kmr.test;

import java.net.BindException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;

import kmr.test.ISocket;


public class UdpSocket implements ISocket{
	
	public volatile boolean isConnected;
	DatagramSocket UDPConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	public String remote_ip;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	int COMport;
    static BindException b;
    String nodename;

	
	public UdpSocket(String remote_ip, int port, String node_name) {
		isConnected = false;
		COMport = port;
		this.remote_ip = remote_ip;
		this.nodename=node_name;
        UDPConn = connect();
	}
	
	public DatagramSocket connect()
	{
		while (!isConnected){
			try{
				System.out.println("Connecting  "+ this.nodename+ " to ROS over UDP on port: " + COMport); 
		    	int kuka_port = this.COMport; // change this if cannot bind error
		        InetSocketAddress socket_address = new InetSocketAddress(kuka_port);
		    	

		    	int ros_port = this.COMport;
		        InetAddress address = InetAddress.getByName(this.remote_ip);
		        
		        UDPConn = new DatagramSocket(null); 
		        UDPConn.setReuseAddress(true);
		        
		        byte buf[] = "UDP Connenction established".getBytes();
		        byte buf1[] = new byte[1024]; 
		        
		        package_out = new DatagramPacket(buf, buf.length, address, ros_port); 
		        package_in = new DatagramPacket(buf1, buf1.length); 
		      
		        // send() method 
		        UDPConn.send(package_out); 
		  
		        // receive() method 
		        UDPConn.setSoTimeout(3000); //ms

		        UDPConn.receive(package_in); 
		        String s = decode(package_in);
		        System.out.println(this.nodename+ " received packet data over UDP on port : " + COMport + " Message: " +s);  
		        
		        if(s.length()<1){
                    UDPConn.close();
		       		isConnected=false;
		       		System.out.println( this.nodename+ "  did not receive any message in 3 seconds, shutting off");
		       		break;
		       	 }
		        UDPConn.setSoTimeout(0);
		        isConnected=true;
			}
			catch(Exception e1){
		        System.out.println("ERROR connecting  "+ this.nodename+ " to ROS over UDP on port: " + this.COMport + " Error: " + e1);
		        isConnected=false;
		        close();
				b = new BindException();
		        if(e1.getClass().equals(b.getClass())){
		        	e1.printStackTrace();
		        	break;
		        }
		        break;
		        }
			}
		return UDPConn;
	}
	
	public String decode(DatagramPacket pack)  {
    	byte[] data = pack.getData();
    	String message = new String(data,0,pack.getLength(), UTF8_CHARSET);
        return message;

    }

	@Override
    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }
    
	@Override
    public void send_message(String msg)
	{
    	byte[] bytlist = msg.getBytes(UTF8_CHARSET);
    	package_out.setData(bytlist);
    	package_out.setLength(bytlist.length);
     try {
    	 	UDPConn.send(package_out);
		} catch (Exception e) {
			System.out.println( this.nodename+ " could not send package over UDP on port: "  + this.COMport + " error: " + e);
		}
	}
    @Override
	public String receive_message()
	{
		String line;
		try{
			UDPConn.receive(package_in);
			line = decode(package_in);
	    	return line;
		}
		catch(Exception e){
			System.out.println("Error receiving package  "+ this.nodename+ " over UDP on port: " + this.COMport + " error: " + e);
			return " ";
		}
	}
	
    @Override
	public void close(){
		try {
			UDPConn.close();
			System.out.println("Connection to ROS closed on port: " + COMport);
			isConnected=false;
		} catch (Exception e) {
			System.out.println("ERROR closing the UDP communication for "+ this.nodename+ " on port to ROS: " + COMport);
		}
	}


	@Override
	public boolean isConnected() {	
		return this.isConnected;
	}

}