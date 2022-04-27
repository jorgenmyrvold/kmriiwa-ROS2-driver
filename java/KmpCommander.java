// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


package kmr.test;


import kmr.test.KmpJogger;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;

//Daniel
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformPosition;
//import javax.task.Inject;
//import com.kuka.task.ITaskLogger;


public class KmpCommander extends Node {

	// Robot
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = {0.0,0.0,0.0};
	KmpJogger kmp_jogger;
	long jogging_period  = 20L;

	/*
	public KMP_commander(KmpOmniMove robot) {
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp, jogging_period);
	}
	*/

	public KmpCommander(String remote_ip, int port, KmpOmniMove robot, String ConnectionType ){
		super(remote_ip, port, ConnectionType, "KMP commander");
		this.kmp = robot;
		this.kmp_jogger = new KmpJogger((ICartesianJoggingSupport)kmp, jogging_period);
		
		if (!(isSocketConnected())) {
			Thread monitorKMPCommandConnections = new MonitorKMPCommandConnectionsThread();
			monitorKMPCommandConnections.start();
			} else {
				setisKMPConnected(true);
		}
	}

	public void run() {
		Thread emergencyStopThread = new MonitorEmergencyStopThread();
		emergencyStopThread.start();
		
		while(isNodeRunning()) {
			String Commandstr = this.socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if(!getShutdown() && !closed) {
		    	if ((splt[0]).equals("")) { //HER SKAL DET EGENTLIG STÅ "shutdown"!!!!!!!!!!!
		    		System.out.println("KMP received shutdown");
					setShutdown(true);	
					break;
				}
		
		    	if ((splt[0]).equals("setTwist") && (!getEmergencyStop() || true)) {
					setNewVelocity(Commandstr);
					System.out.println(Commandstr + "asd\n");
				}
		    	//this.Logger = getLoer();
		    	MobilePlatformPosition MobilePlatformRelativeMotion_mesage=MobilePlatformRelativeMotion.getCoveredDistance(KMP_currentMotion);
				System.out.println("Covered distance " + "X=" + MobilePlatformRelativeMotion_mesage.getX() + ", Y=" + MobilePlatformRelativeMotion_mesage.getY() +", Theta=" + MobilePlatformRelativeMotion_mesage.getTheta());
		    	//this.socket.send_message(MobilePlatformRelativeMotion_mesage); 
			    //get_logger().info("Covered distance " + "X=" + MobilePlatformRelativeMotion_mesage.getX() + ", Y=" + MobilePlatformRelativeMotion_mesage.getY() +", Theta=" + MobilePlatformRelativeMotion_mesage.getTheta());
			    //this.socket.send_message("Covered distance " + "X=" + MobilePlatformRelativeMotion_mesage.getX() + ", Y=" + MobilePlatformRelativeMotion_mesage.getY() +", Theta=" + MobilePlatformRelativeMotion_mesage.getTheta());
	    	}
		}
		
		System.out.println("KMPcommander no longer running");
		
		/* SIMPLE JOGGER
        this.kmp_jogger.startJoggingExecution();
        setNewVelocity("lmao 0.05 0 0.1");
		while(true)
		{
			continue;
		}
		*/
	}
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(isNodeRunning()) {
				if (getEmergencyStop() && getisKMPMoving()){
					setisKMPMoving(false);
					kmp_jogger.killJoggingExecution(getisKMPMoving());
					System.out.println("successfully killed jogging execution from emergency stop");
					if(!(KMP_currentMotion==null)){
						KMP_currentMotion.cancel();
					}
				}
			}
		}
	}
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				createSocket();
				if (isSocketConnected()){
					setisKMPConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println(node_name + " connection thread could not sleep");
				}
			}
			if(!closed){
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}

	public void setNewVelocity(String vel) {
		String []lineSplt = vel.split(" ");

		if(lineSplt.length==4) {
			this.velocities[0] = Double.parseDouble(lineSplt[1]); 	// x
			this.velocities[1] = Double.parseDouble(lineSplt[2]); 	// y
			this.velocities[2] = Double.parseDouble(lineSplt[3]);	// theta
			
			if(velocities[0] == 0 && velocities[1] == 0 && velocities[2] == 0) {
				if(getisKMPMoving()) {
					setisKMPMoving(false);
					this.kmp_jogger.killJoggingExecution(true);
				}
			} else {
				if(getisKMPMoving()&& (!getEmergencyStop() || true)) {
					this.kmp_jogger.updateVelocities(this.velocities);
				}
				else if(!getisKMPMoving() && (!getEmergencyStop() || true) && kmp.isReadyToMove()) {
					setisKMPMoving(true);
					this.kmp_jogger.updateVelocities(this.velocities);
					this.kmp_jogger.startJoggingExecution();
				} else {
					System.out.println("Can not jog robot, is ready to move is: " +kmp.isReadyToMove() + " calculated value: " + KmpOmniMove.calculateReadyToMove(kmp.getSafetyState()));
				}
					
			}
		}
	}
	
	/*
	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");

		if(lineSplt.length==4){
				this.velocities[0] = Double.parseDouble(lineSplt[1]);	// x
				this.velocities[1] = Double.parseDouble(lineSplt[2]);	// y
				this.velocities[2] = Double.parseDouble(lineSplt[3]);	// theta
				
				this.kmp_jogger.updateVelocities(this.velocities);
        }
        
	}
	*/
	
	/**
	 * Override the Thread.close() function to kill jogger and close socket.
	 */
	@Override
	public void close() {
		closed = true;
		try {
			this.kmp_jogger.killJoggingExecution(getisKMPMoving());
			System.out.println("KMPJogger ended successfully");
		}catch(Exception e){
			System.out.println("Could not kill jogging execution");
		}
		try {
			this.socket.close();
		}catch(Exception e) {
			System.out.println("Could not close KMP commander connection: " +e);
		}
		System.out.println("KMP commander closed!");
 	}
}