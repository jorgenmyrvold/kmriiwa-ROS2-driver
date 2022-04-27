
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

// Configuration
import javax.inject.Inject;
import javax.inject.Named;
import org.apache.log4j.BasicConfigurator;

// Implementated classes
import kmr.test.KmpCommander;
import kmr.test.LbrCommander;
import kmr.test.KMP_sensor_reader; //Daniel	

//RoboticsAPI
import com.kuka.roboticsAPI.annotations.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.LBR;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


import com.kuka.generated.ioAccess.ControlPanelIOGroup;

// AUT MODE: 3s, T1/T2/CRR: 2s
@ResumeAfterPauseEvent(delay = 0 ,  afterRepositioning = true)
public class KmpApp extends RoboticsAPIApplication {
	
	// Runtime Variables
	private volatile boolean AppRunning;
	private IAutomaticResumeFunction resumeFunction;
	
	// test
	// Declare KMP
	@Inject
	@Named("KMR_omniMove_200_1")
	public KmpOmniMove kmp;
	public Controller controller;

	// Declare LBR
	@Inject
	@Named("LBR_iiwa_14_R820_1")
	public LBR lbr;

	// Implemented node classes
	KmpCommander kmp_commander;
	LbrCommander lbr_commander;
	KMP_sensor_reader kmp_sensor_reader; //Daniel
	
	// Check if application is paused:
	@Inject
	ControlPanelIOGroup ControlPanelIO;

	// Connection types
	String connection = "TCP";

	// Ports
	int kmp_commander_port = 30002;
	int kmp_laser_port = 30003; //Daniel
	int kmp_odometry_port = 30004; //Daniel
	int lbr_commander_port = 30005;
	
	
	// IP-address
	String remote_ip = "172.31.1.69";
	//String remote_ip = "192.168.10.79";


	public void initialize() {
		System.out.println("Initializing Robotics API Application");

		// Configure application
		BasicConfigurator.configure();
		resumeFunction = getTaskFunction(IAutomaticResumeFunction.class);

		// Configure robot;
		//controller = getController("KUKA_Sunrise_Cabinet_1");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);
		lbr = getContext().getDeviceFromType(LBR.class);	
		
		// Create nodes for communication
		// kmp_commander = new KMP_commander(kmp);
		kmp_commander = new KmpCommander(remote_ip, kmp_commander_port, kmp, connection);
		lbr_commander = new LbrCommander(remote_ip, lbr_commander_port, lbr, connection, getApplicationData().getFrame("/DrivePos"));

		// Check if a commander node is active
		long startTime = System.currentTimeMillis();
		int shutDownAfterMs = 10000; 
		while(!AppRunning) {
			if(lbr_commander.isSocketConnected()) {
				AppRunning = true;
				System.out.println("Application ready to run!");	
				break;
			} else if((System.currentTimeMillis() - startTime) > shutDownAfterMs) {
				System.out.println("Could not connect to a command node after " + shutDownAfterMs/1000 + "s. Shutting down.");	
				shutdown_application();
				break;
			}				
		}
		// Establish remaining nodes Daniel
		if(AppRunning){
			kmp_sensor_reader = new KMP_sensor_reader(kmp_laser_port, kmp_odometry_port, connection, connection);
		}
	}
	
	public void run() {
		setAutomaticallyResumable(true);

		System.out.println("Running app!");

		kmp_commander.setPriority(Thread.MAX_PRIORITY);
		lbr_commander.setPriority(Thread.MAX_PRIORITY);
		
		if(!(kmp_commander == null)){
			if(kmp_commander.isSocketConnected()) {
				kmp_commander.start();
			}
		}
		
		if(!(lbr_commander == null)){
			if(lbr_commander.isSocketConnected()) {
				lbr_commander.start();
			}
		}
		if(!(kmp_sensor_reader ==null)){ //Daniel
			if(kmp_sensor_reader.isSocketConnected()) {
				kmp_sensor_reader.start();
			}
		}
		
		// Prioritizes odometry and laser scan readings.
		kmp_sensor_reader.setPriority(Thread.MAX_PRIORITY); //Daniel

		while(AppRunning) {    
			AppRunning = (!(kmp_commander.getShutdown() || lbr_commander.getShutdown()));
		}
		System.out.println("Shutdown message received in main application");
		shutdown_application();
	}

	public void shutdown_application(){
		System.out.println("----- Shutting down Application -----");

		kmp_commander.close();
		lbr_commander.close();
		kmp_sensor_reader.close();//Daniel
	
    	System.out.println("Application terminated");
    	    	
    	/*try {
    		dispose();
    	} catch(Exception e) {
    		System.out.println("Application could not be terminated cleanly: " + e);
    	}
    	*/
	}
	
	private void setAutomaticallyResumable(boolean enable)
	{
		if(enable)
		{
			resumeFunction.enableApplicationResuming(getClass().getCanonicalName());
			return;
		}
		resumeFunction.disableApplicationResuming(getClass().getCanonicalName());		
	}

	
	public static void main(String[] args){
		KmpApp app = new KmpApp();
		app.runApplication();
	}
	
}