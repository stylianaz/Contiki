importPackage(java.io);
//TIMEOUT(300000,log.log("last time : " + time + "\n"));  //5mins
//TIMEOUT(360000,log.log("last time : " + time + "\n"));  //6mins
//TIMEOUT(480000,log.log("last time : " + time + "\n")); //8mins
//TIMEOUT(600000,log.log("last time : " + time + "\n")); //10mins
//TIMEOUT(900000,log.log("last time : " + time + "\n")); //15mins
//TIMEOUT(1200000,log.log("last time : " + time + "\n"));   //20mins
//TIMEOUT(1800000,log.log("last time : " + time + "\n"));   //30mins
TIMEOUT(3600000,log.log("last time : " + time + "\n"));   //60mins


var sink = 1;
var idd = 1;
motes = sim.getMotes();
num_messages_tx = 0;
num_messages_rx = 0;
num_packet_losses = 0;
var counter = 1;

timeout_function = function () {
    log.log("Script timed out.\n");
    log.testOK();
}

log.log("number of motes: "+ motes.length+"\n");

while(true){

    if (msg) {	

		if(msg.contains("total distance = ")) {
			var retval = msg.split("distance = ");
			log.log("Distance: "+time + "," + id + "," + retval[1] + "\n");
		}

		//for energy consumptio
		if(msg.contains("Remain energy")) {
			var retval = msg.split("Remain energy ");
			log.log("RemainEnergy: "+time + "," + id + "," + retval[1] + "\n");
		}
		
		//for mobile node energy
		if(msg.contains("total_energy")) {
			var retval = msg.split("total_energy = ");
			log.log("TotalEnergy: "+time + "," + id + "," +  retval[1] + "\n");
	    	}

		if(msg.contains("Send message!") ) {
			num_messages_tx += 1;
			var retval = msg.split("Send message! ");
			log.log("Packet sent: "+ retval[1] + "\n");
		}

		if(msg.startsWith("Packet lost!") ) {
			num_packet_losses += 1;
			num_messages_tx += 1;
			var retval = msg.split("Packet lost! ");
			log.log("Packet lost: "+ retval[1] + "\n");
			log.log("Number Packet lost: " + time  + ","+  num_packet_losses+ "\n");
			//log.log("Packet lost: " + time  + ","+  (num_packet_losses/num_messages_tx)+","+(num_messages_tx)+ "\n");
			//ratioooooo
    		}


		if(msg.contains("Packet Received! ") ) {
			var retval = msg.split("Packet Received! ");
			log.log("Packet Received: "+ retval[1] + "\n");
		}
	
		//for throughput	
		if(msg.startsWith("wsp Multihop message received") ) {
			num_messages_rx += 1;
			var retval = msg.split("Delay: ");
			//log.log("T: "+ time + ":" +(num_messages_tx-num_messages_rx) + "," + num_messages_rx + "," + retval[1] + "\n");
			log.log("Received ratio: "+time+","+(num_messages_rx/(num_messages_tx))+","+(num_messages_tx)+"\n");
			log.log("Delay: "+ retval[1] +","+ time + "\n");
    		}
		/*

		if (num_messages_tx>0 && counter < time/1000000 ){
			log.log("Throughput: "+time+","+(num_messages_rx/(num_messages_tx))+","+(num_messages_tx)+"\n");
			counter++;
		}*/

		if(msg.contains("INIT_NEIGHBORS") && id<=motes.length){
			log.log("id = "+idd+"\n");
			write(sim.getMoteWithID(sink), ""+idd);
			write(sim.getMoteWithID(sink), ""+mote.getSimulation().getMoteWithID(idd).getInterfaces().getPosition().getXCoordinate());
			write(sim.getMoteWithID(sink), ""+mote.getSimulation().getMoteWithID(idd).getInterfaces().getPosition().getYCoordinate());
			idd++;
			YIELD();
		}

		//Faulty Suspicion Neighbor
		if(msg.contains("DEAD")){
			log.log("msg ="+ msg + "\n");
			var retval= msg.split("&");			
			log.log("retval ="+retval[0]+ " & "+retval[1] + "\n");
			mote.getSimulation().getMoteWithID(retval[1]).getInterfaces().getPosition().setCoordinates(300,300,0);
			YIELD();
			log.log("dead!"+ "\n");
		}

		//Faulty Suspicion Neighbor
		if(msg.contains("XYFSN")){
			log.log("msg ="+ msg + "\n");
			var retval= msg.split("&");			
			log.log("retval ="+retval[0]+ " & "+retval[1]+" & "+retval[2] +" & "+retval[3] + retval[4] + retval[5] + retval[6]+"\n");
			mote.getSimulation().getMoteWithID(retval[1]).getInterfaces().getPosition().setCoordinates(retval[2],retval[3],0);
			YIELD();
			log.log("Time : " +  sim.getSimulationTime() + "\n")
			log.log("x=" + mote.getSimulation().getMoteWithID(retval[1]).getInterfaces().getPosition().getXCoordinate() +"\n");
			log.log("y=" + mote.getSimulation().getMoteWithID(retval[1]).getInterfaces().getPosition().getYCoordinate() +"\n");
			log.log("neighbors= "+retval[5]+"\n");
			log.log("Discovery code= "+retval[6]+"\n");
			write(sim.getMoteWithID(retval[1]), ""+mote.getSimulation().getMoteWithID(retval[1]).getInterfaces().getPosition().getXCoordinate());
			write(sim.getMoteWithID(retval[1]), ""+mote.getSimulation().getMoteWithID(retval[1]).getInterfaces().getPosition().getYCoordinate());
			write(sim.getMoteWithID(retval[1]), ""+retval[4]); //hops
			write(sim.getMoteWithID(retval[1]), ""+retval[5]); //neighbors
			write(sim.getMoteWithID(retval[1]), ""+retval[6]); //discovery code
			YIELD();
			log.log("moved!" + "\n");
		}
		
    }
    YIELD();
}
