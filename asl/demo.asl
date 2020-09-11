/**
 * Demo BDI program
 * This example BDI program implements the goal 
 * 'demonstrate'. This goal has to be passed as 
 * an 'achieve' message. Once it has that goal,
 * it performs the action 'do(action)' when it 
 * receives a perception of the format 
 * 'time(1234)'. It also boradcasts the message
 * heardTime(1234).
 * @author	Patrick Gavigan
 * @date	6 December 2019
 */

+!demonstrate
	:	time(A)
	<-	do(something);
		.broadcast(tell, heardTime(A)).
		
+!demonstrate
	<-	!demonstrate.
