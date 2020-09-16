/**
 * @author	Chidiebere Onyedinma
 * @author	Patrick Gavigan
 * @date	3 August 2020
 */
 
/**
 * Main beliefs for the robot
 * Mission is hard coded for now (need to update this)
 */
senderLocation(post1).		// The location of the mail sender
receiverLocation(post4).	// The location of the mail receiver
dockStation(post5).			// The location of the docking station

/**
 * Navigation rules
 */

// Arrived at the destination
atDestination :-
	destination(DESTINATION) &
	postPoint(DESTINATION,_).

// Destination is the previously seen post point
destinationBehind :-
	destinaton(DESTINATION) &
	postPoint(_,DESTINATION).

// Rules @ post1, post4, and post5: at the edge of the map, everything is ahead
destinationAhead :-
 	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	((CURRENT = post1) | (CURRENT = post4) | (CURRENT = post5)) &
	not (PAST = CURRENT). 
// Do we need to deal with the case where we were trying to drive off the end of
// the map? Likely yes, not certain.
	
// Rules @ post2, PAST = post1, (not DESTINATION = post1): Everything else is to
// the left of us.
destinationLeft :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	not (DESTINATION = PAST).
	
// Rules @ post2, PAST = post1, DESTINATION = post1: Everything else is
// ahead of us.
destinationBehind :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	DESTINATION = PAST.
	
// Rules @ post2, not (PAST = post1), not (DESTINATION = post1): Everything else
// is behond of us.
destinationBehind :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	not (PAST = post1) &
	not (DESTINATION = PAST).
	
// Rules @ post3, PAST = post4, DESTINATION = post5
destinationAhead :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	DESTINATION = post5.

// Rules @ post3, PAST = post5, DESTINATION = post4
destinationAhead :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	DESTINATION = post4.

// Rules @ post3, PAST = post5, DESTINATION = post1 or post 2
destinationRight :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	((DESTINATION = post1) | (DESTINATION = post2)).
	
// Rules @ post3, PAST = post4, DESTINATION = post1 or post 2
destinationLeft :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	((DESTINATION = post1) | (DESTINATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post1 or post2
destinationBehind :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	((DESTINATION = post1) | (DESTINATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post4
destinationRight :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	DESTINATION = post4.

// Rules @ post3, PAST = post2, DESTINATION = post4
destinationLeft :-
	destination(DESTINATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	DESTINATION = post5.
	
/*
NAVIGATION WITH NEW MODULE

// Arrived at the destination
atDestination :-
	(destination(DESTINATION) & postPoint(DESTINATION,_)) | direction(arrived).

// Destination is the previously seen post point
destinationBehind :-
	(destinaton(DESTINATION) & postPoint(_,DESTINATION)) | direction(behind).

destinationAhead :- 
	direction(forward).
	
destinationRight :-
	direction(right).
	
destinationLeft :-
	direction(left).
*/

/**
 * High level goals
 */
!deliverMail.		// Highest level task: Deliver mail from sender to receiver
//!goToLocation.	// Go to a destination location (such as a post point)
//!followPath.		// Follow the path (line on the ground) 
//!manageBattery.	// Dock the robot when it is time to recharge

/**
 * deliverMail
 * Go get the mail from the sender, deliver it to the receiver if I have it
 */
 
 // Case where I have a sender location and don't yet have the mail, not 
 // currently at the senderLocation.
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		(not postPoint(SENDER,_)))// &
		//battery(ok))
	<- 	do(1);
		setDestination(SENDER);
		!goToLocation;
		!deliverMail.
		
// Case where I am at the sender location
// Assume that the fact that I have arrived at the sender location means that 
// I have the mail (this will need to be updated)
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		postPoint(SENDER,_))// & 
		//battery(ok))
	<- 	do(2);
		+haveMail;
		.broadcast(tell, mailUpdate(collected));
		!deliverMail.
 
// Case where I have the mail and need to deliver it to the receiver
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		not postPoint(RECEIVER,_))// &
		//battery(ok))
	<- 	do(3);
		!setDestination(RECEIVER);
		!goToLocation;
		!deliverMail.
		
// Case where I have the mail and am at the receiver location
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		postPoint(RECEIVER,_))// &
		//battery(ok))
	<- 	do(4);
		-haveMail;
		.broadcast(tell, mailUpdate(delivered));
		!deliverMail.

// Case where the battery is low
// +!deliverMail
// 	: 	(battery(low) &
//		dockStation(DOCK))	
//	<-	do(5);
//		!manageBattery;
		//-destination(_);
		//+destination(DOCK);
		//!dock;
//		!deliverMail.


/** 
 * goToLocation
 * This is where the path planning stuff happens, deciding how to get to the
 * destination.
 */
+!goToLocation
	:	destinationAhead
	<-	do(6);
		drive(forward);
		!followPath;
		!goToLocation.

+!goToLocation
	:	atDestination
	<-	do(7);
		drive(stop).
	
+!goToLocation
	:	destinationLeft	// TODO: Update to use unification for left, right, behind?
	<-	do(8);
		turn(left);	// TODO: This (or something similar) needs to be implementd
		!followPath;
		!goToLocation.
		
+!goToLocation
	:	destinationRight	// TODO: Update to use unification for left, right, behind?
	<-	do(9);
		turn(right);		// TODO: This (or something similar) needs to be implementd
		!followPath;
		!goToLocation.
	
+!goToLocation
	:	destinationBehind	// TODO: Update to use unification for left, right, behind?
	<-	do(10);
		turn(left);		// TODO: This (or something similar) needs to be implementd
		!followPath;
		!goToLocation.

/** 
 * followPath
 * Follow the line.
 */
 
 // Ideally, these plans could be combined using unification (see the last plan
 // in this set). This would need a modification of the scripts that interpret 
 // drive() action, or the script that generates the line() message (of both)
+!followPath
	:	line(center)// &
		//not postPoint(_,_)
	<-	do(12);
		drive(forward).
		
+!followPath
	:	line(lost)// & 
		//not postPoint(_,_)
	<-	do(13);
		drive(spiral).

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) //& 
		//not postPoint(_,_)
	<-	do(14);
		drive(DIRECTION).
		
		
/**
 * manageBattery
 * Go to the dock station to charge the battery
 */
 // Low battery, not at the dock station, need to set the destination to the 
 // dock station and go there
+!manageBattery
	:	battery(low) & dockStation(DOCK) & dest(DEST) & (not (DOCK = DEST) & 
		not postPoint(DOCK,_))
	<-	do(16);
		!setDestination(DOCK); 
		!goToLocation;
		!manageBattery.
		
// We are at the station, dock to charge the battery
+!manageBattery
	: 	battery(low) & dockStation(DOCK) & postPoint(DOCK,_)
	<-	do(17);
		drive(stop);
		station(dock);
		.broadcast(tell, battery(charging));
		!manageBattery.
		
// We are at the station, charging is done, undock
+!manageBattery
	: 	battery(full)
	<-	do(18);
		.broadcast(tell, battery(charged));
		station(undock);
		!manageBattery.

		
/**
 * Set the destination of the robot
 */
+!setDestination(DESTINATION)
	<-	do(20);
		-destination(_);
		+destination(DESTINATION);
		setDestination(DESTINATION).	// Used with new navigation module only
		
		
/**
 * Default plans, in case things go wrong.
 */
+!deliverMail
	<-	do(5);
		!deliverMail.
		
+!goToLocation
	<-	do(11);
		!followPath;
		!goToLocation.
		
// Default follow path. try again if it didn't work
+!followPath
	<- 	do(15).
	
+!manageBattery
	<-	do(19);
		!manageBattery.
