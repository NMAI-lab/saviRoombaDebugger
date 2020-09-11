/**
 * @author	Chidiebere Onyedinma
 * @author	Patrick Gavigan
 * @date	2 July 2020
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
	
/**
 * High level goals
 */
!deliverMail.		// Highest level task: Deliver mail from sender to receiver
//!goToLocation.	// Go to a destination location (such as a post point)
//!followPath.		// Follow the path (line on the ground) 
//!dock.			// Dock the robot when it is time to recharge

/**
 * deliverMail
 * Go get the mail from the sender, deliver it to the receiver if I have it
 */
 
 // Case where I have a sender location and don't yet have the mail, not 
 // currently at the senderLocation.
+!deliverMail
	: 	(not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		postPoint(OTHER,_) & 
		not (OTHER = SENDER) &
		batteryOK
	<- 	-destination(_);
		+destination(SENDER);
		!goToLocation;
		!deliverMail.
		
// Case where I am at the sender location
// Assume that the fact that I have arrived at the sender location means that 
// I have the mail (this will need to be updated)
+!deliverMail
 	: 	(not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		postPoint(SENDER,_) & 
		batteryOK
	<- 	+haveMail;
		//-senderLocation(_);	// Should we remove the sender location here?
		!deliverMail.
 
// Case where I have the mail and need to deliver it to the receiver
+!deliverMail
 	: 	haveMail &
		receiverLocation(RECEIVER) &
		postPoint(OTHER,_) &
		not (OTHER = RECEIVER) &
		batteryOK
	<- 	-destination(_);
		+destination(RECEIVER);
		!goToLocation;
		!deliverMail.
		
// Case where I have the mail and am at the receiver location
+!deliverMail
 	: 	haveMail &
		receiverLocation(RECEIVER) &
		postPoint(RECEIVER,_) //&
		//batteryOK
	<- 	-haveMail;
		!deliverMail.
		// -receiverLocation(_).	// Should we remove the receiver location here?

// Case where the battery is low
+!deliverMail
	: 	(batteryLow &
		dockStation(DOCK))	
	<-	-destination(_);
		+destination(DOCK);
		!goToLocation;
		!deliverMail.
		
// Catchall (suspect that this should not be needed)
+!deliverMail
	<-	!deliverMail.

/** 
 * goToLocation
 * This is where the path planning stuff happens, deciding how to get to the
 * destination.
 */
+!goToLocation
	:	destinationAhead
	<-	!followPath;
		!goToLocation.

+!goToLocation
	:	atDestination
	<-	drive(stop).
	
+!goToLocation
	:	destinationLeft	// TODO: Update to use unification for left, right, behind?
	<-	turn(left);	// TODO: This (or something similar) needs to be implementd
		!followPath;
		!goToLocation.
		
+!goToLocation
	:	destinationRight	// TODO: Update to use unification for left, right, behind?
	<-	turn(right);		// TODO: This (or something similar) needs to be implementd
		!followPath;
		!goToLocation.
	
+!goToLocation
	:	destinationBehind	// TODO: Update to use unification for left, right, behind?
	<-	turn(left);		// TODO: This (or something similar) needs to be implementd
		!followPath;
		!goToLocation.
/*
+!goToLocation
	:	batteryLow	// Not sure if this is properly handled.
	<-	!dock;
		!goToLocation.

+!goToLocation
	:	batteryOK & docked
	<-	!undock;
		!goToLocation.	// Has this plan been implemented?
*
+!goToLocation
	<-	run(6);
		!goToLocation;
		!followPath.
*/

/** 
 * followPath
 * Follow the line.
 */
 
 // Ideally, these plans could be combined using unification (see the last plan
 // in this set). This would need a modification of the scripts that interpret 
 // drive() action, or the script that generates the line() message (of both)
+!followPath
	:	line(center)
	<-	drive(forward);
		!followPath.

+!followPath
	:	line(lost) | line(across)
	<-	drive(left);
		drive(forward);
		!followPath.

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) &
		((DIRECTION = left) | (DIRECTION = right))
	<-	drive(DIRECTION);
		!followPath.
		
+!followPath
	<-	!followPath.

/**
 * dock
 * Dock the robot at the charging station
 */
+!dock
 	:	not atDockPost & onTrack
	<-	!navigate;
		!dock.

+!dock
	:	atDockPost & moving
	<-	drive(stop);
	   	!dock.

+!dock
	:	atDockPost & not moving
	<-	dock_bot.
	
+!dock.

