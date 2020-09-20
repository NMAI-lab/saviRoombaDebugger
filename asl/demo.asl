/**
 * @author	Chidiebere Onyedinma
 * @author	Patrick Gavigan
 * @date	3 August 2020
 */
 
/**
 * Main beliefs for the robot
 * Mission is hard coded for now (need to update this)
 */
//mailMission(post1,post4).		// locations of the sender and receiver, 
								// format: mailMission(SENDER,RECEIVER)
//senderLocation(post1).		// The location of the mail sender
//receiverLocation(post4).	// The location of the mail receiver
dockStation(post5).			// The location of the docking station

/**
 * Navigation rules
 */

// Arrived at the destination
atDestination :-
	destination(DESTINATION) &
	position(DESTINATION,_).

// Destination is the previously seen post point
destinationBehind :-
	destinaton(DESTINATION) &
	position(_,DESTINATION).

// Rules @ post1, post4, and post5: at the edge of the map, everything is ahead
destinationAhead :-
 	destination(DESTINATION) &
	position(CURRENT,_) &
	((CURRENT = post1) | (CURRENT = post4) | (CURRENT = post5)) &
	not (CURRENT = DESTINATION). 
// Do we need to deal with the case where we were trying to drive off the end of
// the map? Likely yes, not certain.
	
// Rules @ post2, PAST = post1, (not DESTINATION = post1): Everything else is to
// the left of us.
destinationLeft :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1.
	
// Rules @ post2, PAST = post1, DESTINATION = post1: Everything else is
// ahead of us.
destinationBehind :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	DESTINATION = PAST.
	
// Rules @ post2, not (PAST = post1), not (DESTINATION = post1): Everything else
// is behond of us.
destinationBehind :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	not (PAST = post1) &
	not (DESTINATION = PAST).
	
// Rules @ post3, PAST = post4, DESTINATION = post5
destinationAhead :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	DESTINATION = post5.

// Rules @ post3, PAST = post5, DESTINATION = post4
destinationAhead :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	DESTINATION = post4.

// Rules @ post3, PAST = post5, DESTINATION = post1 or post 2
destinationRight :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	((DESTINATION = post1) | (DESTINATION = post2)).
	
// Rules @ post3, PAST = post4, DESTINATION = post1 or post 2
destinationLeft :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	((DESTINATION = post1) | (DESTINATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post1 or post2
destinationBehind :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	((DESTINATION = post1) | (DESTINATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post4
destinationRight :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	DESTINATION = post4.

// Rules @ post3, PAST = post2, DESTINATION = post4
destinationLeft :-
	destination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	DESTINATION = post5.
	
/*
NAVIGATION WITH NEW MODULE

// Arrived at the destination
atDestination :-
	(destination(DESTINATION) & position(DESTINATION,_)) | direction(arrived).

// Destination is the previously seen post point
destinationBehind :-
	(destinaton(DESTINATION) & position(_,DESTINATION)) | direction(behind).

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

+postPoint(A,B)
	:	not position(_,_)
	<-	+position(A,B);
		setPost(A,B).
	
+postPoint(A,B)
	:	position(C,D) &
		((not (A = C)) | (not (B = D)))
	<-	-position(_,_);
		+position(A,B);
		setPostWithDrop(A,B).

/**
 * Set the destination of the robot
 */
+!setDestination(DESTINATION)
	<-	do(20);
		-destination(_);
		+destination(DESTINATION);
		setDestination(DESTINATION).	// Used with new navigation module only
		
/**
 * manageBattery
 * Go to the dock station to charge the battery
 */
 // Low battery, not at the dock station, need to set the destination to the 
 // dock station and go there
+!manageBattery
	:	battery(low) & dockStation(DOCK) & 
		((dest(DEST) & (not (DOCK = DEST)) | not dest(_)) & 
		not position(DOCK,_))
	<-	do(16);
		!setDestination(DOCK); 
		!goToLocation;
		!manageBattery.
		
// We are at the station, dock to charge the battery
+!manageBattery
	: 	battery(low) & dockStation(DOCK) & position(DOCK,_) & (not docked)
	<-	do(17);
		drive(stop);
		station(dock);
		+docked
		.broadcast(tell, battery(charging));
		!manageBattery.
		
// We are at the station, charging is done, undock
+!manageBattery
	: 	battery(full) & docked
	<-	do(18);
		.broadcast(tell, battery(charged));
		station(undock);
		-docked
		!manageBattery.

/**
 * deliverMail
 * Go get the mail from the sender, deliver it to the receiver if I have it
 */
 
// Case where the mission is being specified
+!deliverMail
	:	mailMission(SENDER,RECEIVER) &
		not (senderLocation(_) | receiverLocation(_)) 
	<-	do(missionSet);
		+senderLocation(SENDER);
		+receiverLocation(RECEIVER);
		-mailMission(_,_);
		!deliverMail.
 
 // Case where I have a sender location and don't yet have the mail, not 
 // currently at the senderLocation.
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		(not position(SENDER,_)))	// There's a bug here -> position is dropped as part of navigation
									// Perhaps the temp fix is to have visited predicates of places I've been?
	<- 	do(1);
		!setDestination(SENDER);
		!goToLocation;
		!deliverMail.
		
// Case where I am at the sender location
// Assume that the fact that I have arrived at the sender location means that 
// I have the mail (this will need to be updated)
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		position(SENDER,_))
	<- 	do(2);
		+haveMail;
		-senderLocation(_);
		.broadcast(tell, mailUpdate(collected));
		!deliverMail.
 
// Case where I have the mail and need to deliver it to the receiver
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		not position(RECEIVER,_))
	<- 	do(3);
		!setDestination(RECEIVER);
		!goToLocation;
		!deliverMail.
		
// Case where I have the mail and am at the receiver location
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		position(RECEIVER,_))
	<- 	do(4);
		-receiverLocation(_);
		-haveMail;
		.broadcast(tell, mailUpdate(delivered)).//;
		//!deliverMail.

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
		-position(_,_);
		!followPath;
		!goToLocation.

+!goToLocation
	:	atDestination
	<-	do(7);
		-position(_,_);
		drive(stop).
	
+!goToLocation
	:	destinationLeft	// TODO: Update to use unification for left, right, behind?
	<-	do(8);
		turn(left);
		-position(_,_);
		!followPath;
		!goToLocation.
		
+!goToLocation
	:	destinationRight	// TODO: Update to use unification for left, right, behind?
	<-	do(9);
		turn(right);		// TODO: This (or something similar) needs to be implementd
		-position(_,_);
		!followPath;
		!goToLocation.
	
+!goToLocation
	:	destinationBehind	// TODO: Update to use unification for left, right, behind?
	<-	do(10);
		turn(left);		// TODO: This (or something similar) needs to be implementd
		-position(_,_);
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
	:	line(center) &
		not (postPoint(_,_) | position(_,_))
	<-	do(12);
		drive(forward).
		
+!followPath
	:	line(lost) & 
		not (postPoint(_,_) | position(_,_))
	<-	do(13);
		drive(spiral).

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) & 
		not (postPoint(_,_) | position(_,_))
	<-	do(14);
		drive(DIRECTION).
		
		
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
