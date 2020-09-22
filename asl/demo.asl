/**
 * @author	Patrick Gavigan
 * @date	21 September 2020
 */
 
/**
 * Main beliefs for the robot
 * Dock location is hard coded for now
 */
dockStation(post5).			// The location of the docking station

/**
 * Navigation rules
 * TODO: Replace with navigation module
 */
 
destination(unknown) :-
	not setDestination(_).
 
// Arrived at the destination
destination(arrived) :-
	setDestination(DESTINATION) &
	position(DESTINATION,_).

// Destination is the previously seen post point
destination(behind) :-
	setDestination(DESTINATION) &
	position(_,DESTINATION).

// Rules @ post1, post4, and post5: at the edge of the map, everything is ahead
destination(forward) :-
 	setDestination(DESTINATION) &
	position(CURRENT,_) &
	((CURRENT = post1) | (CURRENT = post4) | (CURRENT = post5)) &
	not (CURRENT = DESTINATION). 

// Rules @ post2, PAST = post1, (not DESTINATION = post1): Everything else is to
// the left of us.
destination(left) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1.
	
// Rules @ post2, PAST = post1, DESTINATION = post1: Everything else is
// ahead of us.
destination(behind) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	DESTINATION = PAST.

// Rules @ post2, not (PAST = post1), not (DESTINATION = post1): Everything else
// is behind of us.
destination(behind) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	not (PAST = post1) &
	not (DESTINATION = PAST).
	
// Rules @ post3, PAST = post4, DESTINATION = post5
destination(forward) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	DESTINATION = post5.

// Rules @ post3, PAST = post5, DESTINATION = post4
destination(forward) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	DESTINATION = post4.

// Rules @ post3, PAST = post5, DESTINATION = post1 or post 2
destination(right) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	((DESTINATION = post1) | (DESTINATION = post2)).
	
// Rules @ post3, PAST = post4, DESTINATION = post1 or post 2
destination(left) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	((DESTINATION = post1) | (DESTINATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post1 or post2
destination(behind) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	((DESTINATION = post1) | (DESTINATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(right) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	DESTINATION = post4.

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(left) :-
	setDestination(DESTINATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	DESTINATION = post5.
	
/**
 * High level goals
 */
!collectAndDeliverMail(post1,post4).	// Highest level task: Deliver mail from sender to receiver
//!collectMail(post1)
//!deliverMail(post4)
//!goTo(post4).		// Go to a destination location (such as a post point)
//!followPath.		// Follow the path (line on the ground) 
//!manageBattery.	// Dock the robot when it is time to recharge

// TODO: Clean this up
+postPoint(A,B)
	:	not position(_,_)
	<-	.broadcast(tell, updatePosition(A,B));
		+position(A,B).
	
+postPoint(A,B)
	:	position(C,D) &
		((not (A = C)) | (not (B = D)))
	<-	.broadcast(tell, updatePositionWithDrop(A,B));
		-position(_,_);
		+position(A,B).
		
//+position(A,B)
//	:	not visited(A)
//	<-	.broadcast(tell, updateVisited(A));
//		+visited(A).


/**
 * manageBattery
 * Go to the dock station to charge the battery
 */
 // Low battery, dock the robot
+!manageBattery
	:	battery(low) & dockStation(DOCK)	
	<-	.broadcast(tell, battery(chargingNeeded));
		!goTo(DOCK);
		.broadcast(tell, battery(atDock));
		station(dock);
		.broadcast(tell, battery(docked));
		+docked
		!manageBattery.
		
// Battery is full, undock the robot
+!manageBattery
	: 	battery(full) & docked
	<-	.broadcast(tell, battery(charged));
		station(undock);
		-docked;
		.broadcast(tell, battery(unDocked));
		!manageBattery.
		
/**
 * Primary mission: collect and deliver mail
 */
+!collectAndDeliverMail(SENDER,RECEIVER)
	<-	.broadcast(tell, mailUpdate(receivedMission,SENDER,RECEIVER));
		!collectMail(SENDER);//	!goTo(SENDER);
		.broadcast(tell, mailUpdate(arrivedAtSender,SENDER,RECEIVER));
		//+haveMail;
		.broadcast(tell, mailUpdate(collected,SENDER,RECEIVER));
		!deliverMail(RECEIVER); //!goTo(RECEIVER);
		.broadcast(tell, mailUpdate(arrivedAtReceiver,RECEIVER));
		//-haveMail;
		.broadcast(tell, mailUpdate(delivered,RECEIVER)).

+!collectMail(LOCATION)
	:	not haveMail
	<-	!goTo(SENDER);
		+haveMail.
		
+!deliverMail(LOCATION)
	:	haveMail
	<-	!goTo(RECEIVER);
		-haveMail.
		
// Case where I have to collect mail and am not at the location.
//+!collectMail(LOCATION)
//	:	(not haveMail) & (not visited(LOCATION))
//	<-	.broadcast(tell, mailUpdate(goingToSenderLocation));
//		!goTo(LOCATION);
//		!collectMail(LOCATION).
		
// Case where I have to collect mail and am at the location.
//+!collectMail(LOCATION)
//	:	(not haveMail) &
//		visited(LOCATION)			// **** TODO: Update navigation predicates ****	
//	<-	.broadcast(tell, mailUpdate(collected));
//		+haveMail;
//		-visited(_).

// I have the mail, not at the receiver location
// +!deliverMail(LOCATION)
// 	: 	haveMail &
//		(not visited(LOCATION))			// **** TODO: Update navigation predicates ****	
//	<- 	.broadcast(tell, mailUpdate(goingToReceiverLocation));
//		!goTo(LOCATION);
//		!deliverMail(LOCATION).
		
// Case where I have the mail and am at the receiver location
// +!deliverMail(LOCATION)
// 	: 	haveMail &
//		visited(LOCATION)			// **** TODO: Update navigation predicates ****	
//	<- 	.broadcast(tell, mailUpdate(delivered));
//		-visited(_);			// **** TODO: Update navigation predicates ****	
//		-haveMail.

/** 
 * !goTo(Location)
 * This is where the path planning stuff happens, deciding how to get to the
 * destination.
 */
+!goTo(LOCATION)
	:	destination(unknown)
	<-	.broadcast(tell, navigationUpdate(unknown));
		+setDestination(DESTINATION);	// **** Remove + for navigation module
		-position(_,_);			// **** TODO: Update navigation predicates ****	
		!goTo(LOCATION).

+!goTo(LOCATION)
	:	destination(arrived)
	<-	.broadcast(tell, navigationUpdate(arrived));
		-position(_,_);			// **** TODO: Update navigation predicates ****	
		-setDestination(_);		// **** TODO: Update navigation module actions ****
		drive(stop).
	
+!goTo(LOCATION)
	:	destination(behind)
	<-	.broadcast(tell, navigationUpdate(behind));
		turn(left);
		-position(_,_);			// **** TODO: Update navigation predicates ****	
		!followPath;
		!goTo(LOCATION).
		
+!goTo(LOCATION)
	:	destination(DIRECTION)	&
		((DIRECTION = forward) | (DIRECTION = left) | (DIRECTION = right))
	<-	.broadcast(tell, navigationUpdate(DIRECTION));
		drive(DIRECTION);
		-position(_,_);			// **** TODO: Update navigation predicates ****
		!followPath;
		!goTo(LOCATION).
		
/** 
 * !followPath
 * Follow the line.
 */
+!followPath
	:	line(center) &
		not (postPoint(_,_) | position(_,_))		// Use plan priority to remove need for this
	<-	.broadcast(tell, path(center));
		drive(forward).
		
+!followPath
	:	line(lost) & 
		not (postPoint(_,_) | position(_,_))		// Use plan priority to remove need for this
	<-	.broadcast(tell, path(lost));
		drive(spiral).

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) & 
		not (postPoint(_,_) | position(_,_))		// Use plan priority to remove need for this
	<-	.broadcast(tell, path(DIRECTION));
		drive(DIRECTION).
		
/**
 * Default plans, in case things go wrong.
 */
+!collectAndDeliverMail(_,_)
	<-	.broadcast(tell, collectAndDeliverMail(default));
		!collectAndDeliverMail(_,_).

+!collectMail(_)
	<-	.broadcast(tell, collectMail(default));
		!collectMail(_).

+!deliverMail(_)
	<-	.broadcast(tell, deliverMail(default));
		!deliverMail(_).

+!goTo(_)
	<-	.broadcast(tell, goTo(default));
		!goTo(_).
		
+!followPath
	<-	.broadcast(tell, followPath(default));
		!goTo(_). 

+!manageBattery
	<-	.broadcast(tell, manageBattery(default));
		!manageBattery.

