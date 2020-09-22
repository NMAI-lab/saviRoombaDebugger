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
	setDestination(LOCATION) &
	position(LOCATION,_).

// Destination is the previously seen post point
destination(behind) :-
	setDestination(LOCATION) &
	position(_,LOCATION).

// Rules @ post1, post4, and post5: at the edge of the map, everything is ahead
destination(forward) :-
 	setDestination(LOCATION) &
	position(CURRENT,_) &
	((CURRENT = post1) | (CURRENT = post4) | (CURRENT = post5)) &
	not (CURRENT = LOCATION). 

// Rules @ post2, PAST = post1, (not DESTINATION = post1): Everything else is to
// the left of us.
destination(left) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1.
	
// Rules @ post2, PAST = post1, DESTINATION = post1: Everything else is
// ahead of us.
destination(behind) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	LOCATION = PAST.

// Rules @ post2, not (PAST = post1), not (DESTINATION = post1): Everything else
// is behind of us.
destination(behind) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post2 &
	not (PAST = post1) &
	not (LOCATION = PAST).
	
// Rules @ post3, PAST = post4, DESTINATION = post5
destination(forward) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	LOCATION = post5.

// Rules @ post3, PAST = post5, DESTINATION = post4
destination(forward) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	LOCATION = post4.

// Rules @ post3, PAST = post5, DESTINATION = post1 or post 2
destination(right) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	((LOCATION = post1) | (LOCATION = post2)).
	
// Rules @ post3, PAST = post4, DESTINATION = post1 or post 2
destination(left) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	((LOCATION = post1) | (LOCATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post1 or post2
destination(behind) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	((LOCATION = post1) | (LOCATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(right) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	LOCATION = post4.

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(left) :-
	setDestination(LOCATION) &
	position(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	LOCATION = post5.
	
/**
 * High level goals
 */
 !collectAndDeliverMail(post1,post4).	// Highest level task: Deliver mail from sender to receiver
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
		!goTo(SENDER);
		.broadcast(tell, mailUpdate(arrivedAtSender,SENDER,RECEIVER));
		+haveMail;
		.broadcast(tell, mailUpdate(collected,SENDER,RECEIVER));
		!goTo(RECEIVER);
		.broadcast(tell, mailUpdate(arrivedAtReceiver,RECEIVER));
		-haveMail;
		.broadcast(tell, mailUpdate(delivered,RECEIVER)).

/** 
 * !goTo(Location)
 * This is where the path planning stuff happens, deciding how to get to the
 * destination.
 */
+!goTo(LOCATION)
	:	destination(unknown)
	<-	.broadcast(tell, navigationUpdate(unknown));
		+setDestination(LOCATION);	// **** Remove + for navigation module
		-position(_,_);			// **** TODO: Update navigation predicates ****	
		!goTo(LOCATION).

+!goTo(LOCATION)
	:	destination(arrived)
	<-	.broadcast(tell, navigationUpdate(arrived));
		-position(_,_);					// **** TODO: Update navigation predicates ****	
		-setDestination(LOCATION);		// **** TODO: Update navigation module actions ****
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
		drive(forward);
		!followPath.
		
+!followPath
	:	line(lost) & 
		not (postPoint(_,_) | position(_,_))		// Use plan priority to remove need for this
	<-	.broadcast(tell, path(lost));
		drive(spiral);
		!followPath.

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) & 
		not (postPoint(_,_) | position(_,_))		// Use plan priority to remove need for this
	<-	.broadcast(tell, path(DIRECTION));
		drive(DIRECTION);
		!followPath.
		
/**
 * Default plans, in case things go wrong.
 */
+!collectAndDeliverMail(SENDER,RECEIVER)
	<-	.broadcast(tell, collectAndDeliverMail(default));
		!collectAndDeliverMail(SENDER,RECEIVER).

+!goTo(LOCATION)
	<-	.broadcast(tell, goTo(default));
		//!followPath;		// If we are stuck, try to go somewhere
		!goTo(LOCATION).
		
+!followPath
	<-	.broadcast(tell, followPath(default)). 

+!manageBattery
	<-	.broadcast(tell, manageBattery(default));
		!manageBattery.

