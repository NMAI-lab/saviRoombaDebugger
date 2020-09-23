/**
 * @author	Patrick Gavigan
 * @date	22 September 2020
 */
 
/**
 * Navigation rules
 * TODO: Replace with navigation module
 */
 
destination(unknown) :-
	not setDestination(_).
 
// Arrived at the destination
destination(arrived) :-
	setDestination(LOCATION) &
	postPoint(LOCATION,_).

// Destination is the previously seen post point
destination(behind) :-
	setDestination(LOCATION) &
	postPoint(_,LOCATION).

// Rules @ post1, post4, and post5: at the edge of the map, everything is ahead
destination(forward) :-
 	setDestination(LOCATION) &
	postPoint(CURRENT,_) &
	((CURRENT = post1) | (CURRENT = post4) | (CURRENT = post5)) &
	not (CURRENT = LOCATION). 

// Rules @ post2, PAST = post1, (not DESTINATION = post1): Everything else is to
// the left of us.
destination(left) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1.
	
// Rules @ post2, PAST = post1, DESTINATION = post1: Everything else is
// ahead of us.
destination(behind) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	LOCATION = PAST.

// Rules @ post2, not (PAST = post1), not (DESTINATION = post1): Everything else
// is behind of us.
destination(behind) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	not (PAST = post1) &
	not (LOCATION = PAST).
	
// Rules @ post3, PAST = post4, DESTINATION = post5
destination(forward) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	LOCATION = post5.

// Rules @ post3, PAST = post5, DESTINATION = post4
destination(forward) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	LOCATION = post4.

// Rules @ post3, PAST = post5, DESTINATION = post1 or post 2
destination(right) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	((LOCATION = post1) | (LOCATION = post2)).
	
// Rules @ post3, PAST = post4, DESTINATION = post1 or post 2
destination(left) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	((LOCATION = post1) | (LOCATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post1 or post2
destination(behind) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	((LOCATION = post1) | (LOCATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(right) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	LOCATION = post4.

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(left) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	LOCATION = post5.
	
/**
 * High level goals
 */
//!collectAndDeliverMail(post1,post4).	// Highest level task: Deliver mail from sender to receiver
//!goTo(post4).		// Go to a destination location (such as a post point)
//!followPath.		// Follow the path (line on the ground) 
!manageBattery.		// Dock the robot when it is time to recharge

/**
 * manageBattery
 * Go to the dock station to charge the battery
 */
 // Low battery, Go to the docking station, charge the robot
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
 * Plan is simple: 
 * (1) Go to the sender location
 * (2) Collect the mail
 * (3) Go to the receiver location
 * (4) Deliver the mail
 * Updating the user at every step.
 *
 * NOTE: The robot must start with a post point visible!
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
 * Used for navigating the robot via the post points. Decide if the robot needs 
 * to turn or drive forward. Uses the sub goal of !followPath to move between 
 * post points.
 * 
 * Note: The robot needs to have a post point visible to start things off or 
 * this won't work properly.
 */
 
 // Case where the robot has not yet set a destination to navigate to. Need to 
 // set the destination.
+!goTo(LOCATION)
	:	destination(unknown)
	<-	.broadcast(tell, navigationUpdate(setDestination,LOCATION));
		+setDestination(LOCATION);	// **** Remove + for navigation module
		!goTo(LOCATION).

// Case where the robot has arrived at the destination.
+!goTo(LOCATION)
	:	destination(arrived)
	<-	.broadcast(tell, navigationUpdate(arrived));
		-setDestination(LOCATION);		// **** TODO: Update navigation module actions ****
		drive(stop).
	
// Destination is behind us: turn and start following the path.
+!goTo(LOCATION)
	:	destination(behind)
	<-	.broadcast(tell, navigationUpdate(behind));
		turn(left);
		!followPath;
		!goTo(LOCATION).
		
// Destiantion is forward. Drive forward, follow the path.
+!goTo(LOCATION)
	:	destination(forward)
	<-	.broadcast(tell, navigationUpdate(forward));
		drive(forward);
		!followPath;
		!goTo(LOCATION).

// Destiantion is either left or right. Turn and then follow the path.
+!goTo(LOCATION)
	:	destination(DIRECTION)	&
		((DIRECTION = left) | (DIRECTION = right))
	<-	.broadcast(tell, navigationUpdate(DIRECTION));
		turn(DIRECTION);
		!followPath;
		!goTo(LOCATION).

/** 
 * !followPath
 * Follow the line until a post point is visible, then stop.
 * Search for the line if it is not visible, adjust course if the line is 
 * drifting to the left or right.
 */
 
 // Case where the postPoint is visible, no driving.
 +!followPath
 	:	postPoint(A,B)
	<-	.broadcast(tell, followPath(PostPointStop,A,B));
		drive(stop).
 
 
// Line is center, no post point, drive forward
+!followPath
	:	line(center) &
		(not postPoint(_,_))
	<-	.broadcast(tell, followPath(center));
		drive(forward);
		!followPath.
		
// Line is lost, use the spiral action to try and find it.
+!followPath
	:	line(lost) & 
		(not postPoint(_,_))
	<-	.broadcast(tell, followPath(lost));
		drive(spiral);
		!followPath.

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) & 
		((DIRECTION = left) | (DIURECTION = right)) &
		(not postPoint(_,_))
	<-	.broadcast(tell, followPath(DIRECTION));
		drive(DIRECTION);
		!followPath.

/**
 * Default plans.
 * These can run when unrelated perceptions are received, resulting in a 
 * reasoning cycle where no plan context is applicable for that reasoning cycle.
 */
 // Ensure recursion and remember the parameters.
 // Note: This plan should never run as there are no context guards in the 
 // main plan for this goal above. If this plan runs, something strange 
 // happened.
+!collectAndDeliverMail(SENDER,RECEIVER)
	<-	.broadcast(tell, collectAndDeliverMail(default));
		!collectAndDeliverMail(SENDER,RECEIVER).

// Deal with the scenario where the reasoning cycle runs on a perception other 
// than a post point.
+!goTo(LOCATION)
	<-	.broadcast(tell, goTo(default));
		!goTo(LOCATION).
		
// Ensure recursion. For example, if only a battery perception is received, you
// don't want to lose the followPath intention.
+!followPath
	<-	.broadcast(tell, followPath(default));
		!followPath. 

// Ensure recursion. We always need to manage the battery
+!manageBattery
	<-	//.broadcast(tell, manageBattery(default));
		!manageBattery.

