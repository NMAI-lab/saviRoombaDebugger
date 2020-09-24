/**
 * @author	Patrick Gavigan
 * @date	23 September 2020
 */
 
/**
 * Navigation rules
 * TODO: Replace with navigation module
 */

// Destination has not yet been set
destination(LOCATION,unknown,unknown) :-
	not setDestination(A).
	
// Destination was set previously but it isn't where we want to go right now.
destination(LOCATION,old,unknown) :-
	setDestination(OTHER) &
	not (OTHER = LOCATION).
 
// Arrived at the destination
destination(LOCATION,LOCATION,arrived) :-
	setDestination(LOCATION) &
	postPoint(LOCATION,_).

// Destination is the previously seen post point
destination(LOCATION,LOCATION,behind) :-
	setDestination(LOCATION) &
	postPoint(_,LOCATION).

// Rules @ post1, post4, and post5: at the edge of the map, everything is ahead
destination(LOCATION,LOCATION,forward) :-
 	setDestination(LOCATION) &
	postPoint(CURRENT,_) &
	((CURRENT = post1) | (CURRENT = post4) | (CURRENT = post5)) &
	not (CURRENT = LOCATION). 

// Rules @ post2, PAST = post1, (not DESTINATION = post1): Everything else is to
// the left of us.
destination(LOCATION,LOCATION,left) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1.
	
// Rules @ post2, PAST = post1, DESTINATION = post1: Everything else is
// ahead of us.
destination(LOCATION,LOCATION,behind) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	PAST = post1 &
	LOCATION = PAST.

// Rules @ post2, not (PAST = post1), not (DESTINATION = post1): Everything else
// is behind of us.
destination(LOCATION,LOCATION,behind) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post2 &
	not (PAST = post1) &
	not (LOCATION = PAST).
	
// Rules @ post3, PAST = post4, DESTINATION = post5
destination(LOCATION,LOCATION,forward) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	LOCATION = post5.

// Rules @ post3, PAST = post5, DESTINATION = post4
destination(LOCATION,LOCATION,forward) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	LOCATION = post4.

// Rules @ post3, PAST = post5, DESTINATION = post1 or post 2
destination(LOCATION,LOCATION,right) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post5 &
	((LOCATION = post1) | (LOCATION = post2)).
	
// Rules @ post3, PAST = post4, DESTINATION = post1 or post 2
destination(LOCATION,LOCATION,left) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post4 &
	((LOCATION = post1) | (LOCATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post1 or post2
destination(LOCATION,LOCATION,behind) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	((LOCATION = post1) | (LOCATION = post2)).

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(LOCATION,LOCATION,right) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	LOCATION = post4.

// Rules @ post3, PAST = post2, DESTINATION = post4
destination(LOCATION,LOCATION,left) :-
	setDestination(LOCATION) &
	postPoint(CURRENT,PAST) &
	CURRENT = post3 &
	PAST = post2 &
	LOCATION = post5.
	
/**
 * +battery(low)
 * When the battery(low) perception is received, we need to pickup the goal
 * of !chargeBattery, if we haven't already.
 */
// Check if I'm on a mailMission, if so, I'll need to launch the mission again
// once the battery is charged.
// Also need to confirm that I'm not already charging the battery. 
+battery(low)
	:	(not charging) & 
		dockStation(DOCK) &
		mailMission(SENDER,RECEIVER)
	<-	+charging;
		.drop_all_intentions;
		.broadcast(tell, battery(chargingNeeded));
		.broadcast(tell, battery(intentionsDropped));
		!chargeBattery;
		-charging;
		.broadcast(tell, battery(chargingFinished));
		.broadcast(tell, battery(returnToMission));
		!collectAndDeliverMail(SENDER,RECEIVER).

// No mailMission on the go, just need to charge the battery if I'm not already
// dealing with it.
+battery(low)
	:	(not charging) &
		dockStation(DOCK) &
		not mailMission(SENDER,RECEIVER)
	<-	+charging;
		.broadcast(tell, battery(chargingNeeded));
		.broadcast(tell, battery(noMissionToInterrupt));
		!chargeBattery;
		-charging;
		.broadcast(tell, battery(chargingFinished)).

+battery(low)
	:	not dockStation(DOCK)
	<-	.broadcast(tell, battery(chargingNeeded));
		.broadcast(tell, battery(noDockFound)).
		
/**
 * !chargeBattery
 * The plan for getting the battery to battery(full) if needed.
 */
 // Battery isn't full and I'm not docked. Go to the docking station and dock.
 // This is rerursiuve as we need to wait for the battery to charge.
 +!chargeBattery
	:	(not battery(full)) &
		dockStation(DOCK) &
		(not docked)
	<-	.broadcast(tell, battery(chargingNeeded));
		!goTo(DOCK);
		.broadcast(tell, battery(atDock));
		station(dock);
		.broadcast(tell, battery(docked));
		+docked;
		!chargeBattery.
		
// Battery is full, undock the robot
+!chargeBattery
	: 	battery(full) & docked
	<-	.broadcast(tell, battery(charged));
		station(undock);
		-docked;
		.broadcast(tell, battery(unDocked)).
		
/**
 * Primary mission: collect and deliver mail
 * Plan is simple: 
 * (1) Make mental note that I have a mail mission
 * (2) Collect the mail
 * (3) Deliver the mail
 * (4) Delete mental note about mail mission
 * Updating the user at every step.
 *
 * NOTE: The robot must start with a post point visible!
 */
+!collectAndDeliverMail(SENDER,RECEIVER)
	:	(not charging)
	<-	+mailMission(SENDER,RECEIVER)
		.broadcast(tell, mailUpdate(receivedMission,SENDER,RECEIVER));
		!collectMail(SENDER);
		.broadcast(tell, mailUpdate(gotMail,SENDER,RECEIVER));
		!deliverMail(RECEIVER)
		.broadcast(tell, mailUpdate(delivered,RECEIVER));
		-mailMission(SENDER,RECEIVER).

/**
 * !collectMail(SENDER)
 * Go get the mail from the sender
 */
 // I do not yet have the mail, go get it.
+!collectMail(SENDER)
	:	not haveMail
	<-	.broadcast(tell, collectMail(goToSender,SENDER));
		!goTo(SENDER);
		.broadcast(tell, collectMail(atSender,SENDER));
		+haveMail;
		.broadcast(tell, collectMail(haveMail,SENDER)).

// I already have the mail.
+!collectMail(SENDER)
	:	haveMail
	<-	.broadcast(tell, collectMail(alreadyHaveMail,SENDER)).

/**
 * !deliverMail(RECEIVER)
 * Give the mail to the receiver
 */
 // Case where I have the mail.
+!deliverMail(RECEIVER)
	:	haveMail
	<-	.broadcast(tell, deliverMail(goToReceiver,RECEIVER));
		!goTo(RECEIVER);
		.broadcast(tell, deliverMail(arrivedAtReceiver,RECEIVER));
		-haveMail;
		.broadcast(tell, deliverMail(delivered,RECEIVER)).
		
// Case where there is no mail to deliver.
+!deliverMail(RECEIVER)
	:	not haveMail
	<-	.broadcast(tell, deliverMail(noMail,RECEIVER)).

/** 
 * !goTo(LOCATION)
 * Used for navigating the robot via the post points. Decide if the robot needs 
 * to turn or drive forward. Uses the sub goal of !followPath to move between 
 * post points.
 * 
 * LOCATION: The location we want to go to
 * SET_DESTINATION: The location that the agent has set itself to navigate to
 * DIRECTION: The direction that the agent needs to go in order to move toward 
 *				SED_DESTINATION
 *
 * Note: The robot needs to have a post point visible to start things off or 
 * this won't work properly.
 */

 // Case where the robot has not yet set a destination to navigate to. Need to 
 // set the destination.
+!goTo(LOCATION)
	:	destination(_,unknown,_)
	<-	.broadcast(tell, navigationUpdate(setDestination,LOCATION));
		+setDestination(LOCATION);	// **** Remove + for navigation module
		!goTo(LOCATION).

 // The robot has a different destination than the one we need to go to.
 +!goTo(LOCATION)
	:	destination(LOCATION,old,_)
	<-	.broadcast(tell, navigationUpdate(updateDestination,LOCATION));
		-setDestination(_);
		+setDestination(LOCATION);	// **** Remove + for navigation module
		!goTo(LOCATION).
		
// Case where the robot has arrived at the destination.
+!goTo(LOCATION)
	:	destination(LOCATION,LOCATION,arrived)
	<-	.broadcast(tell, navigationUpdate(arrived));
		-setDestination(LOCATION);		// **** TODO: Update navigation module actions ****
		drive(stop).
	
// Destination is behind us: turn and start following the path.
+!goTo(LOCATION)
	:	destination(LOCATION,LOCATION,behind)
	<-	.broadcast(tell, navigationUpdate(behind));
		turn(left);
		!followPath;
		!goTo(LOCATION).
		
// Destiantion is forward. Drive forward, follow the path.
+!goTo(LOCATION)
	:	destination(LOCATION,LOCATION,forward)
	<-	.broadcast(tell, navigationUpdate(forward));
		drive(forward);
		!followPath;
		!goTo(LOCATION).

// Destiantion is either left or right. Turn and then follow the path.
+!goTo(LOCATION)
	:	destination(LOCATION,LOCATION,DIRECTION) &
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

// Ensure recursion while we are waiting for the battery to charge.
+!chargeBattery
	<-	.broadcast(tell, manageBattery(waitingToCharge));
		!chargeBattery.

// Default plan for collect mail. Should not be possible to get here.
+!collectMail(SENDER)
	<-	.broadcast(tell, collectMail(error,SENDER)).
		
// Default plan for deliver mail. Should not be possible to get here.
+!deliverMail(RECEIVER)
	<-	.broadcast(tell, deliverMail(error,RECEIVER)).
