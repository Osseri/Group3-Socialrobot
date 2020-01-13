
:- module(predicate,
    [
    objectSize/2,
    handSize/2,
    currentObjectPerception/2,
    currentObjectVertex/2,
    counter/2,
    set/2,
    currentJointPerception/2,
	semanticMapPerception/2,
	currentGraspDetection/2,
    currentObjectPose/2,
    currentObjectPosition/2,
    currentHandPose/2,
    currentRobotBodyPose/2,
    currentJointAngle/2,
    tilted_Object/1,
    standing_Object/1,
    intersects/3,
    empty_hand/1,
    overlap_hand/2,
    closed_hand/1,
    opened_hand/1,
    graspedBy/2,
	close_empty_hand/1,
	close_overlap_hand/1,
    currentJointAngle/2,
    currentJointEffort/2,
    currentJointVelocity/2,
    currentHandPerception/2,
	currentRobotBodyPerception/2,
    type/2,
    detected_object/1,
	latest_detection_of_robot/2,
	latest_detection_of_object/2,
	currentHandVertex/2,
	semanticPose/2,
	batteryState/1,
	semanticObjectPose/2,
    joint_ex/4,
    joint_check/2,
    empty_hand_test/3,
    closed_hand_test/2,
    detected_object_test/2,
	currentGraspPose/2,
	placeSize/2,
	placePose/2,
	objectsName/2,
	objectPlaceName/2,
	robotPlaceName/2,
    currentBboxSizse/2,
	empty_slot/2,
	worldPosition/2
    ]).

batteryState(State):-
	nb_getval(predicate__batteryState_low_robot_battery_threshold, Low_robot_battery_threshold),
  nb_getval(predicate__batteryState_high_robot_battery_threshold, High_robot_battery_threshold),

  get_timepoint(T),
	rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
  	latest_battery_detection_of_instance(Robot,VPR),
	rdf(VPR, 'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,Rb))),atom_to_term(Rb,RB,_),
	(RB=<Low_robot_battery_threshold,State ='Low'),!;
	(RB>Low_robot_battery_threshold,RB=<High_robot_battery_threshold,State='Middle'),!;
	(RB>High_robot_battery_threshold,State='High').

currentObjectPerception(Object, CurrentPerception):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),	
   latest_detection_of_object(Object, CurrentPerception).

currentHandPerception(Hand, CurrentPerception):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   latest_detection_of_hand(Hand, CurrentPerception).

currentRobotBodyPerception(Robot, CurrentPerception):-
   rdfs_individual_of(Robot,  knowrob:'Robot'),
   latest_detection_of_robot(Robot, CurrentPerception).
   
currentJointPerception(Joint, CurrentPerception) :-
   rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'),
   latest_detection_of_joint(Joint, CurrentPerception).

currentTouchPerception(TouchSensor, CurrentPerception) :-
	rdfs_individual_of(TouchSensor, 'http://knowrob.org/kb/srdl2-comp.owl#PressureSensor'),
	latest_detection_of_touch(TouchSensor, CurrentPerception).

currentGraspDetection(Object, CurrentDetection) :-
	rdfs_individual_of(Object,  knowrob:'Artifact'),
	latest_detection_of_grasp(Object, CurrentDetection).
	
semanticMapPerception(Object, Perception):-
   %rdfs_individual_of(Object, knowrob:'Artifact'),
   rdf(Perception, rdf:type, knowrob:'SemanticMapPerception'),
   rdf_has(Perception, knowrob:objectActedOn, Object).
	
objectSize(Object, Size):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
   rdf(Object, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
   rdf(Object, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
   rdf(Object, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
   Size = [S1X, S1Y, S1Z].
   
currentGraspPose(Object, Pose):-
   currentGraspDetection(Object, CurrentDetection),
   rdf(CurrentDetection, knowrob:eventOccursAt, Grasp_C_Matrix),

   rdf(CurrentDetection, knowrob:approachGraspPoint0, Grasp_Point0),
   rdf(CurrentDetection, knowrob:approachGraspPoint1, Grasp_Point1),
   rdf(CurrentDetection, knowrob:approachGraspPoint2, Grasp_Point2),
   rdf(CurrentDetection, knowrob:approachGraspPoint3, Grasp_Point3),
   rdf(CurrentDetection, knowrob:approachGraspPoint4, Grasp_Point4),
   rdf(CurrentDetection, knowrob:approachGraspPoint5, Grasp_Point5),
   Grasp_Point_list = [Grasp_Point0,Grasp_Point1,Grasp_Point2,Grasp_Point3,Grasp_Point4,Grasp_Point5],
   foreach(member(Grasp_Point,Grasp_Point_list),
   rdf(Grasp_Point, knowrob:approachOrientationForRightFinger, Grasp_Point_0),
   rdf(Grasp_Point, knowrob:approachOrientationForLeftFinger, Grasp_Point_1),
   rdf(Grasp_Point, knowrob:approachOrientationForHand, Grasp_Point_Appr),
   rdf(Grasp_C_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Grasp_C_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Grasp_C_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Grasp_Point_0,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,R1x))),atom_to_term(R1x,R1X,_),
   rdf(Grasp_Point_0,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,R1y))),atom_to_term(R1y,R1Y,_),
   rdf(Grasp_Point_0,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,R1z))),atom_to_term(R1z,R1Z,_),
   rdf(Grasp_Point_1,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,L1x))),atom_to_term(L1x,L1X,_),
   rdf(Grasp_Point_1,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,L1y))),atom_to_term(L1y,L1Y,_),
   rdf(Grasp_Point_1,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,L1z))),atom_to_term(L1z,L1Z,_),
   rdf(Grasp_Point_Appr,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,H1x))),atom_to_term(H1x,H1X,_),
   rdf(Grasp_Point_Appr,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,H1y))),atom_to_term(H1y,H1Y,_),
   rdf(Grasp_Point_Appr,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,H1z))),atom_to_term(H1z,H1Z,_),
   Pose = [P1X, P1Y, P1Z, R1X, R1Y, R1Z, L1X, L1Y, L1Z, H1X, H1Y, H1Z]).
   
currentBboxSizse(Object, Size) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentDetection, knowrob:maxBoundingBox, Max_Bbox),
   rdf(CurrentDetection, knowrob:minBoundingBox, Max_Bbox),
   rdf(Max_Bbox,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Maxx))),atom_to_term(Maxx,MaxX,_),
   rdf(Max_Bbox,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Maxy))),atom_to_term(Maxy,MaxY,_),
   rdf(Max_Bbox,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Maxz))),atom_to_term(Maxz,MaxZ,_),

   rdf(Max_Bbox,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Minx))),atom_to_term(Minx,MinX,_),
   rdf(Max_Bbox,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Miny))),atom_to_term(Miny,MinY,_),
   rdf(Max_Bbox,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Minz))),atom_to_term(Minz,MinZ,_),
   X is MaxX-MinX, Y is MaxY-MinY, Z is MaxZ-MinZ,
   Size = [X,Y,Z].

handSize(Hand, HandSize):-
    rdfs_individual_of(Hand,  knowrob:'Hand'),
    rdf(Hand, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
    rdf(Hand, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
    rdf(Hand, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
    HandSize = [S1X, S1Y, S1Z].
	
placeSize(Place, PlaceSize):-
    rdfs_individual_of(Place,  knowrob:'SemanticEnvironmentMap'),
    rdf(Place, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
    rdf(Place, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
    rdf(Place, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
    PlaceSize = [S1X, S1Y, S1Z].
	
placePose(Place, Pose) :-
   semanticMapPerception(Place, CurrentPetection),
   rdfs_individual_of(Place,  knowrob:'SemanticEnvironmentMap'),
   rdf(CurrentPetection, knowrob:eventOccursAt, Place_Matrix),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,P1a))),atom_to_term(P1a,P1A,_),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,P1b))),atom_to_term(P1b,P1B,_),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,P1c))),atom_to_term(P1c,P1C,_),
   rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,P1d))),atom_to_term(P1d,P1D,_),
   Pose = [P1X, P1Y, P1Z, P1A, P1B, P1C, P1D].

   
objectsName(Place,Objects) :-
	rdfs_individual_of(Place,  knowrob:'SemanticEnvironmentMap'),
	semanticMapPerception(Place, PlaceDetection),
	findall(Object,(rdfs_individual_of(Object,  knowrob:'SpatialThing-Localized'),
    (currentObjectPerception(Object, ObjectPerception);semanticMapPerception(Object, ObjectPerception)),
	currentBboxSizse(Object, Size),
    nth0(0, Size, OD),
    nth0(1, Size, OW),
    nth0(2, Size, OH),
	Place\=Object,
    rdf(PlaceDetection,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Place_Matrix),
    rdf(ObjectPerception,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,PX,_),
    rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,PY,_),
    rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,PZ,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,OY,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,O2z))),atom_to_term(O2z,OZ,_),
    rdf(Place,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,P1d))),atom_to_term(P1d,PD,_),
    rdf(Place,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,P1w))),atom_to_term(P1w,PW,_),
    rdf(Place,'http://knowrob.org/kb/knowrob.owl#heightOfObject', literal(type(_,P1h))),atom_to_term(P1h,PH,_),
    % read the dimensions of the Object2
	
    %rdf(Object,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,O2d))),atom_to_term(O2d,OD,_),
    %rdf(Object,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,O2w))),atom_to_term(O2w,OW,_),
    %rdf(Object,'http://knowrob.org/kb/knowrob.owl#heightOfObject', literal(type(_,O2h))),atom_to_term(O2h,OH,_),
    % InnerObj is contained by OuterObj if (center_i+0.5*dim_i)<=(center_o+0.5*dim_o)
    % for all dimensions (x, y, z)
 
    >=( (PX - 0.5*PD), (OX - 0.5*OD)-0.5), =<( (PX + 0.5*PD),  (OX + 0.5*OD)+0.5 ),
    >=( (PY - 0.5*PW), (OY - 0.5*OW)-0.5 ), =<( (PY + 0.5*PW), (OY + 0.5*OW)+0.5 ),
    >=( (PZ - 0.5*PH), (OZ - 0.5*OH)-0.5 ), =<( (PZ + 0.5*PH), (OZ + 0.5*OH)+0.5 )), Objects).
   
objectPlaceName(Object,Place) :-
	rdfs_individual_of(Place,  knowrob:'SemanticEnvironmentMap'),
	semanticMapPerception(Place, PlaceDetection),
    currentObjectPerception(Object, ObjectPerception),
	currentBboxSizse(Object, Size),
    nth0(0, Size, OD),
    nth0(1, Size, OW),
    nth0(2, Size, OH),
   
    rdf(PlaceDetection,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Place_Matrix),
    rdf(ObjectPerception,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
	
    rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,PX,_),
    rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,PY,_),
    %rdf(Place_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,PZ,_),
	
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,OY,_),
    %rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,O2z))),atom_to_term(O2z,OZ,_),
	
    %rdf(Place,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,P1d))),atom_to_term(P1d,PD,_),
    %rdf(Place,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,P1w))),atom_to_term(P1w,PW,_),
    %rdf(Place,'http://knowrob.org/kb/knowrob.owl#heightOfObject', literal(type(_,P1h))),atom_to_term(P1h,PH,_),

    %>=( (PX - 0.5*PD), (OX - 0.5*OD)-0.5), =<( (PX + 0.5*PD),  (OX + 0.5*OD)+0.5 ),
    %>=( (PY - 0.5*PW), (OY - 0.5*OW)-0.5 ), =<( (PY + 0.5*PW), (OY + 0.5*OW)+0.5 ).
	>=( (PX - 0.5*PD), OX), =<( (PX + 0.5*PD), OX),
    >=( (PY - 0.5*PW), OY), =<( (PY + 0.5*PW), OY).
	
worldPosition(Object, WorldPose) :-
	currentObjectPosition(Object, Pose),
	currentRobotBodyPose(Robot, Pose),
	nth0(0, Pose, RX),
	nth0(1, Pose, RY),
	nth0(0, Size, OX),
    nth0(1, Size, OY),
	ThetaR = atan2(RY,RX),
	ThetaO = atan2(OY,OX),
	TwoOX = OX**2,
	TwoOY = OY**2,
	SumXY = TwoOX + TwoOY,
	Dist = sqrt(SumXY),
	SumRO = ThetaR  + ThetaO,
	CosO = cos(SumRO),
	SinO = sin(SumRO),
	FinalOX = RX + (Dist * CosO),
	FinalOY = RY + (Dist * SinO),
	OWorldTheta = atan2(FinalOY,FinalOX),
	WorldPose = [FinalOX, FinalOY, OWorldTheta].
	
%%%%%%%%%%여기
robotPlaceName(Robot, Place):-
	semanticMapPerception(Place, PlaceDetection),
    currentRobotBodyPose(Robot, Pose),
	nth0(0, Pose, RX),
	nth0(1, Pose, RY),
	rdf(PlaceDetection,'http://knowrob.org/kb/knowrob.owl#physicalParts', Place_Region),
	rdfs_individual_of(Place_Region,  knowrob:'Region'),
	
	rdf(Place_Region,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Region_Matrix),
	rdf(Place_Region,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,P2d))),atom_to_term(P2d,PD,_),
    rdf(Place_Region,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,P2w))),atom_to_term(P2w,PW,_),
	
	rdf(Region_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,PX,_),
    rdf(Region_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,PY,_),
    rdf(Region_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,PZ,_),

    >=( (PX - 0.5*PD), RX), =<( (PX + 0.5*PD),  RX),
    >=( (PY - 0.5*PW), RY), =<( (PY + 0.5*PW),  RY).
	
%%%empty
empty_slot(Plane, Goal):-
	semanticMapPerception(Place, PlaceDetection),
    currentObjectPerception(Object, Perception),
	rdf(Place,'http://knowrob.org/kb/knowrob.owl#physicalParts', Shelf),
	rdfs_individual_of(Shelf,  knowrob:'Shelf'),
	rdf(Shelf,'http://knowrob.org/kb/knowrob.owl#physicalParts', Plane),
	rdf(Plane,'http://knowrob.org/kb/knowrob.owl#physicalParts', Region),
	rdfs_individual_of(Region,  knowrob:'Region'),
	rdf(Region,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,P2d))),atom_to_term(P2d,PD,_),
    rdf(Region,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,P2w))),atom_to_term(P2w,PW,_),
	rdf(Region,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Region_Matrix),
	rdf(Region_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,PX,_),
    rdf(Region_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,PY,_),
	rdf(Object,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
	rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    >=( (PX - 0.5*PD), OX), =<( (PX + 0.5*PD),  OX),
    >=( (PY - 0.5*PW), OY), =<( (PY + 0.5*PW),  OY),
	once((Region == 'point1', Goal is 1);
   (Region == 'point2', Goal is 2);
   (Region == 'point3', Goal is 3)).
	
currentTipContactPerception(TipList) :-
      (rdfs_individual_of(Subject,  knowrob:'UrdfLink'),
      rdfs_individual_of(Subject,  knowrob:'Finger')),
      append([], Subject, TipList).

set([], []).
set([H|T], [H|T1]) :- subtract(T, [H], T2), set(T2, T1).


counter([],0).
counter([_|T],N) :-
      counter(T,M),
      N is M+1 .

currentHandTorque(Hand, Torques) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(Torque, (rdfs_individual_of(Fingertip, knowrob:'FingerTip'), currentFingerTipTorque(Fingertip,Torque)),Torques).
      
currentHandForce(Hand, Forces) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(Force, (rdfs_individual_of(Fingertip, knowrob:'FingerTip'), currentFingerTipForce(Fingertip,Force)),Forces).

currentObjectPose(Object, Pose) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   
   rdf(CurrentPerception, knowrob:coordinateXVector, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,X1x))),atom_to_term(X1x,X1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,X1y))),atom_to_term(X1y,X1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,X1z))),atom_to_term(X1z,X1Z,_),

   rdf(CurrentPerception, knowrob:coordinateYVector, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Y1x))),atom_to_term(Y1x,Y1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Y1y))),atom_to_term(Y1y,Y1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Y1z))),atom_to_term(Y1z,Y1Z,_),
   
   rdf(CurrentPerception, knowrob:coordinateZVector, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Z1x))),atom_to_term(Z1x,Z1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Z1y))),atom_to_term(Z1y,Z1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Z1z))),atom_to_term(Z1z,Z1Z,_),
   
   Pose = [P1X, P1Y, P1Z, X1X, X1Y, X1Z, Y1X, Y1Y, Y1Z, Z1X, Z1Y, Z1Z].

currentObjectPosition(Object, Pose) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   Pose = [P1X, P1Y, P1Z].

currentHandPose(Hand, Pose) :-
   currentHandPerception(Hand, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

currentRobotBodyPose(Body, Pose) :-
   currentRobotBodyPerception(Body, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].
   
semanticPose(Object, Pose) :-
   semanticPerception(Object, SemanticPerception),
   rdf(SemanticPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

semanticObjectPose(Object, Pose):-
   semanticMapPerception(Object, SemanticMapPerception),
   rdf(SemanticMapPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

	
currentJointAngle(Joint, Angle):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:'radius',literal(type(_,J1r))),atom_to_term(J1r,J1R,_),
   Angle = J1R.
   
currentJointVelocity(Joint, Velocity):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:'velocity',literal(type(_,J1v))),atom_to_term(J1v,J1V,_),
   Velocity = J1V.

currentJointEffort(Joint, Effort):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:'effort',literal(type(_,J1e))),atom_to_term(J1e,J1E,_),
   Effort = J1E.

tilted_Object(Object):-
   nb_getval(predicate__tilted_Object_low_angle_threshold, Low_angle_threshold),
   nb_getval(predicate__tilted_Object_high_angle_threshold, High_angle_threshold),
   currentObjectPose(Object, Pose),
   nth0(3, Pose, Tilted_RadiusX),
   nth0(4, Pose, Tilted_RadiusY),
   once(
   (Tilted_RadiusX < 0 , RadiusX is Tilted_RadiusX + 360);
   (Tilted_RadiusX >= 0 , RadiusX is Tilted_RadiusX )
   ),
   once(
   (Tilted_RadiusY < 0 , RadiusY is Tilted_RadiusY + 360);
   (Tilted_RadiusY >= 0 , RadiusY is Tilted_RadiusY )
   ),
   (RadiusX>Low_angle_threshold, RadiusX<High_angle_threshold),
   (RadiusY>Low_angle_threshold, RadiusY<High_angle_threshold).

standing_Object(Object):-
   nb_getval(predicate__standing_Object_low_angle_threshold, Low_angle_threshold),
   nb_getval(predicate__standing_Object_high_angle_threshold, High_angle_threshold),
   currentObjectPose(Object, Pose),
   nth0(3, Pose, Tilted_RadiusX),
   nth0(4, Pose, Tilted_RadiusY),
   once(
   (Tilted_RadiusX < 0 , RadiusX is Tilted_RadiusX + 360);
   (Tilted_RadiusX >= 0 , RadiusX is Tilted_RadiusX )
   ),
   once(
   (Tilted_RadiusY < 0 , RadiusY is Tilted_RadiusY + 360);
   (Tilted_RadiusY >= 0 , RadiusY is Tilted_RadiusY )
   ),
   (RadiusX=<Low_angle_threshold; RadiusX>=High_angle_threshold),
   (RadiusY=<Low_angle_threshold; RadiusY>=High_angle_threshold).

%추가 및 수정
currentObjectVertex(Object, ObjectPoint):-
   currentObjectPose(Object, ObjectPose),
   objectSize(Object, ObjectSize),
   nth0(0, ObjectPose, ObjectPoseX),
   nth0(1, ObjectPose, ObjectPoseY),
   nth0(2, ObjectPose, ObjectPoseZ),

   nth0(0, ObjectSize, ObjectSizeX),
   nth0(1, ObjectSize, ObjectSizeY),
   nth0(2, ObjectSize, ObjectSizeZ),

   P1X is ObjectPoseX - (ObjectSizeX/2),
   P1Y is ObjectPoseY - (ObjectSizeY/2),
   P1Z is ObjectPoseZ - (ObjectSizeZ/2),
   P1 = [P1X, P1Y, P1Z],
   P2X is ObjectPoseX - (ObjectSizeX/2),
   P2Y is ObjectPoseY + (ObjectSizeY/2),
   P2Z is ObjectPoseZ - (ObjectSizeZ/2),
   P2 = [P2X, P2Y, P2Z],
   P3X is ObjectPoseX + (ObjectSizeX/2),
   P3Y is ObjectPoseY + (ObjectSizeY/2),
   P3Z is ObjectPoseZ - (ObjectSizeZ/2),
   P3 = [P3X, P3Y, P3Z],
   P4X is ObjectPoseX + (ObjectSizeX/2),
   P4Y is ObjectPoseY - (ObjectSizeY/2),
   P4Z is ObjectPoseZ - (ObjectSizeZ/2),
   P4 = [P4X, P4Y, P4Z],
   P5X is ObjectPoseX - (ObjectSizeX/2),
   P5Y is ObjectPoseY - (ObjectSizeY/2),
   P5Z is ObjectPoseZ + (ObjectSizeZ/2),
   P5 = [P5X, P5Y, P5Z],
   P6X is ObjectPoseX - (ObjectSizeX/2),
   P6Y is ObjectPoseY + (ObjectSizeY/2),
   P6Z is ObjectPoseZ + (ObjectSizeZ/2),
   P6 = [P6X, P6Y, P6Z],
   P7X is ObjectPoseX + (ObjectSizeX/2),
   P7Y is ObjectPoseY + (ObjectSizeY/2),
   P7Z is ObjectPoseZ + (ObjectSizeZ/2),
   P7 = [P7X, P7Y, P7Z],
   P8X is ObjectPoseX + (ObjectSizeX/2),
   P8Y is ObjectPoseY - (ObjectSizeY/2),
   P8Z is ObjectPoseZ + (ObjectSizeZ/2),
   P8 = [P8X, P8Y, P8Z],
   ObjectPoint = [P1, P2, P3, P4, P5, P6, P7, P8].

%추가 및 수정
currentHandVertex(Hand, HandPoint):-
   currentHandPose(Hand, HandPose),
   handSize(Hand, HandSize),
   nth0(0, HandPose, HandPoseX),
   nth0(1, HandPose, HandPoseY),
   nth0(2, HandPose, HandPoseZ),

   nth0(0, HandSize, HandSizeX),
   nth0(1, HandSize, HandSizeY),
   nth0(2, HandSize, HandSizeZ),
   P1X is HandPoseX - (HandSizeX/2),
   P1Y is HandPoseY - (HandSizeY/2),
   P1Z is HandPoseZ - (HandSizeZ/2),
   P1 = [P1X, P1Y, P1Z],
   P2X is HandPoseX - (HandSizeX/2),
   P2Y is HandPoseY + (HandSizeY/2),
   P2Z is HandPoseZ - (HandSizeZ/2),
   P2 = [P2X, P2Y, P2Z],
   P3X is HandPoseX + (HandSizeX/2),
   P3Y is HandPoseY + (HandSizeY/2),
   P3Z is HandPoseZ - (HandSizeZ/2),
   P3 = [P3X, P3Y, P3Z],
   P4X is HandPoseX + (HandSizeX/2),
   P4Y is HandPoseY - (HandSizeY/2),
   P4Z is HandPoseZ - (HandSizeZ/2),
   P4 = [P4X, P4Y, P4Z],
   P5X is HandPoseX - (HandSizeX/2),
   P5Y is HandPoseY - (HandSizeY/2),
   P5Z is HandPoseZ + (HandSizeZ/2),
   P5 = [P5X, P5Y, P5Z],
   P6X is HandPoseX - (HandSizeX/2),
   P6Y is HandPoseY + (HandSizeY/2),
   P6Z is HandPoseZ + (HandSizeZ/2),
   P6 = [P6X, P6Y, P6Z],
   P7X is HandPoseX + (HandSizeX/2),
   P7Y is HandPoseY + (HandSizeY/2),
   P7Z is HandPoseZ + (HandSizeZ/2),
   P7 = [P7X, P7Y, P7Z],
   P8X is HandPoseX + (HandSizeX/2),
   P8Y is HandPoseY - (HandSizeY/2),
   P8Z is HandPoseZ + (HandSizeZ/2),
   P8 = [P8X, P8Y, P8Z],
   HandPoint = [P1, P2, P3, P4, P5, P6, P7, P8].

%추가 및 수정
intersects(Hand, Object, IntersectPer):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   rdfs_individual_of(Object,  knowrob:'Artifact'),
 
   currentObjectVertex(Object, ObjectPoint),
   currentHandVertex(Hand, HandPoint),
   
   nth0(0, HandPoint, Hand1), nth0(0, Hand1, X1), nth0(1, Hand1, Y1), nth0(2, Hand1, Z1),
   nth0(6, HandPoint, Hand2), nth0(0, Hand2, X2), nth0(1, Hand2, Y2), nth0(2, Hand2, Z2),
   nth0(0, ObjectPoint, Object3), nth0(0, Object3, X3), nth0(1, Object3, Y3), nth0(2, Object3, Z3),
   nth0(6, ObjectPoint, Object4), nth0(0, Object4, X4), nth0(1, Object4, Y4), nth0(2, Object4, Z4),
   
   once((X2<X3, LengthX is 0);
   (X1<X3, X3<X2, X2<X4, LengthX is X2-X3);
   (X1<X3, X3<X2, X4<X2, LengthX is X4-X3);
   (X3<X1, X2<X4, LengthX is X2-X1);
   (X3<X1, X1<X4, X4<X2, LengthX is X4-X1);
   (X4<X1, LengthX is 0)),
   
   once((Y2<Y3, LengthY is 0);
   (Y1<Y3, Y3<Y2, Y2<Y4, LengthY is Y2-Y3);
   (Y1<Y3, Y3<Y2, Y4<Y2, LengthY is Y4-Y3);
   (Y3<Y1, Y2<Y4, LengthY is Y2-Y1);
   (Y3<Y1, Y1<Y4, Y4<Y2, LengthY is Y4-Y1);
   (Y4<Y1, LengthY is 0)),
   
   once((Z2<Z3, LengthZ is 0);
   (Z1<Z3, Z3<Z2, Z2<Z4, LengthZ is Z2-Z3);
   (Z1<Z3, Z3<Z2, Z4<Z2, LengthZ is Z4-Z3);
   (Z3<Z1, Z2<Z4, LengthZ is Z2-Z1);
   (Z3<Z1, Z1<Z4, Z4<Z2, LengthZ is Z4-Z1);
   (Z4<Z1, LengthZ is 0)),
   
   HandLengthX is X2-X1, HandLengthY is Y2-Y1, HandLengthZ is Z2-Z1,
   HandVolume is HandLengthX*HandLengthY*HandLengthZ,
   IntersectVolume is LengthX*LengthY*LengthZ,
   IntersectPer is IntersectVolume/HandVolume.

empty_hand(Hand):-
   nb_getval(predicate__empty_hand_intersectPer_threshold, IntersectPer_threshold),
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   findall(Object,  (currentObjectPerception(Object, CurrentPerception)), Objects),
   (counter(Objects,0)->true;
   foreach(member(O,Objects), intersects(Hand, O, IntersectPer)),
   IntersectPer =< IntersectPer_threshold).

empty_hand_test(Hand,Object,IntersectPer):-
   nb_getval(predicate__empty_hand_intersectPer_threshold, IntersectPer_threshold),
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   intersects(Hand, Object, IntersectPer),
   findall(Object,  (currentObjectPerception(Object, CurrentPerception)), Objects),
   foreach(member(O,Objects), intersects(Hand, O, IntersectPer)).

type(Object, Type):-
	rdf(Object, rdf:type, Type).

%추가 및 수정
overlap_hand(Hand,Object):-
   nb_getval(predicate__overlap_hand_intersectPer_threshold, IntersectPer_threshold),
   intersects(Hand, Object, IntersectPer),
   IntersectPer > IntersectPer_threshold.
	
radianToDegree(Radian,Degree):-
	Degree1 is round(Radian * 57.295),
	once((Degree1 =< 0, Degree is 360 + Degree1);
	(Degree1 >= 360, Degree is mod(Degree1,360));
	(Degree1 >= 0, Degree1 < 360, Degree is Degree1)).
	
opened_hand(Hand):-
   nb_getval(predicate__joint_opened_hand_finger_number, Finger_number),
   joint_check(Hand,Number),
   Number >= Finger_number.
   
closed_hand(Hand):-
   nb_getval(predicate__joint_closed_hand_finger_number, Finger_number),
   joint_check(Hand,Number),
   Number < Finger_number.

closed_hand_test(Hand, Number):-
   nb_getval(predicate__joint_closed_hand_finger_number, Finger_number),
   joint_check(Hand,Number).
   
joint_ex(Hand,J1,J2,J3):-
   rdf(Hand, rdf:type,  knowrob:'Hand'),
   findall(Angle,  (rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint), 	rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'), currentJointAngle(Joint,Angle)),  Angles),
   [A1,A2,A3|_] = Angles,
   radianToDegree(A1,J1),
   radianToDegree(A2,J2),
   radianToDegree(A3,J3).

joint_check(Hand,Number):-
   nb_getval(predicate__joint_check_joint_angle_threshold, Joint_threshold),
   rdf(Hand, rdf:type,  knowrob:'Hand'),
   findall(Angle,  (rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint), 	rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'), currentJointAngle(Joint,Angle)),  Angles),
   [A1,A2,A3|_] = Angles,
/*   radianToDegree(A1,AD1),
   radianToDegree(A2,AD2),
   radianToDegree(A3,AD3),
   once((AD1 > Joint_threshold, AN1 is 1);
	(AN1 is 0)),
   once((AD2 > Joint_threshold, AN2 is 1);
	(AN2 is 0)),
   once((AD3 > Joint_threshold, AN3 is 1);
	(AN3 is 0)),
   Number is (AN1+AN2+AN3).
*/
   once((A1 < Joint_threshold, AN1 is 1);

	(AN1 is 0)),

   once((A2 < Joint_threshold, AN2 is 1);

	(AN2 is 0)),

   once((A3 < Joint_threshold, AN3 is 1);

	(AN3 is 0)),

   Number is (AN1+AN2+AN3).

%추가 및 수정
close_empty_hand(Hand):-
	closed_hand(Hand),
	empty_hand(Hand).

%추가 및 수정
close_overlap_hand(Hand):-
	closed_hand(Hand),
	overlap_hand(Hand,Object).

graspedBy(Object, Hand):-
   closed_hand(Hand),
   overlap_hand(Hand,Object).
/*
graspedBy(Hand, Object):-
   closed_hand(Hand),
   (rdf(Perception,'graspedBy',Hand),rdf_has(Perception,knowrob:'startTime',StartTime1),rdf_split_(StratTime1,_,T1),
   currentObjectPerception(Object, Perception2),rdf_has(Perception2,knowrob:'startTime',StartTime2),iri_xml_namespace(StartTime2,T2),
   T2 - T1 < 4 -> true;
   
   overlap_hand(Hand, Object),
   
   currentObjectPerception(Object, Perception),
   rdf_retractall(S,'graspedBy',Hand),
   rdf_assert(Perception,'graspedBy',Hand)).
*/
detected_object(Object) :-
   nb_getval(predicate__detected_object_min_time_threshold, Min_time_threshold),
   nb_getval(predicate__detected_object_max_time_threshold, Max_time_threshold),

   (rdfs_individual_of(Object,  knowrob:'Artifact');rdfs_individual_of(Object,  knowrob:'HumanScaleObject')),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
   latest_detection_of_object(Object, CurrentPerception),
   
   rdf_has(CurrentPerception, knowrob:'startTime', StartTimeR),
   rdf_split_url(_, StartTt, StartTimeR),
   atom_concat('timepoint_', StartTAtom, StartTt),
   term_to_atom(StartTime, StartTAtom),
   atom_number(Time,IntTime),
   get_time(ST),
   (ST - IntTime >= Min_time_threshold, ST - IntTime =< Max_time_threshold).
	

detected_object_test(Object, StartTimeR) :-
   nb_getval(predicate__detected_object_min_time_threshold, Min_time_threshold),
   nb_getval(predicate__detected_object_max_time_threshold, Max_time_threshold),

   (rdfs_individual_of(Object,  knowrob:'Artifact');rdfs_individual_of(Object,  knowrob:'HumanScaleObject')),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
   latest_detection_of_object(Object, CurrentPerception),
   
   rdf_has(CurrentPerception, knowrob:'startTime', StartTimeR),
   rdf_split_url(_, StartTt, StartTimeR).
   %atom_concat('timepoint_', StartTAtom, StartTt),
   %term_to_atom(StartTime, StartTAtom),
   %%atom_number(Time,IntTime),
   %get_time(ST),
   %(ST - IntTime >= Min_time_threshold, ST - IntTime =< Max_time_threshold).

latest_detection_of_object(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualObjectPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_detection_of_robot(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualRobotBodyPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
	
latest_detection_of_touch(Subject, LatestDetection) :-

  ((rdf_has(Subject, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Subject,St], (rdf_has(D_i, knowrob:objectActedOn, Subject),
                              ( rdfs_individual_of(D_i,  knowrob:'TouchPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
	
latest_detection_of_grasp(Subject, LatestDetection) :-

  ((rdf_has(Subject, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Subject,St], (rdf_has(D_i, knowrob:objectActedOn, Subject),
                              ( rdfs_individual_of(D_i,  knowrob:'GraspDetection')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_battery_detection_of_instance(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  arbi:'BatteryPerception'),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_detection_of_joint(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'JointPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
	
latest_detection_of_hand(Hand, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualRobotHandPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).




    % compatibility with Prolog < 5.8
:- if(\+current_predicate(atomic_list_concat, _)).

  atomic_list_concat(List, Atom) :-
    concat_atom(List, Atom).

  atomic_list_concat(List, Separator, Atom) :-
    concat_atom(List, Separator, Atom).

:- endif.
detection_starttime(Detection, StartTime) :-

  % start time is asserted
  rdf_triple(knowrob:startTime, Detection, StartTtG),
  rdf_split_url(_, StartTt, StartTtG),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom),! ;

  rdf_split_url(_, StartTt, Detection),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom).



detection_endtime(Detection, EndTime) :-

  rdf_triple(knowrob:endTime, Detection, EndTtG),
  rdf_split_url(_, EndTt, EndTtG),
  atom_concat('timepoint_', EndTAtom, EndTt),
  term_to_atom(EndTime, EndTAtom),!;

  ( rdf_has(LaterDetection, knowrob:previousDetectionOfObject, Detection),
    rdf_triple(knowrob:startTime, LaterDetection, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  ( rdf_has(Detection, knowrob:objectActedOn, Object),
    rdf_has(Destruction, knowrob:inputsDestroyed, Object),
    Destruction \= Detection,
    rdfs_individual_of(Destruction,  knowrob:'PhysicalDestructionEvent'),
    rdf_triple(knowrob:startTime, Detection, StT),
    rdf_triple(knowrob:startTime, Destruction, EndTtG),
    rdf_triple(knowrob:after, StT, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % otherwise take the current time (plus a second to avoid glitches)
  ( get_time(ET), EndTime is ET + 1.0).

%% compare_object_detections(-Delta, +P1, +P2) is det.
%
% Sort detections by their start time
%
% @param Delta  One of '>', '<', '='
% @param P1     List [_, _, Time] as used in latest_detection_of_object, latest_detection_of_type, latest_inferred_object_set
% @param P2     List [_, _, Time] as used in latest_detection_of_object, latest_detection_of_type, latest_inferred_object_set
%
compare_object_detections(Delta, P1, P2) :-

    nth0(2, P1, St1),
    nth0(2, P2, St2),
    compare(Delta, St2, St1).
