(define
	(domain socialrobot)

	(:requirements
		:strips
		:adl
		:disjunctive-preconditions
		:negative-preconditions
		:typing
		:equality
	)

	(:types
		Position
		Object
	)

	(:constants
		DualArm Arm Mobile Gripper - Object        
        TempPosition RelocatePosition HandoverPosition - Position 
	)

	(:predicates
        (metric)
		(obstruct ?hand - Object ?target - Object ?obstacle - Object)
		(transferred ?hand ?object - Object)
		(aboveOf ?obj1 - Object ?obj2 - Object)
		(accesible ?robotpart - Object ?object - Object)
		(inContGeneric ?container - Object ?content - Object)
		(graspable ?hand - Object ?object - Object)
		(clear ?position - Position)
		(graspedBy ?hand - Object ?object - Object)
		(detectedObject ?object - Object)
		(locatedAt ?object - Object ?location - Position)
		(openedHand ?hand - Object)
		(openedContainer ?container - Object)
		(inWorkspace ?robotBase - Object ?target - Object)
		(heavyWeight ?obj - Object)
		(largeSized ?obj - Object)
		(type ?object - Object ?type - Object)
		(emptyHand ?hand - Object)
        (rearranged ?object - Object)
		(onPhysical ?object - Object ?supportPlane - Object)
		(near ?obj1 - Object ?obj2 - Object)
		(inFrontOf ?obj1 - object ?obj2 - object)
		(belowOf ?obj1 - object ?obj2 - object)
		(behind ?obj1 - object ?obj2 - object)
		(emptyContainer ?obj - object)
        (standby ?robotpart - Object)
	)

	;;;;;;;;;;;;;;;;;;;;;;;;;;; Define actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;
	(:action standby
		:parameters (?robotPart - Object)
        :precondition 
            (and   
                (not (standby ?robotPart))                  
            )
        :effect
            (and
                (standby ?robotPart)
            ) 
		:constraints (?position - controller ?Arm - hardware_group ?standby - planner)  
	)

	(:action move_arm
		:parameters (?robotArm - Object ?initPosition ?goalPosition - Position)
        :precondition 
            (and       
                (metric)              
            )
        :effect
            (and
            ) 
		:constraints (?position - controller ?Arm - hardware_group ?movearm - planner)  
	)

    (:action move_base
		:parameters (?robotBase - Object ?initPosition ?goalPosition - Position)
        :precondition 
            (and   
                (metric)
            )
        :effect
            (and
            ) 
		:constraints (?position - controller ?Mobile - hardware_group ?movebase - planner)  
	)

    (:action carry_object
		:parameters (?robotBase ?robotPart ?targetObject - Object ?initPosition ?goalPosition - Position)
        :precondition 
            (and   
                (type ?robotBase Mobile)
                (locatedat ?robotBase ?initPosition)
                (graspedBy ?robotPart ?targetObject)
            )
        :effect
            (and
                (not (locatedat ?robotBase ?initPosition))
                (locatedat ?robotBase ?goalPosition)
            ) 
		:constraints (?position - controller ?Mobile - hardware_group ?movebase - planner)  
		:primitives (?robotBase ?initPosition ?goalPosition - move_base)
	)

	(:action release_object
		:parameters (?robotPart ?targetObject - Object ?initPosition - Position)
        :precondition 
            (and
                (type ?robotPart Gripper)
                (not (openedHand ?robotPart)) 
                (not (emptyHand ?robotPart))
                (graspedBy ?robotPart ?targetObject) 
                (locatedAt ?robotPart ?initPosition)
                (locatedAt ?targetObject ?initPosition)
            )
        :effect
            (and
                (openedHand ?robotPart)
                (emptyHand ?robotPart)
                (not (graspedBy ?robotPart ?targetObject))
            )
		:constraints (?position - controller ?Gripper - hardware_group ?grasp - planner)
		:primitives (?robotPart - open_hand)
	)

	(:action open_hand
		:parameters (?robotPart - Object)
        :precondition 
            (and         
                (type ?robotPart Gripper)
                (not (openedHand ?robotPart))   
            )
        :effect
            (and
                (openedHand ?robotPart)
            )
		:constraints (?position - controller ?Gripper - hardware_group ?openclose - planner)
	)

	(:action hold_object
		:parameters (?robotPart ?targetObject - Object ?initPosition ?objectPosition - Position)
        :precondition 
            (and
                (type ?robotPart Gripper)
                (detectedObject ?targetObject)
                (not (heavyWeight ?targetObject))
                (not (largeSized ?targetObject))
                (openedHand ?robotPart)
                ;(emptyHand ?robotPart)
                (not (graspedBy ?robotPart ?targetObject)) 
                (locatedAt ?robotPart ?initPosition)
                (locatedAt ?targetObject ?objectPosition)                
                (forall (?objects - Object) 
                    (not (obstruct ?robotPart ?targetObject ?objects))
                )
                (inWorkspace ?robotPart ?targetObject)
            )
        :effect
            (and
                (not (openedHand ?robotPart))
                ;(not (emptyHand ?robotPart))
                (graspedBy ?robotPart ?targetObject)
                (locatedAt ?robotPart ?objectPosition)
                (not (locatedAt ?robotPart ?initPosition))
            )
		:constraints (?position - controller ?Gripper - hardware_group ?grasp - planner)
		:primitives (?robotPart ?targetObject ?initPosition ?objectPosition - approach_arm ?robotPart ?targetObject ?objectPosition - grasp_object)				
	)

    ; (:action hold_object_with_dualarm
	; 	:parameters (?leftArm ?rightArm ?targetObject - Object ?leftArmPosition ?rightArmPosition ?objectPosition - Position)
    ;     :precondition 
    ;         (and
    ;             (or
    ;                 (heavyWeight ?targetObject)
    ;                 (largeSized ?targetObject)
    ;             )
    ;             (not (= ?rightArm ?leftArm))
    ;             (detectedObject ?targetObject)
    ;             (openedHand ?leftArm)
    ;             (openedHand ?rightArm)
    ;             (emptyHand ?leftArm)
    ;             (emptyHand ?rightArm)
    ;             (not (graspedBy ?leftArm ?targetObject)) 
    ;             (not (graspedBy ?rightArm ?targetObject)) 
    ;             (locatedAt ?leftArm ?leftArmPosition)
    ;             (locatedAt ?rightArm ?rightArmPosition)
    ;             (locatedAt ?targetObject ?objectPosition)                

    ;             (inWorkspace ?leftArm ?targetObject)
    ;             (inWorkspace ?rightArm ?targetObject)
    ;         )
    ;     :effect
    ;         (and
    ;             (not (openedHand ?leftArm))
    ;             (not (openedHand ?rightArm))
    ;             (not (emptyHand ?leftArm))
    ;             (not (emptyHand ?rightArm))
    ;             (graspedBy ?leftArm ?targetObject)
    ;             (graspedBy ?rightArm ?targetObject)
    ;             (locatedAt ?leftArm ?objectPosition)
    ;             (locatedAt ?rightArm ?objectPosition)
    ;             (not (locatedAt ?leftArm ?leftArmPosition))
    ;             (not (locatedAt ?rightArm ?rightArmPosition))
    ;         )
	; 	:constraints (?position - controller ?Gripper - hardware_group ?grasp - planner)
	; )

    (:action relocate_obstacle
		:parameters (?robotPart - Object ?targetObject - Object ?relocateObstacle - Object ?initPosition ?goalPosition - Position)
        :precondition 
        (and  
            (type ?robotPart Gripper)
            (detectedObject ?relocateObstacle)
            (locatedAt ?robotPart ?initPosition)
            (locatedAt ?relocateObstacle ?initPosition)            
            (= ?goalPosition RelocatePosition)
            (not (locatedAt ?robotPart ?goalPosition)) 
            (not (locatedAt ?relocateObstacle ?goalPosition)) 
            (graspedBy ?robotPart ?relocateObstacle)
            (obstruct ?robotPart ?targetObject ?relocateObstacle)   
        )
        :effect 
        (and 
            (not (locatedAt ?robotPart ?initPosition))
            (locatedAt ?robotPart ?goalPosition)
            (locatedAt ?relocateObstacle ?goalPosition)
            (not (locatedAt ?relocateObstacle ?initPosition))
            (not (obstruct ?robotPart ?targetObject ?relocateObstacle))
        )
		:constraints (?position - controller ?Arm - hardware_group ?relocate - planner)
	)

    (:action rearrange_object
		:parameters (?robotPart - Object ?targetObject - Object ?initPosition ?rearragePosition - Position)
        :precondition 
        (and  
            (type ?robotPart Gripper)
            (detectedObject ?targetObject)
            (locatedAt ?robotPart ?initPosition)     
            (locatedAt ?targetObject ?initPosition)            
            (= ?rearragePosition RelocatePosition)
            (not (locatedAt ?robotPart ?rearragePosition)) 
            (not (locatedAt ?targetObject ?rearragePosition)) 
            (graspedBy ?robotPart ?targetObject)
        )
        :effect 
        (and 
            (not (locatedAt ?robotPart ?initPosition))
            (not (locatedAt ?targetObject ?initPosition))
            (locatedAt ?robotPart ?rearragePosition)
            (locatedAt ?targetObject ?rearragePosition)
            (rearranged ?targetObject)
        )
		:constraints (?position - controller ?Arm - hardware_group ?relocate - planner)
	)
    
	(:action close_hand
		:parameters (?robotPart - Object)
        :precondition 
            (and 
                (type ?robotPart Gripper)
                (openedHand ?robotPart)              
            )
        :effect
            (and
                (not (openedHand ?robotPart))
            )
		:constraints (?position - controller ?Gripper - hardware_group ?openclose - planner)
	)

	(:action grasp_object
		:parameters (?robotPart ?targetObject - Object ?nearPosition ?goalPosition - Position)
        :precondition
            (and
            )    
        :effect
            (and
            )
		:constraints (?position - controller ?Gripper - hardware_group ?grasp - planner)
		:primitives (?robotPart - close_hand)	
	)

	; (:action handover_dualarm
	; 	:parameters (?fromPart ?toPart ?targetObject - Object ?fromPartPosition ?toPartPosition ?objectPosition ?handoverPosition - Position)
    ;     :precondition 
    ;         (and         
    ;             (type ?fromPart Gripper)
    ;             (type ?toPart Gripper)
    ;             (emptyhand ?toPart)
    ;             (openedHand ?toPart)
    ;             (emptyhand ?fromPart)
    ;             (openedHand ?fromPart)
    ;             (detectedObject ?targetObject)
    ;             (= ?handoverPosition HandoverPosition)
    ;             (not (heavyWeight ?targetObject))
    ;             (not (largeSized ?targetObject))             
    ;             (not (graspedBy ?fromPart ?targetObject))
    ;             (not (graspedBy ?toPart ?targetObject))
    ;             (locatedAt ?targetObject ?objectPosition)
    ;             (locatedAt ?fromPart ?fromPartPosition)
    ;             (locatedAt ?toPart ?toPartPosition)
    ;             (inWorkspace ?fromPart ?targetObject)
    ;             (not (inWorkspace ?toPart ?targetObject))
    ;         )
    ;     :effect
    ;         (and
    ;             (inWorkspace ?toPart ?targetObject)
    ;             (locatedAt ?fromPart ?handoverPosition)
    ;             (locatedAt ?targetObject ?handoverPosition)
    ;             (locatedAt ?toPart ?handoverPosition)
    ;             (forall (?objects - Object) 
	; 			    (not (obstruct ?topart ?targetObject ?objects))
	; 		    )
    ;             (graspedBy ?toPart ?targetObject)
    ;         )
	; 	:constraints (?position - controller ?Arm - hardware_group ?handover - planner)
	; 	:primitives (?fromPart ?targetObject ?fromPartPosition ObjectBottom - approach_arm
    ;                 ?fromPart - close_hand
    ;                 ?fromPart ?toPart ?targetObject ?fromPartPosition ?objectPosition ?handoverPosition - handover_object
    ;                 ?toPart ?targetObject ?toPartPosition ?handoverPosition - takeover_object
    ;                 ?toPart - close_hand
    ;                 ?fromPart - open_hand)
	; )

	(:action handover_object
		:parameters (?fromPart ?toPart ?targetObject - Object ?fromPartPosition ?objectPosition ?handoverPosition - Position)
        :precondition 
            (and      
                (metric)
            )
        :effect
            (and
                (inWorkspace ?toPart ?targetObject)
                (locatedAt ?fromPart ?handoverPosition)
                (locatedAt ?targetObject ?handoverPosition)
                (locatedAt ?toPart ?handoverPosition)
            )
		:constraints (?position - controller ?Arm - hardware_group ?handover - planner)
	)

	(:action takeover_object
		:parameters (?toPart ?targetObject - Object ?toPartPosition ?handoverPosition - Position)
        :precondition 
            (and         
                (metric)
            )
        :effect
            (and
                (not (emptyhand ?toPart))
            )
		:constraints (?position - controller ?Arm - hardware_group ?approach - planner)
        :primitives (?toPart ?targetObject ?toPartPosition ObjectTakeover - approach_arm)
	)

	(:action approach_arm
		:parameters (?robotArm ?targetObject - Object ?initPosition ?targetPosition - Position)
        :precondition
            (and
                (type ?robotArm Gripper)
                (openedHand ?robotArm)
                (detectedObject ?targetObject)
                (not (= ?initPosition ?targetPosition))
                (not (graspedBy ?robotArm ?targetObject))
                (locatedAt ?robotArm ?initPosition)
                (locatedAt ?targetObject ?targetPosition)
                (forall (?objects - Object) 
                    (not (obstruct ?robotArm ?targetObject ?objects))
                )
                (inWorkspace ?robotArm ?targetObject)
                (not (graspable ?robotArm ?targetObject))
            )
        :effect
            (and
                ;(graspable ?robotArm ?targetObject)
                ;(not (locatedAt ?robotArm ?initPosition))
                ;(locatedAt ?robotArm ?targetPosition)
                (not (emptyHand ?robotArm))
            )
		:constraints (?position - controller ?Arm - hardware_group ?approach - planner)
	)

	; (:action approach_base
    ; :parameters (?robotBase ?robotPart ?targetObject - Object ?initPosition ?targetPosition - Position)
    ; :precondition
    ;     (and             
    ;         (type ?robotBase Mobile)
    ;         (type ?robotPart Gripper)
    ;         (not (type ?targetObject Gripper))
    ;         (detectedObject ?targetObject)
    ;         (not (= ?robotBase ?robotPart))
    ;         (not (= ?robotBase ?targetObject))
    ;         (not (= ?robotPart ?targetObject))
    ;         (locatedAt ?robotBase ?initPosition)
    ;         (locatedAt ?targetObject ?targetPosition)  
    ;         (not (inWorkspace ?robotPart ?targetObject))
    ;         (not (graspedBy ?robotPart ?targetObject))
    ;     )    
    ; :effect
    ;     (and
    ;         (not (locatedAt ?robotBase ?initPosition))
    ;         (locatedAt ?robotBase ?targetPosition)
    ;         (inWorkspace ?robotPart ?targetObject)
    ;     )
	; 	:constraints (?position - controller ?Mobile - hardware_group ?approachbase - planner)
	; )

    (:action open_container
    :parameters (?robotBase ?robotPart ?targetContainer - Object ?robotPos ?robotPartPos ?containerPos - Position)
    :precondition
        (and            
            (type ?robotBase Mobile) 
            (not (= ?robotBase ?targetContainer))
            (not (openedContainer ?targetContainer)) 
            (locatedAt ?robotBase ?robotPos)
            (locatedAt ?robotPart ?robotPartPos)  
            (locatedAt ?targetContainer ?containerPos)            
        )    
    :effect
        (and           
		    (openedContainer ?targetContainer)
            (forall (?object - Object)
                (when (inContGeneric ?targetContainer ?object)
                    (and 
                        ;(detectedObject ?object)
                        (inWorkspace ?robotPart ?object)
                    )
                )
            )
        )
		:constraints (?position - controller ?Mobile - hardware_group ?approach - planner)
        :primitives (?robotPart ?robotPos containerHandle - move_arm
        ?robotPart - close_hand
        ?robotPart ?targetContainer ?containerPos - open_door
        ?robotPart - open_hand
        ?robotPart - standby
        ?robotBase ?robotPos ?containerPos - move_base
        )
	)

    (:action open_door
    :parameters (?robotBase ?targetContainer - Object ?containerPosition - Position)
    :precondition
        (and      
                (metric)        
        )    
    :effect
        (and 
        )
		:constraints (?position - controller ?Mobile - hardware_group ?opendoor - planner)
    )

    (:action close_container
    :parameters (?robotBase ?targetContainer - Object ?containerPosition - Position)
    :precondition
        (and         
            (type ?robotBase Mobile)   
            (not (= ?robotBase ?targetContainer))
		    (openedContainer ?targetContainer)
            (locatedAt ?robotBase ?containerPosition)
            (locatedAt ?targetContainer ?containerPosition)  
        )    
    :effect
        (and            
            (not (openedContainer ?targetContainer))
        )
		:constraints (?position - controller ?Arm - hardware_group ?approach - planner)
	)

    (:action move_around
    :parameters (?robotBase ?targetObject - Object  ?robotPosition - Position)
    :precondition
        (and             
            (type ?robotBase Mobile)
            (not (detectedObject ?targetObject))
            (forall (?container - Object)
                (not (inContGeneric ?container ?targetObject))            
            )
            (locatedAt ?robotBase ?robotPosition)  
        )    
    :effect
        (and            
            (detectedObject ?targetObject)
        )
		:constraints (?position - controller ?Arm - hardware_group ?movearound - planner)
	)

	(:action transfer_object
    :parameters (?robotPart ?targetObject - Object  ?robotPosition - Position)
    :precondition
        (and            
            (type ?robotPart Gripper) 
            (locatedat ?robotPart ?robotPosition) 
            (graspedBy ?robotPart ?targetObject)
        )    
    :effect
        (and            
            (transferred ?robotPart ?targetObject)
            (not (graspedBy ?robotPart ?targetObject))
        )
		:constraints (?position - controller ?Arm - hardware_group ?transfer - planner)
	)
)
