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
        TempPosition RelocatePosition - Position 
	)

	(:predicates
		(obstruct ?hand - Object ?target - Object ?obstacle - Object)
		(delivered ?obj - Object)
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
		(inWorkspace ?robotBase - Object ?target - Position)
		(heavyWeight ?obj - Object)
		(largeSized ?obj - Object)
		(type ?object - Object ?type - Object)
		(emptyHand ?hand - Object)
		(onPhysical ?object - Object ?supportPlane - Object)
		(near ?obj1 - Object ?obj2 - Object)
		(inFrontOf ?obj1 - object ?obj2 - object)
		(belowOf ?obj1 - object ?obj2 - object)
		(behind ?obj1 - object ?obj2 - object)
		(emptyContainer ?obj - object)
	)

	;;;;;;;;;;;;;;;;;;;;;;;;;;; Define actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;

	(:action move_arm
		:parameters (?robotArm - Object ?initPosition ?goalPosition - Position)
        :precondition 
            (and                     
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
                (detectedObject ?targetObject)
                (not (heavyWeight ?targetObject))
                (not (largeSized ?targetObject))
                (openedHand ?robotPart)
                (emptyHand ?robotPart)
                (not (graspedBy ?robotPart ?targetObject)) 
                (locatedAt ?robotPart ?initPosition)
                (locatedAt ?targetObject ?objectPosition)                
                (forall (?objects - Object) 
                    (not (obstruct ?robotPart ?targetObject ?objects))
                )
                (inWorkspace ?robotPart ?objectPosition)
            )
        :effect
            (and
                (not (openedHand ?robotPart))
                (not (emptyHand ?robotPart))
                (graspedBy ?robotPart ?targetObject)
                (locatedAt ?robotPart ?objectPosition)
                (not (locatedAt ?robotPart ?initPosition))
            )
		:constraints (?position - controller ?Gripper - hardware_group ?grasp - planner)
		:primitives (?robotPart ?targetObject ?initPosition ?objectPosition - approach_arm ?robotPart ?targetObject ?objectPosition - grasp_object)				
	)

    (:action hold_object_with_dualarm
		:parameters (?leftArm ?rightArm ?targetObject - Object ?leftArmPosition ?rightArmPosition ?objectPosition - Position)
        :precondition 
            (and
                (or
                    (heavyWeight ?targetObject)
                    (largeSized ?targetObject)
                )
                (not (= ?rightArm ?leftArm))
                (detectedObject ?targetObject)
                (openedHand ?leftArm)
                (openedHand ?rightArm)
                (emptyHand ?leftArm)
                (emptyHand ?rightArm)
                (not (graspedBy ?leftArm ?targetObject)) 
                (not (graspedBy ?rightArm ?targetObject)) 
                (locatedAt ?leftArm ?leftArmPosition)
                (locatedAt ?rightArm ?rightArmPosition)
                (locatedAt ?targetObject ?objectPosition)                

                (inWorkspace ?leftArm ?objectPosition)
                (inWorkspace ?rightArm ?objectPosition)
            )
        :effect
            (and
                (not (openedHand ?leftArm))
                (not (openedHand ?rightArm))
                (not (emptyHand ?leftArm))
                (not (emptyHand ?rightArm))
                (graspedBy ?leftArm ?targetObject)
                (graspedBy ?rightArm ?targetObject)
                (locatedAt ?leftArm ?objectPosition)
                (locatedAt ?rightArm ?objectPosition)
                (not (locatedAt ?leftArm ?leftArmPosition))
                (not (locatedAt ?rightArm ?rightArmPosition))
            )
		:constraints (?position - controller ?Gripper - hardware_group ?grasp - planner)
	)

	(:action relocate_object
		:parameters (?robotPart - Object ?targetObject - Object ?initPosition ?goalPosition - Position)
        :precondition 
        (and  
            (detectedObject ?targetObject)
            (locatedAt ?robotPart ?initPosition)
            (locatedAt ?targetObject ?initPosition)            
            (= ?goalPosition RelocatePosition)
            (not (locatedAt ?robotPart ?goalPosition)) 
            (not (locatedAt ?targetObject ?goalPosition)) 
            (graspedBy ?robotPart ?targetObject)
        )
        :effect 
        (and 
            (not (locatedAt ?robotPart ?initPosition))
            (locatedAt ?robotPart ?goalPosition)
            (locatedAt ?targetObject ?goalPosition)
            (not (locatedAt ?targetObject ?initPosition))
            (forall (?objects - Object) 
                (when   (obstruct ?robotPart ?objects ?targetObject)                 
                    (not (obstruct ?robotPart ?objects ?targetObject))
                )
            )
        )
		:constraints (?position - controller ?Arm - hardware_group ?movearm - planner)
		:primitives (?robotPart ?initPosition ?goalPosition - move_arm)	
	)

	(:action close_hand
		:parameters (?robotPart - Object)
        :precondition 
            (and 
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

	(:action handover_object
		:parameters (?fromPart ?toPart ?targetObject - Object ?toPartPosition ?objectPosition ?handoverPosition - Position)
        :precondition 
            (and         
                (detectedObject ?targetObject)
                (= ?handoverPosition TempPosition)
                (not (heavyWeight ?targetObject))
                (not (largeSized ?targetObject))             
                (graspedBy ?fromPart ?targetObject)
                (not (graspedBy ?toPart ?targetObject))
                (locatedAt ?targetObject ?objectPosition)
                (locatedAt ?fromPart ?objectPosition)
                (locatedAt ?toPart ?toPartPosition)
                (inWorkspace ?fromPart ?objectPosition)
                (not (inWorkspace ?toPart ?objectPosition))
            )
        :effect
            (and
                (inWorkspace ?toPart ?objectPosition)
                (locatedAt ?fromPart ?handoverPosition)
                (locatedAt ?targetObject ?handoverPosition)
                (locatedAt ?toPart ?handoverPosition)
                (graspedBy ?toPart ?targetObject)
            )
		:constraints (?position - controller ?Arm - hardware_group ?movearm - planner)
		:primitives (?fromPart ?objectPosition ?handoverPosition - move_arm 
                    ?toPart ?targetObject ?toPartPosition ?handoverPosition- hold_object)	
	)
	(:action approach_arm
		:parameters (?robotArm ?targetObject - Object ?initPosition ?targetPosition - Position)
        :precondition
            (and
                (openedHand ?robotArm)
                (detectedObject ?targetObject)
                (not (= ?initPosition ?targetPosition))
                (not (graspedBy ?robotArm ?targetObject))
                (locatedAt ?robotArm ?initPosition)
                (locatedAt ?targetObject ?targetPosition)
                (forall (?objects - Object) 
                    (not (obstruct ?robotArm ?targetObject ?objects))
                )
                (inWorkspace ?robotArm ?targetPosition)
                (not (graspable ?robotArm ?targetObject))
            )
        :effect
            (and
                (graspable ?robotArm ?targetObject)
                (not (locatedAt ?robotArm ?initPosition))
                (locatedAt ?robotArm ?targetPosition)
            )
		:constraints (?position - controller ?Arm - hardware_group ?approach - planner)
	)

	(:action approach_base
    :parameters (?robotBase ?robotPart ?targetObject - Object ?initPosition ?targetPosition - Position)
    :precondition
        (and             
            (type ?robotBase Mobile)
            (type ?robotPart Gripper)
            (not (type ?targetObject Gripper))
            (detectedObject ?targetObject)
            (not (= ?robotBase ?robotPart))
            (not (= ?robotBase ?targetObject))
            (not (= ?robotPart ?targetObject))
            (locatedAt ?robotBase ?initPosition)
            (locatedAt ?targetObject ?targetPosition)  
            (not (inWorkspace ?robotPart ?targetPosition))
            (not (graspedBy ?robotPart ?targetObject))
        )    
    :effect
        (and
            (not (locatedAt ?robotBase ?initPosition))
            (locatedAt ?robotBase ?targetPosition)
            (inWorkspace ?robotPart ?targetPosition)
        )
		:constraints (?position - controller ?Mobile - hardware_group ?approachbase - planner)
	)

    (:action open_container
    :parameters (?robotBase ?targetContainer - Object ?containerPosition - Position)
    :precondition
        (and            
            (type ?robotBase Mobile) 
            (not (= ?robotBase ?targetContainer))
            (not (openedContainer ?targetContainer)) 
            (locatedAt ?robotBase ?containerPosition)
            (locatedAt ?targetContainer ?containerPosition)            
        )    
    :effect
        (and           
		    (openedContainer ?targetContainer)
            (forall (?object - Object)
                (when (inContGeneric ?targetContainer ?object)
                    (detectedObject ?object)
                )
            )
        )
		:constraints (?position - controller ?Mobile - hardware_group ?approach - planner)
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
)