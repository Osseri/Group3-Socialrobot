(define
	(domain socialrobot)

	(:requirements
		:strips
		:adl
		:disjunctive-preconditions
		:negative-preconditions
        :conditional-effects
		:typing
		:equality
	)

	(:types
		Position
		Object
        ObjectType
	)

	(:constants
		DualArm Arm Mobile - ObjectType        
        TempPosition RelocatePosition HandoverPosition CurrentPosition - Position 
	)

	(:predicates
        (primitive)
        (obstruct ?hand - Object ?target - Object ?obstacle - Object)
        (accesible ?robot_group - Object ?object - Object)
        (inContGeneric ?content - Object ?container - Object)
        (graspedBy ?hand - Object ?object - Object)
        (clear ?position - Position)
        (detectedObject ?object - Object)
        (locatedAt ?object - Object ?location - Position)
        (openedHand ?hand - Object)
        (openedContainer ?container - Object)
        (inWorkspace ?robot_group - Object ?target - Object)
        (heavyWeight ?obj - Object)
        (largeSized ?obj - Object)
        (type ?object - Object ?type - ObjectType)
        (emptyHand ?hand - Object)
        (rearranged ?object - Object)
        (onPhysical ?object - Object ?supportPlane - Object)
        (near ?obj1 - Object ?obj2 - Object)
        (inFrontOf ?obj1 - object ?obj2 - object)
        (belowOf ?obj1 - object ?obj2 - object)
        (behind ?obj1 - object ?obj2 - object)
        (aboveOf ?obj1 - Object ?obj2 - Object)
        (emptyContainer ?obj - object)
        (standby ?robot_group - Object)
        (affordanceOf ?object - Object ?affordance - Object)
        (transferred ?object ?to - Object)
        (delivered ?object ?to - Object)
        (oneHandGraspable ?object - Object)
        (twoHandGraspable ?object - Object)
        (handedover ?robot_group ?object - Object)
	)

	;;;;;;;;;;;;;;;;;;;;;;;;;;; Define actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;;; DO NOT add duplicate action name

    ; initialize robot pose
	(:action standby
		:parameters (?robot_group - Object)
        :precondition 
            (and   
                (not (standby ?robot_group))                  
            )
        :effect
            (and
                (standby ?robot_group)
            ) 
		:constraints (?position - controller ?Arm - hardware_group ?standby - planner)  
	)

    ; initialize robot pose with grasped object
    (:action standby_with_object
		:parameters (?robot_group ?target_object - Object)
        :precondition 
            (and   
                (primitive)
            )
        :effect
            (and
            ) 
		:constraints (?position - controller ?Arm - hardware_group ?standbywithobject - planner)  
	)

    ; open robot gripper
    (:action open_hand
		:parameters (?robot_group - Object)
        :precondition 
            (and         
                (or
                    (type ?robot_group Arm)
                    (type ?robot_group DualArm)
                )
                (not (openedHand ?robot_group))   
            )
        :effect
            (and
                (openedHand ?robot_group)
                ; (when (type ?robot_group DualArm)   ;when dual arm open, all single arm also open
                ;     (forall (?arm - object)
                ;         (when (type ?arm Arm)
                ;             (openedHand ?arm)
                ;         )
                ;     ) 
                ; )
			    (forall (?objects - object)     ;if object is graspedby gripper, not graspedby effect
				    (when (graspedby ?robot_group ?objects)
					    (not (graspedby ?robot_group ?objects))
				    )
			    )
            )
		:constraints (?position - controller ?Gripper - hardware_group ?openhand - planner)
	)    

    ; close robot gripper
	(:action close_hand
		:parameters (?robot_group - Object)
        :precondition 
            (and 
                (or
                    (type ?robot_group Arm)
                    (type ?robot_group DualArm)
                )
                (openedHand ?robot_group)              
            )
        :effect
            (and
                (not (openedHand ?robot_group))
            )
		:constraints (?position - controller ?Gripper - hardware_group ?closehand - planner)
	)

    ; hold object using single-arm
	(:action hold_object
		:parameters (?robot_group ?target_object - Object ?initPosition ?objectPosition - Position)
        :precondition 
            (and
                (type ?robot_group Arm)
                (not (type ?robot_group DualArm))
                (not (type ?robot_group Mobile))
                (not (type ?target_object Arm))
                (not (type ?target_object DualArm))
                (not (type ?target_object Mobile))
                (detectedObject ?target_object)
                (not (heavyWeight ?target_object))
                (not (largeSized ?target_object))
                (emptyHand ?robot_group)
                (openedHand ?robot_group)
                (not (graspedBy ?robot_group ?target_object)) 
                (locatedAt ?robot_group ?initPosition)
                (locatedAt ?target_object ?objectPosition)                
                (forall (?objects - Object) 
                    (and
                        (not (obstruct ?robot_group ?target_object ?objects))
                        (not (graspedBy ?robot_group ?objects))
                    )
                )    
                (forall (?objects - Object)    
                    (and
                        (not (graspedBy ?objects ?target_object))
                    )
                )      
                ; TODO: 
                (or 
                    (exists (?object - Object)    ; if target in container, container should be opened
                        (and
                            (inContGeneric ?target_object ?object)
                            (openedContainer ?object)
                        )
                    ) 
                    (forall (?objects - Object)   ; target not in containers
                        (not (inContGeneric ?target_object ?object))
                    )
                )
            )
        :effect
            (and
                (not (openedHand ?robot_group))
                (graspedBy ?robot_group ?target_object)
                (locatedAt ?robot_group ?objectPosition)
                (not (locatedAt ?robot_group ?initPosition))
            )
		:constraints (?position - controller ?Arm ?Gripper - hardware_group ?grasp - planner)
		:primitives 
            (and      
                (when (not (inWorkspace ?robot_group ?target_object))
                    (and 
                        (?robot_group ?target_object ?initPosition ?objectPosition - approach_base)
                    )
                )                
                (?robot_group ?target_object ?initPosition ?objectPosition - approach_arm)
                (?robot_group ?target_object ?initPosition ?objectPosition - grasp_object)
            )				
	)

    ; hold object using dual-arm
    (:action hold_object_dualarm
		:parameters (?robot_group ?target_object - Object ?initPosition ?objectPosition - Position)
        ; robot_group = [left_arm, right_arm]
        :precondition 
            (and
                (or
                    (heavyWeight ?target_object)
                    (largeSized ?target_object)
                    (twoHandGraspable ?target_object)
                )
                (type ?robot_group DualArm)
                (detectedObject ?target_object)
                (openedHand ?robot_group)
                (forall (?objects - Object) 
                    (and
                        (not (obstruct ?robot_group ?target_object ?objects))
                        (not (graspedBy ?robot_group ?objects))
                    )
                )          
                (forall (?objects - Object) 
                    (and
                        (not (obstruct ?robot_group ?target_object ?objects))
                        (not (graspedBy ?robot_group ?objects))
                        (not (inContGeneric ?target_object ?objects))
                    )
                )    
                (forall (?objects - Object) 
                    (and
                        (not (graspedBy ?objects ?target_object))
                    )
                )
                (locatedAt ?robot_group ?initPosition)  
                (locatedAt ?target_object ?objectPosition)           
                ;(inWorkspace ?robot_group ?target_object)
            )
        :effect
            (and                
                (not (openedHand ?robot_group))
                (graspedBy ?robot_group ?target_object)
                ;(locatedAt ?robot_group ?objectPosition)
            )
		:constraints (?position - controller ?Arm ?Gripper - hardware_group ?grasp - planner)
        :primitives 
            (and 
                ;(when (not (inWorkspace ?robot_group ?target_object))
                ;    (and 
                        (?robot_group ?target_object ?initPosition ?objectPosition - approach_base)
                ;    )
                ;) 
                (?robot_group ?target_object ?initPosition ?objectPosition - approach_dualarm)
                (?robot_group ?target_object ?initPosition - grasp_object)
            )
	)

    ;move arm
    (:action move_arm
		:parameters (?robot_group ?target_object - Object ?initPosition ?targetPosition - Position)
        :precondition
            (and
                (primitive)
            )
        :effect
            (and
            )
		:constraints (?position - controller ?Arm - hardware_group ?movearm - planner)
	)

    ;move arm for the pre-grasping with single arm
	(:action approach_arm
		:parameters (?robot_group ?target_object - Object ?initPosition ?targetPosition - Position)
        :precondition
            (and
                (primitive)
                (type ?robot_group Arm)
                (openedHand ?robot_group)
                (detectedObject ?target_object)
                (not (= ?initPosition ?targetPosition))
                (not (graspedBy ?robot_group ?target_object))
                (locatedAt ?robot_group ?initPosition)
                (locatedAt ?target_object ?targetPosition)
                (forall (?objects - Object) 
                    (not (obstruct ?robot_group ?target_object ?objects))
                )
                (inWorkspace ?robot_group ?target_object)
            )
        :effect
            (and
                (not (locatedAt ?robot_group ?initPosition))
                (locatedAt ?robot_group ?targetPosition)
                (not (emptyHand ?robot_group))
            )
		:constraints (?position - controller ?Arm - hardware_group ?approacharm - planner)
	)

    ;move dual arm for the pre-grasping 
	(:action approach_dualarm
		:parameters (?robot_group ?target_object - Object ?initPosition ?targetPosition - Position)
        :precondition
            (and
            )
        :effect
            (and
            )
		:constraints (?position - controller ?Arm - hardware_group ?approachdualarm - planner)
	)

    ; move base to hold target object with gripper
	(:action approach_base
    :parameters (?robot_group ?target_object - Object ?initPosition ?targetPosition - Position)
    :precondition
        (and           
            (primitive)
            (type ?robot_group Arm)
            (not (type ?target_object Gripper))
            (detectedObject ?target_object)
            (not (= ?robot_group ?target_object))
            (locatedAt ?robot_group ?initPosition)
            (locatedAt ?target_object ?targetPosition)  
            (not (inWorkspace ?robot_group ?target_object))
            (not (graspedBy ?robot_group ?target_object))
        )    
    :effect
        (and
            (inWorkspace ?robot_group ?target_object)
        )
		:constraints (?position - controller ?Mobile - hardware_group ?approachbase - planner)
	)

    ; grasp object just closing gripper
	(:action grasp_object
		:parameters (?robot_group ?target_object - Object ?initPosition - Position)
        :precondition
            (and
                (primitive)
                (or
                    (type ?robot_group Arm)
                    (type ?robot_group DualArm)
                )
                (openedHand ?robot_group)
                (not (emptyHand ?robot_group))
                (not (graspedBy ?robot_group ?target_object)) 
                (locatedAt ?robot_group ?initPosition)
                (locatedAt ?target_object ?initPosition)
            )    
        :effect
            (and
                (not (openedHand ?robot_group)) 
                (graspedBy ?robot_group ?target_object) 
            )
		:constraints (?position - controller ?Gripper - hardware_group ?graspobject - planner)
		:primitives 
            (and 
                (?robot_group - close_hand)
            )	
	)

    ; relocate obstacle to hold target object
    (:action relocate_obstacle
		:parameters (?robot_group - Object ?target_object1 - Object ?target_object2 - Object ?target_object3 - Object ?initPosition ?goalPosition - Position)
        ;target_object1 : graping target
        ;target_object2 : obstacle
        ;target_object3 : workspace
        :precondition 
        (and  
            (type ?robot_group Arm)
            (not (openedHand ?robot_group))
            (detectedObject ?target_object1)
            (detectedObject ?target_object2)
            (locatedAt ?robot_group ?initPosition)
            (locatedAt ?target_object2 ?initPosition)            
            (= ?goalPosition RelocatePosition)
            (not (locatedAt ?robot_group ?goalPosition)) 
            (not (locatedAt ?target_object2 ?goalPosition)) 
            (graspedBy ?robot_group ?target_object2)
            (obstruct ?robot_group ?target_object1 ?target_object2)  
			(or
				(incontgeneric ?target_object2 ?target_object3)
				(onPhysical ?target_object2 ?target_object3)
			) 
        )
        :effect 
        (and 
            (not (locatedAt ?robot_group ?initPosition))
            (locatedAt ?robot_group ?goalPosition)
            (locatedAt ?target_object2 ?goalPosition)
            (not (locatedAt ?target_object2 ?initPosition))
            (not (obstruct ?robot_group ?target_object1 ?target_object2))
        )
		:constraints (?position - controller ?Arm - hardware_group ?relocateobstacle - planner)
	)

    ; take out object from container object
    (:action takeout_object
    :parameters (?robot_group1 ?robot_group2 ?target_object1 ?target_object2 - Object ?mobile_position ?arm_position ?container_position ?target_position - Position)
    ; robot_group1 = mobile
    ; robot_group2 = single arm
    ; target_object1 = target object
    ; target_object2 = container
    :precondition
        (and            
            (type ?robot_group1 Mobile)
            (type ?robot_group2 Arm)
            (not (type ?target_object1 Arm))
            (not (type ?target_object2 Arm))
            (not (= ?robot_group1 ?robot_group2))
            (locatedAt ?robot_group1 ?mobile_position)
            (locatedAt ?robot_group2 ?arm_position)  
            (locatedAt ?target_object2 ?container_position)   
            (locatedAt ?target_object1 ?target_position)   
            ;(openedHand ?robot_group2)
            (inContGeneric ?target_object1 ?target_object2)      
            ;(not (graspedBy ?robot_group2 ?target_object1))    
            (graspedBy ?robot_group2 ?target_object1)
            (openedContainer ?target_object2)                    
            (forall (?objects - Object) 
                (and
                    (not (obstruct ?robot_group2 ?target_object1 ?objects))
                )
            )  
        )    
    :effect
        (and
            ; (when (not (openedContainer ?target_object2))
            ;     (openedContainer ?target_object2)                    
            ; )
            ; (detectedObject ?target_object1)
            ; (inWorkspace ?robot_group2 ?target_object1)
            (graspedBy ?robot_group2 ?target_object1)
            (not (openedHand ?robot_group2))
            (not (emptyHand ?robot_group2))
            (not (inContGeneric ?target_object1 ?target_object2))
            (not (locatedAt ?robot_group2 ?arm_position)) 
            (locatedAt ?robot_group2 ?target_position)   
        )
		:constraints (?position - controller ?Mobile - hardware_group ?takeoutobject - planner)
        :primitives 
            (and
                ; (when (not (openedContainer ?target_object2))
                ;     (and 
                ;         (?robot_group1 ?robot_group2 ?target_object2 ?mobile_position ?arm_position ?container_position - open_container)
                ;     )
                ; )                 
                ;(?robot_group2 ?target_object1 ?arm_position ?target_position - hold_object)
                (?robot_group1 ?initPos [-0.3,0.0,0.0] - move_base)
                (?robot_group2 ?target_object1 - standby_with_object)
            )
	)

    ; put object into container
    (:action putin_object
    :parameters (?robot_group1 ?robot_group2 ?target_object1 ?target_object2 - Object ?mobile_position ?arm_position ?container_position ?target_position - Position)
    ; robot_group1 = mobile
    ; robot_group2 = single arm
    ; target_object1 = target object
    ; target_object2 = container
    :precondition
        (and           
            (type ?robot_group1 Mobile)
            (type ?robot_group2 Arm)
            (not (type ?target_object1 Arm))
            (not (type ?target_object2 Arm))
            (not (= ?robot_group1 ?robot_group2))
            (graspedBy ?robot_group2 ?target_object1)
            (not (inContGeneric ?target_object1 ?target_object2))
            (openedContainer ?target_object2)    
            (locatedAt ?robot_group1 ?mobile_position)
            (locatedAt ?robot_group2 ?arm_position)  
            (locatedAt ?target_object2 ?container_position)   
            (locatedAt ?target_object1 ?target_position)     
        )    
    :effect
        (and            
            (inContGeneric ?target_object1 ?target_object2)
        )
		:constraints (?position - controller ?Mobile - hardware_group ?putinobject - planner)
        :primitives 
            (and              
                (?robot_group2 ?target_object1 - standby_with_object)
                (?robot_group1 ?target_object2 ?mobile_position ?container_position - approach_base)
                (?robot_group2 ?target_object1 ?target_object2 ?container_position ?target_position - putup_object)
                (?robot_group2 - open_hand)
                (?robot_group1 ?initPos [-0.3,0.0,0.0] - move_base)
                (obj_dual_hand - standby)
            )
	) 

    ;open fridge container 
    (:action open_container
    :parameters (?robot_group1 ?robot_group2 ?target_object - Object ?initPos ?armPos ?goal_position - Position)
    ; robot_group1 = mobile
    ; robot_group2 = single arm
    ; target_object = container
    :precondition
        (and
            ;(primitive)
            (type ?robot_group1 Mobile)
            (type ?robot_group2 Arm)
            (not (= ?robot_group1 ?robot_group2))
            (not (openedContainer ?target_object)) 
            (emptyHand ?robot_group2)
            (openedHand ?robot_group2)
            (locatedAt ?robot_group1 ?initPos)
            (locatedAt ?robot_group2 ?armPos)  
            (locatedAt ?target_object ?goal_position)    
        )    
    :effect
        (and           
		    (openedContainer ?target_object)
            (forall (?object - Object)
                (when (inContGeneric ?target_object ?object)
                    (and 
                        (detectedObject ?object)
                        (inWorkspace ?robot_group2 ?object)
                    )
                )
            )
        )
		:constraints (?position - controller ?Mobile - hardware_group ?opencontainer - planner)
        :primitives 
            (and                
                (?robot_group1 obj_fridge_bottom_door ?initPos ?goal_position - approach_base)
                (?robot_group2 ?target_object ?armPos ?goal_position - approach_arm)
                (?robot_group2 ?target_object ?armPos ?goal_position - grasp_object)
                (?robot_group2 ?target_object ?initPos ?goal_position - open_door)
                (?robot_group2 - open_hand)
                (obj_dual_hand - standby)
                ;(?robot_group1 ?target_object ?initPos ?goal_position - approach_base)
            )
	)

    ;close fridge container 
    (:action close_container
    :parameters (?robot_group1 ?robot_group2 ?target_object - Object ?initPos ?armPos ?goal_position - Position)
    ; robot_group1 = mobile
    ; robot_group2 = single arm
    ; target_object = container
    :precondition
        (and
            (type ?robot_group1 Mobile)
            (type ?robot_group2 Arm)
            (not (= ?robot_group1 ?robot_group2))
            (openedContainer ?target_object)
            (emptyHand ?robot_group2)
            (openedHand ?robot_group2)
            (locatedAt ?robot_group1 ?initPos)
            (locatedAt ?robot_group2 ?armPos)  
            (locatedAt ?target_object ?goal_position)    
        )    
    :effect
        (and           
		    (not (openedContainer ?target_object))
            ; (forall (?object - Object)
            ;     (when (inContGeneric ?target_object ?object)
            ;         (and 
            ;             (not (detectedObject ?object))
            ;             (not (inWorkspace ?robot_group2 ?object))
            ;         )
            ;     )
            ; )
        )
		:constraints (?position - controller ?Mobile - hardware_group ?closecontainer - planner)
        :primitives 
            (and                
                (?robot_group1 obj_fridge_bottom_door ?initPos ?goal_position - approach_base)
                (?robot_group2 ?target_object ?armPos ?goal_position - approach_arm)
                (?robot_group2 ?target_object ?armPos ?goal_position - grasp_object)
                (?robot_group2 ?target_object ?initPos ?goal_position - close_door)
                (?robot_group2 - open_hand)
                (obj_dual_hand - standby)
            )
	)

    ; open fridge door
    (:action open_door
    :parameters (?robot_group ?target_object - Object ?current_position ?goal_position - Position)
    :precondition
        (and      
            (primitive)        
        )    
    :effect
        (and 
        )
		:constraints (?position - controller ?Mobile - hardware_group ?opendoor - planner)
    )

    ; close fridge door
    (:action close_door
    :parameters (?robot_group ?target_object - Object ?current_position ?goal_position - Position)
    :precondition
        (and      
            (primitive)        
        )    
    :effect
        (and 
        )
		:constraints (?position - controller ?Mobile - hardware_group ?closedoor - planner)
    )

    ; move robot base from to
    (:action move_base
		:parameters (?robot_group - Object ?current_position ?goal_position - Position)
        :precondition 
            (and   
                (primitive)
            )
        :effect
            (and
            ) 
		:constraints (?position - controller ?Mobile - hardware_group ?movebase - planner)  
	)

    ; puton object to other object
    (:action stackup_object
    :parameters (?robot_group ?target_object1 ?target_object2 - Object  ?current_position ?target_position - Position)
    ; above : target_object1
    ; below : target_object2
    :precondition
        (and            
            (not (type ?target_object1 Arm))
            (not (type ?target_object2 Arm))
            (graspedBy ?robot_group ?target_object1)
            (not (belowOf ?target_object2 ?target_object1))
            (locatedat ?robot_group ?current_position) 
            (locatedat ?target_object1 ?current_position) 
            (locatedat ?target_object2 ?target_position) 
			(forall (?objects - object)
				(and
					(not (graspedby ?objects ?target_object2))
				)
			)
            (forall (?objects - object)
				(and
					(not (delivered ?target_object1 ?objects))
				)
			)    
            (forall (?objects - object)
				(and
					(not (inContGeneric ?target_object1 ?objects))
				)
			)       
            (not
                (exists (?object - object)
                    (and
                        (delivered ?target_object2 ?object)
                    )
                )
            )   
        )    
    :effect
        (and            
            (belowOf ?target_object2 ?target_object1)
            (not (locatedat ?robot_group ?current_position))
            (not (locatedat ?target_object1 ?current_position))
            (openedHand ?robot_group)
        )
		:constraints (?position - controller ?Arm - hardware_group ?stackupobject - planner)
        :primitives 
            (and 
                (when (and (not (inWorkspace ?robot_group ?target_object2)) (largeSized ?target_object2))
                    (and 
                        (obj_dual_hand ?target_object2 ?current_position ?target_position - approach_base)
                    )
                )
                (when (and (not (inWorkspace ?robot_group ?target_object2)) (not (largeSized ?target_object2)))
                    (and 
                        (?robot_group ?target_object2 ?current_position ?target_position - approach_base)
                    )
                )
                (?robot_group ?target_object1 ?target_object2 ?current_position ?target_position - putup_object)
                (?robot_group - open_hand)
            )
	)

    (:action putup_object
    :parameters (?robot_group ?target_object1 ?target_object2 - Object  ?current_position ?target_position - Position)
    :precondition
        (and      
                (primitive)        
        )    
    :effect
        (and 
        )
		:constraints (?position - controller ?Arm - hardware_group ?putupobject - planner)
    )

    (:action pour_object
    :parameters (?robot_group ?target_object1 ?target_object2 - Object  ?current_position ?target_position ?container_position - Position)
    :precondition
        (and            
            (type ?robot_group Arm) 
            (locatedat ?robot_group ?current_position) 
            (locatedat ?target_object1 ?target_position) 
            (locatedat ?target_object2 ?container_position) 
            (graspedBy ?robot_group ?target_object1)
        )    
    :effect
        (and            
            (inContGeneric ?target_object2 ?target_object1)
            (not (locatedat ?robot_group ?current_position))
            (not (locatedat ?target_object1 ?target_position))
        )
		:constraints (?position - controller ?Arm - hardware_group ?pourobject - planner)
	)

    (:action handover_object
    :parameters (?robot_group ?target_object1 - Object)
    :precondition
        (and    
                (primitive)                            
        )    
    :effect
        (and 
            (handedover ?robot_group ?target_object1)
        )
		:constraints (?position - controller ?Arm - hardware_group ?handoverobject - planner)
    )


    (:action transfer_object
    :parameters (?robot_group ?target_object1 ?target_object2 - Object  ?current_position ?target_position - Position)
    :precondition
        (and      
                (primitive)        
        )    
    :effect
        (and 
        )
		:constraints (?position - controller ?Arm - hardware_group ?transferobject - planner)
    )

    (:action deliver_object
    :parameters (?robot_group ?target_object1 ?target_object2 - Object  ?current_position ?target_position - Position)
    :precondition
        (and      
            (or
                (type ?robot_group DualArm) 
                (type ?robot_group Arm) 
            )
            (graspedBy ?robot_group ?target_object1)
            (locatedAt ?target_object2 ?target_position)
        )    
    :effect
        (and 
            (delivered ?target_object1 ?target_object2)
            (forall (?object - Object)
                (when (belowOf ?target_object1 ?object)
                    (and 
                        (delivered ?object ?target_object2)
                    )
                )
            )
        )
		:constraints (?position - controller ?Arm - hardware_group ?deliverobject - planner)
        :primitives 
            (and
                (?robot_group ?target_object2 ?current_position ?target_position - approach_base)
                (?robot_group ?target_object1 ?target_object2 ?current_position ?target_position - transfer_object)
            )
    )

    ; pushing object with dual arm
    ; (:action pushing_object_dualarm
    ; :parameters (?robot_group ?target_object1 ?target_object2 - Object  ?current_position ?target_position - Position)
    ; :precondition
    ;     (and      
    ;         (or
    ;             (type ?robot_group DualArm) 
    ;         )
    ;     )    
    ; :effect
    ;     (and 
    ;     )
	; 	:constraints (?position - controller ?Arm - hardware_group ?deliverobject - planner)
    ;     :primitives 
    ;         (and
    ;             (?robot_group ?target_object2 ?current_position ?target_position - hold_object_dualarm)
    ;             ( - pushing)
    ;             ( - move_base)
    ;         )
    ; )
)
