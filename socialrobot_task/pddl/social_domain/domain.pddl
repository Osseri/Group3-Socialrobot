; This one describe the Socialrobot demo scenario
(define (domain skku_robot)
  (:requirements 
        :strips                     ; basic preconditions and effects
        :typing                     ; to define type of objects and parameters
        :disjunctive-preconditions 
        :negative-preconditions     ; to use not in preconditions
        :equality                   ; to use = in preconditions
        :adl                        ; 
    )

    ; Define type of object and parameters
	(:types
		Object  ; 물체
        Position   ; 3차원 x,y,z 위치  
	)
    (:constants
        RobotBody Arm Mobile Gripper - Object      ; 로봇 구성요소
        TempObject Container - Object ; 물체 특성/종류
        TempPosition AboveOf BelowOf FrontOf BehindOf Near - Position ; 임의/상대적 위치
    )
    
    ; Define the relations
    ; Question mark prefix denotes free variables
	(:predicates
        (type ?obj ?type)                               ;Object의 클래스 타입을 서술
        (onPhysical ?obj ?supportPlane)                ;Object가 작업평면 위에 놓여 있음을 서술
        (emptyHand ?robotGripper)                      ;Hand가 비어 있음을 서술
        (openedHand ?robotGripper )                      ;Hand가 열려 있음을 서술
        (inWorkspace ?robotBase - Object ?target - Position)         ;위치나 물체가 로봇의 workspace안에 존재
        (graspable ?hand ?obj - object)                 ;grasp pose가 존재
        (accesible ?hand ?obj - object)                 ;move arm를 위한 path가 clear
        (graspedBy ?hand ?obj - object)                 ;hand에 의해 obj가 파지됨
        (obstruct ?hand ?target ?obs - object)          ;hand가 target을 파지하는데 가로막는 obs가 존재
        (detected ?obj - object)                        ;perception에 인지된 물체
        (clear ?position)                               ;해당위치에 아무것도 없음
        (locatedAt ?obj - Object ?loc - Position)
        (inContGeneric ?container ?content - object)    ;content를 container가 담고있음
        (aboveOf ?obj1 ?obj2 - object)                  ;Object1이 Object2 위에 있음을 서술
        (belowOf ?obj1 ?obj2 - object)
        (FrontOf ?obj1 ?obj2 - object)
        (behindOf ?obj1 ?obj2 - object)
        (near ?obj1 ?obj2 - object)
        (emptyContainer ?container - object)                     ;Container가 비어 있음을 서술 
	)
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;; Define actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;


    ;;;; actions for gripper & manipulagoalPositionr
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:action open_hand
		:parameters (?robotGripper - Object)
        :precondition 
            (and 
                (type ?robotGripper Gripper)         
                (not (openedHand ?robotGripper))   
            )
        :effect
            (and
                (openedHand ?robotGripper)
            )
	)

    (:action close_hand
		:parameters (?robotGripper - Object)
        :precondition 
            (and 
                (type ?robotGripper Gripper) 
                (openedHand ?robotGripper)              
            )
        :effect
            (and
                (not (openedHand ?robotGripper))
            )
	)
    
	(:action release_object
		:parameters (?robotGripper ?targetObject - Object ?eefPosition - Position)
        :precondition 
            (and
                ;precondition of open_hand
                (type ?robotGripper Gripper)
                (not (openedHand ?robotGripper)) 

                ;additional effect
                (not (emptyHand ?robotGripper))
                (graspedBy ?robotGripper ?targetObject) 
                (locatedAt ?robotGripper ?eefPosition)
                (locatedAt ?targetObject ?eefPosition)
            )
        :effect
            (and
                ;effect of open_hand
                (openedHand ?robotGripper)

                ;additional effect
                (emptyHand ?robotGripper)
                (not (graspedBy ?robotGripper ?targetObject))
            )
	)

	(:action grasp_object
		:parameters (?robotGripper ?targetObject - Object ?eefPosition ?targetPosition - Position)
        :precondition 
            (and
                ;precondition of close_hand
                (type ?robotGripper Gripper)
                (openedHand ?robotGripper)
                
                ;additional precondition
                (emptyHand ?robotGripper)
                (not (graspedBy ?robotGripper ?targetObject)) 
                (locatedAt ?robotGripper ?eefPosition)
                (locatedAt ?targetObject ?targetPosition)                
                (forall (?objects - Object) 
                    (not (obstruct ?robotGripper ?targetObject ?objects))
                )
            )
        :effect
            (and
                ;effect of close_hand
                (not (openedHand ?robotGripper))

                ;additional effect
                (not (emptyHand ?robotGripper))
                (graspedBy ?robotGripper ?targetObject)
                (locatedAt ?robotGripper ?targetPosition)
                (not (locatedAt ?robotGripper ?eefPosition))
            )
	)

    (:action move_arm
		:parameters (?targetArm - Object ?eefPosition ?goalPosition - Position)
        :precondition 
            (and                
                (type ?targetArm Gripper)        
                (locatedAt ?targetArm ?eefPosition)
                (forall (?objects - Object) 
                    (and
                        (not (locatedAt ?objects ?goalPosition))
                    )
                )
                (inWorkspace ?targetArm ?goalPosition)
            )
        :effect
            (and
                (not (locatedAt ?targetArm ?eefPosition))
                (locatedAt ?targetArm ?goalPosition)
            )
	)

    ;targetObject을 가로막는 obstuctObject를 eefPosition에서 goalPosition로 이동
    (:action relocate_object
       :parameters (?targetArm - Object ?obstuctObject - Object ?eefPosition ?goalPosition - Position)
        :precondition 
        (and 
            ;preconditions of move_arm
            (type ?targetArm Gripper)        
             (locatedAt ?targetArm ?eefPosition)
             (locatedAt ?obstuctObject ?eefPosition)
            
            ;additional preconditions
            (or 
                (= ?goalPosition TempPosition)
                (forall (?objects - Object) 
                    (not (locatedAt ?objects ?goalPosition))
                )
            )
            (not (locatedAt ?targetArm ?goalPosition)) 
            (not (locatedAt ?obstuctObject ?goalPosition)) 
            (graspedBy ?targetArm ?obstuctObject)
        )
        :effect 
        (and 
            ;effect of move_arm
            (not (locatedAt ?targetArm ?eefPosition))
            (locatedAt ?targetArm ?goalPosition)

            ;additional effect
            (locatedAt ?obstuctObject ?goalPosition)
            (not (locatedAt ?obstuctObject ?eefPosition))
            (forall (?objects - Object) 
                (when  (and (obstruct ?targetArm ?objects ?obstuctObject))            
                    (not (obstruct ?targetArm ?objects ?obstuctObject))
                )
            )
        )
    )

)
