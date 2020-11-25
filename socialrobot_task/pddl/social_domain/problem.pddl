(define 
    (problem pick_and_place) 
    (:domain skku_robot)
    (:objects
        left_hand right_hand - Object
        juice milk water table - Object
        pos_juiceInit pos_milkInit posWaterInit - Position
        pos_leftArmInit pos_rightArmInit - Position
        temp_position - Position
    )        
    (:init        
        ; Define        
        (type left_hand Gripper) (type right_hand Gripper)
        (inWorkspace left_hand TempPosition) 

        ; Perception
        (onPhysical juice table) (onPhysical milk table) (onPhysical water table)
        (emptyHand left_hand) (emptyHand right_hand)
        (locatedAt milk pos_milkInit) (locatedAt juice pos_juiceInit) (locatedAt water posWaterInit)
        (locatedAt left_hand pos_leftArmInit) (locatedAt right_hand pos_rightArmInit)        
        (detected milk) (detected juice) (detected water)
        
        ; Reasoner
        (obstruct left_hand milk juice) ;(obstruct left_hand milk water) 
        (inWorkspace left_hand pos_juiceInit) (inWorkspace left_hand pos_milkInit) (inWorkspace left_hand posWaterInit) 
    )
    (:goal   
        (and
            ;가로막고 있는 장애물 옮기고 목표물체 잡기
            (graspedBy left_hand milk)
        )
        
    )
)